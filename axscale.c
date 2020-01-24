/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <libevdev/libevdev.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/signal.h>
#include <sys/signalfd.h>
#include <unistd.h>

#define cleanup(func) __attribute__((cleanup(func)))

void *stealp(void *p) {
  void **ptr = p;
  void *tmp = ptr;
  *ptr = NULL;
  return tmp;
}

#define DEFINE_GENERIC_CLEANUP_FUNC(name, type, deleter, empty) \
  void name(type *ptr) { \
    if (*ptr != empty) { \
      deleter(*ptr); \
      *ptr = empty; \
    } \
  } \

DEFINE_GENERIC_CLEANUP_FUNC(closep, int, close, -1)
DEFINE_GENERIC_CLEANUP_FUNC(closedirp, DIR *, closedir, NULL)
DEFINE_GENERIC_CLEANUP_FUNC(fclosep, FILE *, fclose, NULL)

void libevdev_freep(struct libevdev **dev) {
  if (*dev != NULL) {
    int fd = libevdev_get_fd(*dev);
    if (fd != -1) {
      if (close(fd) == -1) {
        eprintln("closing fd %d: %s", fd, strerror(errno));
      }
    }

    libevdev_free(*dev);
    *dev = NULL;
  }
}

#define streq(s1, s2) (strcmp((s1), (s2)) == 0)

#define println(msg, ...) printf(msg "\n", ##__VA_ARGS__)
#define eprintln(msg, ...) fprintf(stderr, msg "\n", ##__VA_ARGS__)

#define AXIS_A ABS_X
#define AXIS_Z ABS_RZ
#define AXIS_COUNT (AXIS_Z - AXIS_A + 1)

typedef struct axis_range {
  bool present;
  unsigned int min;
  unsigned int max;
} axis_range;

#define MAP_FILE_LINE_FORMAT "axis %d: min = %u, max = %u\n"

struct libevdev *open_device(const char *path) {
  int fd = open(path, O_RDWR);
  if (fd == -1) {
    eprintln("opening device %s: %s", path, strerror(errno));
    return NULL;
  }

  struct libevdev *dev = libevdev_new();
  int ret = libevdev_set_fd(dev, fd);
  if (ret < 0) {
    eprintln("binding to device %s: %s", path, strerror(-ret));
    libevdev_free(dev);
    close(fd);
    return NULL;
  }

  return dev;
}

bool detect(struct libevdev *dev, const char *map_file) {
  axis_range axes_ranges[AXIS_COUNT] = {0};

  for (int axis = AXIS_A; axis <= AXIS_Z; axis++) {
    if (libevdev_has_event_code(dev, EV_ABS, axis)) {
      axis_range *range = &axes_ranges[axis - AXIS_A];
      range->present = true;
      range->max = 0;
      // max value I usually see
      range->min = USHRT_MAX;
    }
  }

  cleanup(fclosep) FILE *fp = fopen(map_file, "w");
  if (fp == NULL) {
    eprintln("opening %s: %s", map_file, strerror(errno));
    return false;
  }

  sigset_t sigint_mask;
  sigemptyset(&sigint_mask);
  sigaddset(&sigint_mask, SIGINT);

  if (sigprocmask(SIG_BLOCK, &sigint_mask, NULL) == -1) {
    perror("sigprocmask(SIG_BLOCK)");
    return false;
  }

  cleanup(closep) int sigint_fd = signalfd(-1, &sigint_mask, 0);
  if (sigint_fd == -1) {
    perror("signalfd(SIGINT)");
    return false;
  }

  struct pollfd fds[2];
  fds[0].fd = libevdev_get_fd(dev);
  fds[1].fd = sigint_fd;
  fds[0].events = fds[1].events = POLLIN;

  println("Please slowly move all joysticks in a full circle at least once");
  println("Press Ctrl-C when complete");

  for (;;) {
    if (poll(fds, sizeof(fds) / sizeof(fds[0]), -1) == -1) {
      perror("poll");
      return false;
    }

    if (fds[1].revents & POLLIN) {
      for (int axis = AXIS_A; axis <= AXIS_Z; axis++) {
        axis_range *range = &axes_ranges[axis - AXIS_A];
        if (range->present) {
          if (fprintf(fp, MAP_FILE_LINE_FORMAT, axis, range->min, range->max) < 0) {
            eprintln("writing line to %s: %s", map_file, strerror(errno));
            return false;
          }
        }
      }

      return true;
    } else if (fds[0].revents & POLLIN) {
      struct input_event event;
      int ret = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &event);
      if (ret < 0) {
        eprintln("reading next event: %s", strerror(-ret));
        return false;
      } else if (ret == LIBEVDEV_READ_STATUS_SYNC) {
        // Just ignore it, this is just for basic calibration purposes anyway.
        continue;
      }

      if (event.type != EV_ABS || event.code < AXIS_A || event.code > AXIS_Z) {
        continue;
      }

      axis_range *range = &axes_ranges[event.code - AXIS_A];
      if (!range->present) {
        eprintln("INTERNAL ERROR: axis %d is marked as non-present but sent events",
                 event.code);
        return false;
      }

      if (event.value > range->max) {
        range->max = event.value;
      }
      if (event.value < range->min) {
        range->min = event.value;
      }
    } else {
      println("unknown error while polling");
      return false;
    }
  }
}

bool load(struct libevdev *dev, const char *map_file) {
  axis_range axes_ranges[AXIS_COUNT] = {0};

  cleanup(fclosep) FILE *fp = fopen(map_file, "r");
  if (fp == NULL) {
    eprintln("opening %s: %s", map_file, strerror(errno));
    return false;
  }

  while (!feof(fp)) {
    int axis;
    unsigned int min, max;
    if (fscanf(fp, MAP_FILE_LINE_FORMAT, &axis, &min, &max) < 0) {
      eprintln("reading line from %s: %s", map_file, strerror(errno));
    }

    axis_range *range = &axes_ranges[axis - AXIS_A];
    range->present = true;
    range->min = min;
    range->max = max;
  }

  for (int axis = AXIS_A; axis <= AXIS_Z; axis++) {
    axis_range *range = &axes_ranges[axis - AXIS_A];
    if (!range->present) {
      continue;
    }

    if (!libevdev_has_event_code(dev, EV_ABS, axis)) {
      eprintln("axis %d exists in mapping but not in device", axis);
      return false;
    }

    libevdev_set_abs_minimum(dev, axis, range->min);
    libevdev_set_abs_maximum(dev, axis, range->max);
  }

  return true;
}

int main(int argc, char **argv) {
  if (argc > 1 && (streq(argv[1], "-h") || streq(argv[1], "--help"))) {
    println("usage: %s [detect|load] <joystick> <map file>", argv[0]);
    return 0;
  }

  if (argc != 4) {
    eprintln("wrong number of arguments");
    return 1;
  }

  cleanup(libevdev_freep) struct libevdev *dev = open_device(argv[2]);
  if (dev == NULL)  {
    return 1;
  }

  if (streq(argv[1], "detect")) {
    return !detect(dev, argv[3]);
  } else if (streq(argv[1], "load")) {
    return !load(dev, argv[3]);
  } else {
    eprintln("invalid command: %s", argv[1]);
    return 1;
  }
}
