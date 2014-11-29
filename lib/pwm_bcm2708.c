#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

#include "pwm_bcm2708.h"

struct pwm_bwm2708_dev {
  int fd;
};

struct pwm_bwm2708_dev* pwm_bwm2708_create(const char* device_path) {
  int fd = open(device_path, O_WRONLY);
  if (fd == -1) {
    perror("pwm_bwm2708_create error");
    return NULL;
  }

  struct pwm_bwm2708_dev* pwm = (struct pwm_bwm2708_dev*) malloc(sizeof(struct pwm_bwm2708_dev*));
  pwm->fd = fd;

  return pwm;
}

void pwm_bwm2708_free(struct pwm_bwm2708_dev* dev) {
  if (!dev)
    return;

  close(dev->fd);
  free(dev);
}

int pwm_bwm2708_write_data(struct pwm_bwm2708_dev* dev, void* data, size_t len) {
  size_t written = write(dev->fd, data, len);

  if (written != len)
    return -1;

  return 0;
}
