#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "pwm_bcm2708.h"
#include "ws2812_bcm2708.h"

static struct ws2812_array colors_initial = {
  .colors = {
    { 125,   0,   0},
    {   0, 125,   0},
    {   0,   0, 125},
    { 125, 125,   0},
    {   0, 125, 125},
    { 125, 125,   0}
  },
  .count = 6
};

/* To quickly clear the LEDs */
static struct ws2812_array colors_fini = {
  .colors = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  },
  .count = 6
};


int main(int argc, char** argv) {
  if (argc != 2)
    return 0;

  printf("Initial LED setup\n");

  struct pwm_bwm2708_dev* pwm = pwm_bwm2708_create(argv[1]);
  struct ws28128_bcm2708 out;

  ws28128_bcm2708_render(&colors_initial, &out);
  pwm_bwm2708_write_data(pwm, out.buffer, out.size);
  free(out.buffer); /* TODO: horrible */

  printf("Sleeping 1 second...\n");
  sleep(1);
  
  printf("Cleaning up LEDs with darkness...\n");
  ws28128_bcm2708_render(&colors_fini, &out);
  pwm_bwm2708_write_data(pwm, out.buffer, out.size);
  free(out.buffer); /* TODO: horrible */

  printf("Done!\n");

  pwm_bwm2708_free(pwm);

  return 0;
}
