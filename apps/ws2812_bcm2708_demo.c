#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "pwm_bcm2708.h"
#include "ws2812_bcm2708.h"

struct ws2812_array clean_colors = {
  .count = 16
};

static void output_list(struct ws2812_array* list, struct pwm_bwm2708_dev* pwm, struct ws28128_bcm2708* out) {
  ws28128_bcm2708_render(list, out);
  pwm_bwm2708_write_data(pwm, out->buffer, out->size);
  free(out->buffer); /* TODO: horrible */
}

int main(int argc, char** argv) {
  if (argc != 2)
    return 0;

  printf("Initial LED setup\n");

  struct pwm_bwm2708_dev* pwm = pwm_bwm2708_create(argv[1]);
  struct ws28128_bcm2708 out;
  struct ws2812_array colors = clean_colors;
  struct timespec timer;
  timer.tv_sec = 0;
  timer.tv_nsec = 12500000l; /* 0.0125s */

  for (int i = 0; i < 16; i++) {
    struct ws2812_color color = { 50, 0, i*50/16 };
    colors.colors[i] = color;
    output_list(&colors, pwm, &out);
    nanosleep(&timer, NULL);
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 16; i++) {
      struct ws2812_color color = { j*50, 0, j*50 };
      colors.colors[i] = color;
      output_list(&colors, pwm, &out);
      nanosleep(&timer, NULL);
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 16; i++) {
      struct ws2812_color color = { 150, j*50, 150-j*50 };
      colors.colors[i] = color;
      output_list(&colors, pwm, &out);
      nanosleep(&timer, NULL);
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 16; i++) {
      struct ws2812_color color = { 150-j*50, 150-j*50, 0 };
      colors.colors[i] = color;
      output_list(&colors, pwm, &out);
      nanosleep(&timer, NULL);
    }
  }

  /* Smileys */
  struct ws2812_color smiley_colors[4] = {
    {0,0,150}, {0,150,0}, {150,0,0}, {0,0,150}
  };
  for (int i = 0; i < 4; i++) {
    colors = clean_colors;
    colors.colors[6] = colors.colors[1] = smiley_colors[i];
    colors.colors[14] = colors.colors[13] = colors.colors[12] = colors.colors[11] = colors.colors[10] = colors.colors[9] = smiley_colors[i];
    output_list(&colors, pwm, &out);
    sleep(1);
  }

  printf("Sleeping 1 second...\n");
  sleep(1);
  
  printf("Cleaning up LEDs with darkness...\n");
  output_list(&clean_colors, pwm, &out);

  printf("Done!\n");

  pwm_bwm2708_free(pwm);

  return 0;
}
