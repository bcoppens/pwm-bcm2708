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
}

int main(int argc, char** argv) {
  if (argc != 2)
    return 0;

  printf("Initial LED setup\n");

  struct pwm_bwm2708_dev* pwm = pwm_bwm2708_create(argv[1]);
  struct ws28128_bcm2708 out = { .buffer = NULL };
  struct ws2812_array colors = clean_colors;

  /* Set up some colors */
  for (int i = 0; i < 16; i++) {
    struct ws2812_color color = { 50, 0, i*50/16 };
    colors.colors[i] = color;
  }

  /* Strobe between the colors and blackness as fast as possible */
  printf("Strobing...\n");

  for (int i = 0; i < 1000; i++) {
    output_list(&colors, pwm, &out);
    output_list(&clean_colors, pwm, &out);
  }

  printf("Done!\n");

  free(out.buffer);
  pwm_bwm2708_free(pwm);

  return 0;
}
