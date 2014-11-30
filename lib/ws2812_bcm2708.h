#ifndef WS2812_BCM2708_H_
#define WS2812_BCM2708_H_

#include <stddef.h> /* size_t */
#include <stdint.h>

/* Serialize single color, as GREEN RED BLUE */
struct ws2812_color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

/* TODO */
struct ws2812_array {
  struct ws2812_color colors[16];
  size_t count;
};


struct ws28128_bcm2708 {
  void* buffer;
  size_t buffer_size;
  size_t size;
};

void ws28128_bcm2708_render(struct ws2812_array* array, struct ws28128_bcm2708* out);

#endif /* WS2812_BCM2708_H_ */
