#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <time.h>

#include "pwm_bcm2708.h"
#include "ws2812_bcm2708.h"

using namespace std;
using namespace chrono;

static ws2812_array clean_colors;

const int ring_size = 16;
const int ms_per_frame = 125;
const int max_brightness = 140;

/* I want 0 to be on the top, 1 to the right of it, etc..., so this maps
 * those coordinates to the ones that are in use on the current orientation
 * of the LED ring */
struct LEDMapper {
  bool flip;
  int  offset;

  LEDMapper(bool flip, int offset) : flip(flip), offset(offset) {}
  int operator()(int n) {
    if (flip)
      n = ring_size - n;

    return (n + offset) % ring_size;
  }
};

LEDMapper mapper(true, 3);

static void output_list(struct ws2812_array& list, struct pwm_bwm2708_dev* pwm, struct ws28128_bcm2708& out) {
  ws28128_bcm2708_render(&list, &out);
  pwm_bwm2708_write_data(pwm, out.buffer, out.size);
}

template<typename T>
static inline T clamp(T value, T min, T max) {
  if (value > max)
    return max;
  if (value < min)
    return min;
  return value;
}

static ws2812_color color_add(const ws2812_color& c1, const ws2812_color& c2) {
  return { uint8_t(clamp(c1.red + c2.red,     0, max_brightness)),
           uint8_t(clamp(c1.green + c2.green, 0, max_brightness)),
           uint8_t(clamp(c1.blue + c2.blue,   0, max_brightness)) };
}

static ws2812_color color_fraction(const ws2812_color& color, float fraction) {
  return { uint8_t(color.red   * fraction),
           uint8_t(color.green * fraction),
           uint8_t(color.blue  * fraction) };
}

static void add_blend_at(ws2812_array& list, const ws2812_color& color, float pos) {
  int low_pos  = mapper(floor(pos));
  int next_pos = mapper((int(floor(pos)) + 1) % ring_size);

  float mix_high = pos - floor(pos);
  float mix_low  = 1.0 - mix_high;

  /* TODO: this might look nicer with operator overloading */
  list.colors[low_pos]  = color_add(color_fraction(color, mix_low), list.colors[low_pos]);
  list.colors[next_pos] = color_add(color_fraction(color, mix_high), list.colors[next_pos]);
}

ws2812_color hours_color   = { 130,   0,  0 };
ws2812_color minutes_color = {   0, 110,  0 };
ws2812_color seconds_color = {   0,   0, 90 };

/* The time positions shown are continuous: the position of hour smoothly
 * advances 1/12th of the ring during an hour.
 * To do this, we get the current hour, and subtract the current time from that. We reset
 * the current hour once an hour :)
 */
static bool last_hour_set = false;
static int current_hour = 0;
static system_clock::time_point last_hour;

void set_last_hour() {
  time_t time = system_clock::to_time_t(system_clock::now());
  tm time_tm;

  localtime_r(&time, &time_tm);
  time_tm.tm_min = 0;
  time_tm.tm_sec = 0;

  last_hour = system_clock::from_time_t(mktime(&time_tm));
  current_hour = time_tm.tm_hour;
}

void set_colors_for_now(ws2812_array& colors) {
  set_last_hour();
  auto ms = duration_cast<milliseconds>(system_clock::now() - last_hour);

  /* Non-monotonic clock ... */
  if (ms < milliseconds(0))
    ms = milliseconds(0);
  /* Update for the next hour */
  if (ms >= duration_cast<milliseconds>(hours(1))) {
    set_last_hour();
    ms = milliseconds(0);
  }

  /* TODO: this looks not really nice, although the above trickery also doesn't feel well */
  float hours_pos   = float(current_hour) + float(ms.count()) / 60.0 / 60.0 / 1000.0;
  float minutes_pos = float(ms.count()) / 60.0 / 1000.0;
  float seconds_pos = float(ms.count() - 60*1000*floor(minutes_pos)) / 1000.0;

  /* No 24h hours */
  if (hours_pos >= 12.0)
    hours_pos -= 12;

  colors = clean_colors;
  add_blend_at(colors, hours_color, hours_pos * float(ring_size)/12.0);
  add_blend_at(colors, minutes_color, minutes_pos * float(ring_size)/60.0);
  add_blend_at(colors, seconds_color, seconds_pos * float(ring_size)/60.0);
}

int main(int argc, char** argv) {
  if (argc != 2)
    return 0;

  clean_colors.count = 16;

  struct pwm_bwm2708_dev* pwm = pwm_bwm2708_create(argv[1]);
  ws28128_bcm2708_init(pwm);

  struct ws28128_bcm2708 out = { .buffer = NULL };
  struct ws2812_array colors = clean_colors;

  milliseconds rest(ms_per_frame);

  while(true) {
    set_colors_for_now(colors);
    output_list(colors, pwm, out);

    this_thread::sleep_for(rest);
  }

  free(out.buffer);
  pwm_bwm2708_free(pwm);

  return 0;
}
