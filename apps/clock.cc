#include <cmath>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "pwm_bcm2708.h"
#include "ws2812_bcm2708.h"

using namespace std;

static ws2812_array clean_colors;

const int ring_size = 16;
const int ms_per_frame = 125;
const int frames_per_min = 60*1000/ms_per_frame;

static void output_list(struct ws2812_array* list, struct pwm_bwm2708_dev* pwm, struct ws28128_bcm2708* out) {
  ws28128_bcm2708_render(list, out);
  pwm_bwm2708_write_data(pwm, out->buffer, out->size);
}

static ws2812_color color_fraction(const ws2812_color& color, float fraction) {
  return { uint8_t(color.red * fraction),
           uint8_t(color.green * fraction),
           uint8_t(color.blue * fraction) };
}

static void insert_blended_at(ws2812_array& list, const ws2812_color& color, float pos) {
  int low_pos  = floor(pos);
  int next_pos = (low_pos + 1) % ring_size;

  float mix_high = pos - floor(pos);
  float mix_low  = 1.0 - mix_high;

  list.colors[low_pos]  = color_fraction(color, mix_low);
  list.colors[next_pos] = color_fraction(color, mix_high);
}

int main(int argc, char** argv) {
  if (argc != 2)
    return 0;

  clean_colors.count = 16;

  struct pwm_bwm2708_dev* pwm = pwm_bwm2708_create(argv[1]);
  struct ws28128_bcm2708 out = { .buffer = NULL };
  struct ws2812_array colors = clean_colors;

  chrono::milliseconds rest(ms_per_frame);

  for (int i = 0; i <= frames_per_min; i++) {
    colors = clean_colors;
    insert_blended_at(colors, {125,0,0}, float(i) * float(ring_size) / float(frames_per_min) );
    output_list(&colors, pwm, &out);

    this_thread::sleep_for(rest);
  }

  std::free(out.buffer);
  pwm_bwm2708_free(pwm);

  return 0;
}
