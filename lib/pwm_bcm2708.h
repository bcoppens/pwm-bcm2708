#ifndef PWM_BCM2708_H
#define PWM_BCM2708_H

struct pwm_bwm2708_dev;

#include <stddef.h> /* size_t */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: meh */
struct pwm_bwm2708_dev* pwm_bwm2708_create(const char* device_path);
void pwm_bwm2708_free(struct pwm_bwm2708_dev* dev);

int pwm_bwm2708_write_data(struct pwm_bwm2708_dev* dev, void* data, size_t len);

int pwm_bwm2708_get_frequency(struct pwm_bwm2708_dev* dev);
int pwm_bwm2708_set_frequency(struct pwm_bwm2708_dev* dev, uint32_t freq);

#ifdef __cplusplus
}
#endif


#endif /* PWM_BCM2708_H */
