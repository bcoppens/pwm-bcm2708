#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/gpio.h>

#include <linux/platform_data/bcm2708.h>

#include <asm-generic/gpio.h>


#define DRV_NAME	"bcm2708_pwm"

/* TODO */
#define PWM_BASE (BCM2708_PERI_BASE + 0x20c000)

#define GPIO_PIN 18
#define ALT_FUN GPIO_FSEL_ALT5
#define PWM_CHANNEL 0

struct clock {
  void __iomem *ctl;
  void __iomem *div;
};

struct bcm2708_pwm {
  void __iomem *base;
  int channel;
  
  int gpio;
  
  struct clock clock; /* PWM-clock, so it's OK we manage this ourselves */
};

/* TODO */
struct bcm2708_pwm* global;

/* PWM Clocks.
 * https://github.com/hermanhermitage/videocoreiv/wiki/Register-Documentation */
#define CM_PWM_CTL       (BCM2708_PERI_BASE + 0x1010a0)
#define CM_PWM_DIV       (BCM2708_PERI_BASE + 0x1010a4)

#define CM_PASSWD        (0x5a << 24)

#define CM_CTL_BUSY      (1 << 7)
#define CM_CTL_ENAB      (1 << 4)
#define CM_CTL_CLOCK_OSC 0x1

/* The WS2812 data transfer time is ~1.25µs +/- 600ns (one bit transfer), i.e., 800kHz 
 * One WS2812 bit equals 3 PWM-sent bits, thus, we need to clock at 2.4MHz */
#define CLOCK_FREQ (3*800000)

/* Source: https://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-clock-sources-for-the-general-purpose-clocks */
#define OSCILLATOR_FREQUENCY 19200000

/* Busy wait, urgh! */
static void wait_for_clock_ready(struct bcm2708_pwm* pwm) {
  while (readl(pwm->clock.ctl) & CM_CTL_BUSY)
    ;
}

static void pwm_clock_set_div(struct bcm2708_pwm* pwm, uint32_t target_frequency) {
  /* Use integer divider on the 19.2MHz oscillator, no MASH */
  uint32_t divider = OSCILLATOR_FREQUENCY / target_frequency;
  uint32_t div     = CM_PASSWD | (divider << 12);
  
  wait_for_clock_ready(pwm);
  writel(div, pwm->clock.div);
  
  /* The clock source has to be set separately from enabling the clock,
   * so we do that here, where it conceptually belongs */
  wait_for_clock_ready(pwm);
  writel(CM_PASSWD | CM_CTL_CLOCK_OSC, pwm->clock.ctl);
}

static void pwm_clock_enable(struct bcm2708_pwm* pwm) {
  wait_for_clock_ready(pwm);
  writel(CM_PASSWD | CM_CTL_ENAB, pwm->clock.ctl);
}

static void pwm_clock_disable(struct bcm2708_pwm* pwm) {
  wait_for_clock_ready(pwm);
  writel(CM_PASSWD, pwm->clock.ctl);
}

/* PWM Control register flags.
 * pp 142-143 of the BCM2835 peripheral docs */

/* CTL is at offset 0x0 */
#define CTL_MODE1_SERIALIZE (1<<1) /* Channel 1 mode */
#define CTL_CLRFIFO         (1<<6) /* Clear FIFO */
#define CTL_USEFIFO1        (1<<5) /* Channel 1 use fifo */
#define CTL_PWENABLE1       (1<<0) /* Channel 1 enabled */

/* TODO: check status! */

static void __iomem* pwm_fifo(struct bcm2708_pwm* pwm) {
  return pwm->base + 0x18;
}

static void set_pwm_ctl(struct bcm2708_pwm* pwm, int flags) {
  /* We need to write the reserved flags as 0, so this is ok */
  writel(flags, pwm->base);
}

/* For now to test: no dma */
static void write_data(struct bcm2708_pwm* pwm, uint32_t data) {
  writel(data, pwm_fifo(pwm));
}

/* Test code */

/* Serialize single color, as GREEN RED BLUE */
struct ws2812_color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

#define OUTPUT_COLOR_LOOP(value) \
    for (i = 7; i >= 0; i--) { \
      current_data |= (( (value) >> i) & 1) << bit_shift; \
      bit_shift--; \
      \
      if (bit_shift < 0) { /* flush word */ \
        write_data(pwm, current_data); \
        bit_shift = 31; \
        current_data = 0; \
      } \
    }

static void output_single_color(struct bcm2708_pwm* pwm, struct ws2812_color color) {
  int bit_shift = 31;
  uint32_t current_data = 0;
  int i;
  
  OUTPUT_COLOR_LOOP(color.green)
  OUTPUT_COLOR_LOOP(color.red)
  OUTPUT_COLOR_LOOP(color.blue)
  
  /* Flush remaining */
  write_data(pwm, current_data);
  
  /* Silence bit is 0 -> reset command for the WS2812 */

}

static int setup_device(struct bcm2708_pwm* pwm) {
  int err = 0;
  int gpio = GPIO_PIN;
  struct gpio_chip *gc;
  struct ws2812_color color;
  
  err = gpio_request(gpio, "pwm");
  if (err) {
    printk(KERN_ALERT "Failed to acquire gpio pin");
    goto out;
  }
  
  
  gc = gpio_to_chip(gpio);
  err = gpio_direction_input(gpio);
  if (err)
    goto out_free;

  err = bcm2708_set_function(gc, gpio, ALT_FUN);

  pwm->gpio = gpio;
  pwm->base = __io_address(PWM_BASE); /* TODO: use the device struct */
  pwm->channel = PWM_CHANNEL;
  
  pwm->clock.ctl = __io_address(CM_PWM_CTL);
  pwm->clock.div = __io_address(CM_PWM_DIV);
  
  
  
  /* Clear fifo */
  set_pwm_ctl(pwm, CTL_CLRFIFO);
  
  /* TODO: no DMA yet */
  set_pwm_ctl(pwm,   CTL_MODE1_SERIALIZE
                   | CTL_USEFIFO1
                   | CTL_PWENABLE1
  );
  /* TODO: verify status? */

  pwm_clock_set_div(pwm, CLOCK_FREQ);
  pwm_clock_enable(pwm);

  /* Light purple, rather than a bright primary color: less chance of
   * it showing up on the display by accident rather than on purpose */
  color.red = 125; color.green =  0; color.blue = 125;
  output_single_color(pwm, color);
  
  pwm_clock_disable(pwm);

out:
  return err;
out_free:
  gpio_free(gpio);
  return err;
}

static int release_device(struct bcm2708_pwm* pwm) {
  set_pwm_ctl(pwm, 0);

  gpio_free(pwm->gpio);
  
  return 0;
}

/* platform device */
/* TODO: this should be moved into linux/arch/arm/mach-bcm2708/bcm2708.c */
/* TODO: device tree? */
/* TODO: look at whether or not the current gpio code uses pinmux/pinctrl, and if not, if I should modify it */

static struct resource bcm2708_pwm_resources[] = {
  {
    .start	= PWM_BASE,
    .end	= PWM_BASE + 0x32,
    .flags	= IORESOURCE_MEM,
    .name	= "io-memory"
  }
};

static struct platform_device *bcm2708_pwm_device;

/* platform driver */

static int bcm2708_pwm_probe(struct platform_device *pdev)
{
  printk(KERN_ALERT "pwm_probe\n");
  return 0;
}

static int bcm2708_pwm_remove(struct platform_device *pdev)
{
  printk(KERN_ALERT "pwm_remove\n");
  return 0;
}

static const struct of_device_id bcm2708_pwm_match[] = {
  { .compatible = "brcm,bcm2708_pwm", },
  {}
};
MODULE_DEVICE_TABLE(of, bcm2708_pwm_match);

static struct platform_driver bcm2708_pwm_driver = {
  .driver  = {
    .name    = DRV_NAME,
    .owner   = THIS_MODULE,
  },
  .probe  = bcm2708_pwm_probe,
  .remove = bcm2708_pwm_remove,
};

static __init int pwm_bcm2708_init(void)
{
  int ret = 0;

  printk(KERN_ALERT "Hello... registring platform driver\n");

  bcm2708_pwm_device = platform_device_alloc("bcm2708_pwm", 0);
  if (!bcm2708_pwm_device) {
    ret = -ENOMEM;
    printk(KERN_ALERT "Unable to allocate platform device!\n");
    goto out;
  }
  
  ret = platform_device_add_resources(
    bcm2708_pwm_device,
    bcm2708_pwm_resources,
    ARRAY_SIZE(bcm2708_pwm_resources));
  
  if (ret) {
    printk(KERN_ALERT "Unable to add platform resources!\n");
    goto out;
  }

  ret = platform_device_add(bcm2708_pwm_device);
  if (ret) {
    printk(KERN_ALERT "Unable to add platform device!\n");
    goto out;
  }

  ret = platform_driver_register(&bcm2708_pwm_driver);
  if (!ret)
    goto out; /* TODO: unregister device */
  
  ret = setup_device(global);
  
out:
  return ret;
}

static __exit void pwm_bcm2708_exit(void)
{
  printk(KERN_ALERT "Goodbye\n");
  platform_driver_unregister(&bcm2708_pwm_driver);
  platform_device_unregister(bcm2708_pwm_device);
  release_device(global);
}

module_init(pwm_bcm2708_init);
module_exit(pwm_bcm2708_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart Coppens <kde@bartcoppens.be>");
MODULE_DESCRIPTION("Broadcom BCM2708 PWM driver");

