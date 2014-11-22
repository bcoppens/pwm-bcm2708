#include <linux/delay.h>
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
struct bcm2708_pwm global_;
struct bcm2708_pwm* global = NULL;

/* PWM Clocks.
 * https://github.com/hermanhermitage/videocoreiv/wiki/Register-Documentation */
#define CM_PWM_CTL        (BCM2708_PERI_BASE + 0x1010a0)
#define CM_PWM_DIV_OFFSET 0x4

#define CM_PASSWD         (0x5a << 24)

#define CM_CTL_BUSY       (1 << 7)
#define CM_CTL_ENAB       (1 << 4)
#define CM_CTL_CLOCK_OSC  0x1

/* The WS2812 data transfer time is ~1.25µs +/- 600ns (one bit transfer), i.e., 800kHz 
 * One WS2812 bit equals 3 PWM-sent bits, thus, we need to clock at 2.4MHz */
#define CLOCK_FREQ (3*800000)

/* Source: https://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-clock-sources-for-the-general-purpose-clocks */
#define OSCILLATOR_FREQUENCY 19200000

/* TODO: fail when trying to set the clock when (readl(pwm->clock.ctl) & CM_CTL_BUSY) ! */
static void pwm_clock_set_div(struct bcm2708_pwm* pwm, uint32_t target_frequency) {
  /* Use integer divider on the 19.2MHz oscillator, no MASH */
  uint32_t divider = OSCILLATOR_FREQUENCY / target_frequency;
  uint32_t div     = CM_PASSWD | (divider << 12);

  writel(div, pwm->clock.div);
  
  printk(KERN_ALERT "Set the divider to %x...\n", divider);
}

static void pwm_clock_enable(struct bcm2708_pwm* pwm) {
  writel(CM_PASSWD | CM_CTL_CLOCK_OSC, pwm->clock.ctl);
  printk(KERN_ALERT "Set the oscillator...\n");

  writel(CM_PASSWD | CM_CTL_CLOCK_OSC | CM_CTL_ENAB, pwm->clock.ctl);
  printk(KERN_ALERT "Enabled clock...\n");
}

static void pwm_clock_disable(struct bcm2708_pwm* pwm) {
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

/* To send '1', send 110; to send '0', send '100' */
#define OUTPUT_BIT(bit) \
  current_data |= (bit) << bit_shift; \
  bit_shift--; \
  if (bit_shift < 0) { /* flush word */ \
    write_data(pwm, current_data); \
    bit_shift = 31; \
    current_data = 0; \
  }

#define OUTPUT_COLOR_LOOP(value) \
    for (i = 7; i >= 0; i--) { \
      OUTPUT_BIT(1); \
      OUTPUT_BIT( ((value) >> i) & 1 ); \
      OUTPUT_BIT(0); \
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

static int setup_device(struct platform_device *pdev, struct bcm2708_pwm* pwm) {
  int err = 0;
  int gpio = GPIO_PIN;
  struct gpio_chip *gc;
  struct ws2812_color color;
  
  struct resource *regs_pwm;
  struct resource *regs_cm_pwm;

  regs_pwm = platform_get_resource_byname(pdev, IORESOURCE_MEM, "io-memory-pwm");
  if (!regs_pwm) {
    printk(KERN_ALERT "Failed to acquire IO resources for PWM\n");
    err = -ENOMEM;
    goto out;
  }

  regs_cm_pwm = platform_get_resource_byname(pdev, IORESOURCE_MEM, "io-memory-cm-pwm");
  if (!regs_cm_pwm) {
    printk(KERN_ALERT "Failed to acquire IO resources for CM_PWM\n");
    err = -ENOMEM;
    goto out;
  }

  err = gpio_request(gpio, "pwm");
  if (err) {
    printk(KERN_ALERT "Failed to acquire gpio pin\n");
    goto out;
  }
  
  
  gc = gpio_to_chip(gpio);
  err = gpio_direction_output(gpio, 0);
  if (err)
    goto out_free_gpio;
  
  err = bcm2708_set_function(gc, gpio, ALT_FUN);
  if (err)
    goto out_free_gpio;
  
  pwm->gpio = gpio;
  pwm->channel = PWM_CHANNEL;
  
  pwm->base = ioremap(regs_pwm->start, resource_size(regs_pwm));
  if (!pwm->base) {
    printk(KERN_ALERT "Failed to remap IO resources for PWM\n"); 
    err = -ENOMEM;
    goto out_free_gpio;
  }
  
  pwm->clock.ctl = ioremap(regs_cm_pwm->start, resource_size(regs_cm_pwm));
  if (!pwm->base) {
    printk(KERN_ALERT "Failed to remap IO resources for CM PWM\n");
    err = -ENOMEM;
    goto out_free_iomem_pwm;
  }
  
  pwm->clock.div = pwm->clock.ctl + CM_PWM_DIV_OFFSET;
  
  printk(KERN_ALERT "pwm->base = %p (PWM_BASE %p)\n", pwm->base, (void*)PWM_BASE);
  printk(KERN_ALERT "pwmclock = %p (PWM_BASE %p)\n", pwm->clock.ctl, (void*)CM_PWM_CTL);
  
  printk(KERN_ALERT "Configuring clock...\n");
  pwm_clock_set_div(pwm, CLOCK_FREQ);
  pwm_clock_enable(pwm);
  printk(KERN_ALERT "Configured clock!\n");
  
  /* Clear PWM STA bus error bit */
  writel(0x100, pwm->base + 0x4);

  /* Clear fifo */
  set_pwm_ctl(pwm, CTL_CLRFIFO);
  writel(32, pwm->base + 0x10);

  /* TODO: no DMA yet */
  set_pwm_ctl(pwm,   CTL_MODE1_SERIALIZE | CTL_USEFIFO1 | CTL_PWENABLE1);
  /* TODO: verify status? */

  /* Light purple, rather than a bright primary color: less chance of
   * it showing up on the display by accident rather than on purpose */
  color.red = 125; color.green =  0; color.blue = 125;
  printk(KERN_ALERT "Outputting single color\n");

  output_single_color(pwm, color);
  udelay(60);

  printk(KERN_ALERT "Disabling clock again\n");
  pwm_clock_disable(pwm);
  printk(KERN_ALERT "Disabled clock\n");

out:
  return err;

out_free_iomem_cm_pwm:
  iounmap(pwm->clock.ctl);

out_free_iomem_pwm:
  iounmap(pwm->base);

out_free_gpio:
  gpio_free(gpio);

  return err;
}

static int release_device(struct bcm2708_pwm* pwm) {
  printk(KERN_ALERT "Releasing device...\n");

  set_pwm_ctl(pwm, 0);

  iounmap(pwm->base);
  gpio_free(pwm->gpio);
  
  printk(KERN_ALERT "Released device...\n");
  
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
    .name	= "io-memory-pwm"
  },
  {
    .start	= CM_PWM_CTL,
    .end	= CM_PWM_CTL + 0x8,
    .flags	= IORESOURCE_MEM,
    .name	= "io-memory-cm-pwm"
  }
};

static struct platform_device *bcm2708_pwm_device;

/* platform driver */

static int bcm2708_pwm_probe(struct platform_device *pdev)
{
  int ret = 0;

  printk(KERN_ALERT "pwm_probe\n");
  
  global = &global_;

  ret = setup_device(pdev, global);
  if (ret) {
    printk(KERN_ALERT "setup device failed!\n");
    global = NULL;
  }
  
  return ret;
}

static int bcm2708_pwm_remove(struct platform_device *pdev)
{
  printk(KERN_ALERT "pwm_remove\n");
  
  if (global)
    release_device(global);
  
  printk(KERN_ALERT "done\n");
  
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
  
out:
  return ret;
}

static __exit void pwm_bcm2708_exit(void)
{
  printk(KERN_ALERT "Goodbye\n");
  platform_driver_unregister(&bcm2708_pwm_driver);
  printk(KERN_ALERT "Driver unregistered\n");
  platform_device_unregister(bcm2708_pwm_device);
  printk(KERN_ALERT "Device unregistered\n");
}

module_init(pwm_bcm2708_init);
module_exit(pwm_bcm2708_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart Coppens <kde@bartcoppens.be>");
MODULE_DESCRIPTION("Broadcom BCM2708 PWM driver");

