#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <mach/gpio.h>

#include <asm-generic/gpio.h>


#define DRV_NAME	"bcm2708_pwm"

/* TODO */
#define PWM_BASE (BCM2708_PERI_BASE + 0x20c000)

#define GPIO_PIN 18
#define ALT_FUN GPIO_FSEL_ALT5
#define PWM_CHANNEL 0

struct bcm2708_pwm {
  void __iomem *base;
  int channel;
  
  int gpio;
};

/* TODO */
struct bcm2708_pwm* global;

/* PWM Control register flags.
 * pp 142-143 of the BCM2835 peripheral docs */

/* CTL is at offset 0x0 */
#define CTL_MODE1_SERIALIZE (1<<1) /* Channel 1 mode */
#define CTL_CLRFIFO         (1<<6) /* Clear FIFO */
#define CTL_USEFIFO1        (1<<5) /* Channel 1 use fifo */
#define CTL_PWENABLE1       (1<<0) /* Channel 1 enabled */

/* Test data: Green: 255, Red: 0, Blue: 0 */
/* 1 = 110 ; 0 = 100 ; default value (reset code): 0 */
/* => 110 110 110 110 110 110 110 110   100 100 100 100 100 100 100 100   100 100 100 100 100 100 100 100 */
/* = 11011011011011011011011010010010 01001001001001001001001001001001 00100100000000000000000000000000 */
/* = 0xDB6DB692 0x49249249 0x24000000 */

/* TODO: check status! */

static void __iomem* pwm_data_address(struct bcm2708_pwm* pwm, int channel) {
  if (channel == 0) {
    return pwm->base + 0x14;
  }
  return pwm->base + 24;
}

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

static int setup_device(struct bcm2708_pwm* pwm) {
  int err = 0;
  int gpio = GPIO_PIN;
  struct gpio_chip *gc;
  
  err = gpio_request(gpio, "pwm");
  if (err) {
    printk(KERN_ALERT "Failed to acquire gpio pin");
    goto out;
  }
  
  
  gc = gpio_to_chip(gpio);
  err = bcm2708_set_gpio_function(gc, gpio, ALT_FUN);

  pwm->gpio = gpio;
  pwm->base = __io_address(PWM_BASE); /* TODO: use the device struct */
  pwm->channel = PWM_CHANNEL;
  
  /* Clear fifo */
  set_pwm_ctl(pwm, CTL_CLRFIFO);
  
  /* TODO: no DMA yet */
  set_pwm_ctl(pwm,   CTL_MODE1_SERIALIZE
                   | CTL_USEFIFO1
                   | CTL_PWENABLE1
  );
  /* TODO: verify status? */

  write_data(pwm, 0xDB6DB692);
  write_data(pwm, 0x49249249);
  write_data(pwm, 0x24000000);

out:
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

