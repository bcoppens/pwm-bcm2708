#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/platform.h>

#define DRV_NAME	"bcm2708_pwm"

/* TODO */
#define PWM_BASE (BCM2708_PERI_BASE + 0x20c000)

/* platform device */
/* TODO: this should be moved into linux/arch/arm/mach-bcm2708/bcm2708.c */
/* TODO: device tree? */
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
out:
  return ret;
}

static __exit void pwm_bcm2708_exit(void)
{
  printk(KERN_ALERT "Goodbye\n");
  platform_driver_unregister(&bcm2708_pwm_driver);
  platform_device_unregister(bcm2708_pwm_device);
}

module_init(pwm_bcm2708_init);
module_exit(pwm_bcm2708_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart Coppens <kde@bartcoppens.be>");
MODULE_DESCRIPTION("Broadcom BCM2708 PWM driver");

