#include <linux/init.h>
#include <linux/module.h>

static __init int pwm_bcm2708_init(void)
{
  printk(KERN_ALERT "Hello\n");
  return 0;
}

static __exit void pwm_bcm2708_exit(void)
{
  printk(KERN_ALERT "Goodbye\n");
}

module_init(pwm_bcm2708_init);
module_exit(pwm_bcm2708_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart Coppens <kde@bartcoppens.be>");
MODULE_DESCRIPTION("Broadcom BCM2708 PWM driver");

