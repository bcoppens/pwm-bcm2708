#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/bcm2708.h>
#include <linux/mutex.h>
#include <linux/stat.h>
#include <linux/string.h>

#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/platform.h>

#include <asm-generic/gpio.h>

#define DRV_NAME	"bcm2708_pwm"

/* Can be found in register documentation wiki */
#define PWM_BASE (BCM2708_PERI_BASE + 0x20c000)

/* TODO: automatically convert this from PWM_BASE */
#define PWM_BASE_PERIPHERAL_SPACE 0x7e20c000

#define GPIO_PIN 18
#define ALT_FUN GPIO_FSEL_ALT5
#define PWM_CHANNEL 0

/* TODO: check whether or not we can/should use clk_get(&pdev->dev, clock_name) */
struct clock {
	void __iomem *ctl;
	void __iomem *div;
};

/* Serialize single color, as GREEN RED BLUE */
struct ws2812_color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

/* For now, we always allocate a DMA of PAGE_SIZE. TODO: improve this,
 * possibly using scatter/gather (_sg) calls */
/* TODO: look at if Streaming DMA mappings are useful here,
 * rather than coherent DMA mappings ...
 *
 * For now, we put the DMA CB in the beginning of the DMA region...
 */
struct dma_info {
	int can_dma;

	size_t start_offset;	/* sizeof(struct bcm2708_dma_cb) */

	size_t allocated_size;
	size_t usable_size;
	size_t used_size;

	void *buf;
	dma_addr_t handle;	/* IO address of buf */

	int chan;
	int irq;		// TODO don't we need to explicitly acquire it? But not used atm
	void __iomem *base;

	struct bcm2708_dma_cb *cb;
};

struct bcm2708_pwm {
	struct mutex mutex;

	struct device* dev;

	void __iomem *base;
	int channel;

	int gpio;

	struct clock clock;	/* PWM-clock, so it's OK we manage this ourselves */

	/* To test */
	struct ws2812_color color;
	struct dma_info dma;
};

/* TODO */
struct bcm2708_pwm global_;
struct bcm2708_pwm *global = NULL;

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
static void pwm_clock_set_div(struct bcm2708_pwm *pwm,
			      uint32_t target_frequency)
{
	/* Use integer divider on the 19.2MHz oscillator, no MASH */
	uint32_t divider = OSCILLATOR_FREQUENCY / target_frequency;
	uint32_t div = CM_PASSWD | (divider << 12);

	writel(div, pwm->clock.div);

	dev_info(pwm->dev, "Set the divider to %x...\n", divider);
}

static void pwm_clock_enable(struct bcm2708_pwm *pwm)
{
	writel(CM_PASSWD | CM_CTL_CLOCK_OSC, pwm->clock.ctl);
	dev_info(pwm->dev, "Set the oscillator...\n");

	writel(CM_PASSWD | CM_CTL_CLOCK_OSC | CM_CTL_ENAB, pwm->clock.ctl);
	dev_info(pwm->dev, "Enabled clock...\n");
}

static void pwm_clock_disable(struct bcm2708_pwm *pwm)
{
	writel(CM_PASSWD, pwm->clock.ctl);
}

/* PWM Control register flags.
 * pp 142-143 of the BCM2835 peripheral docs */

/* CTL is at offset 0x0 */
#define CTL_MODE1_SERIALIZE (1<<1)	/* Channel 1 mode */
#define CTL_CLRFIFO         (1<<6)	/* Clear FIFO */
#define CTL_USEFIFO1        (1<<5)	/* Channel 1 use fifo */
#define CTL_PWENABLE1       (1<<0)	/* Channel 1 enabled */

/* TODO: check status! */

static void __iomem *pwm_fifo(struct bcm2708_pwm *pwm)
{
	return pwm->base + 0x18;
}

static void __iomem *pwm_dmac(struct bcm2708_pwm *pwm)
{
	return pwm->base + 0x8;
}

static void set_pwm_ctl(struct bcm2708_pwm *pwm, int flags)
{
	/* We need to write the reserved flags as 0, so this is ok */
	writel(flags, pwm->base);
}

/* For now to test: no dma */
static void write_data(struct bcm2708_pwm *pwm, uint32_t data)
{
	writel(data, pwm_fifo(pwm));
}

#define PWM_DMAC_ENAB     (1<<31)
#define PWM_DMAC_PANIC(n) (n<<8)
#define PWM_DMAC_DREQ(n)  (n)
static void pwm_enable_dma(struct bcm2708_pwm *pwm)
{
	writel(PWM_DMAC_ENAB | PWM_DMAC_PANIC(7) | PWM_DMAC_DREQ(3),
	       pwm_dmac(pwm));
}

static void pwm_disable_dma(struct bcm2708_pwm *pwm)
{
	writel(0, pwm_dmac(pwm));
}

/* Test code */

/* To send '1', send 110; to send '0', send '100' */
#define OUTPUT_BIT(bit, write_command) \
  current_data |= (bit) << bit_shift; \
  bit_shift--; \
  if (bit_shift < 0) { /* flush word */ \
    write_command ; \
    bit_shift = 31; \
    current_data = 0; \
  }

#define OUTPUT_COLOR_LOOP(value, write_command) \
    for (i = 7; i >= 0; i--) { \
      OUTPUT_BIT(1, write_command); \
      OUTPUT_BIT( ((value) >> i) & 1, write_command ); \
      OUTPUT_BIT(0, write_command); \
    }

static void output_single_color(struct bcm2708_pwm *pwm,
				struct ws2812_color color)
{
	int bit_shift = 31;
	uint32_t current_data = 0;
	int i;

	OUTPUT_COLOR_LOOP(color.green, write_data(pwm, current_data))
	    OUTPUT_COLOR_LOOP(color.red, write_data(pwm, current_data))
	    OUTPUT_COLOR_LOOP(color.blue, write_data(pwm, current_data))

	    /* Flush remaining */
	    write_data(pwm, current_data);

	/* Silence bit is 0 -> reset command for the WS2812 */
	write_data(pwm, 0);
	udelay(100);
}

static size_t write_ws2812_list_to_buffer(struct ws2812_color *list,
					  size_t nens,
					  uint32_t * buffer, size_t buffer_size)
{
	int bit_shift = 31;
	uint32_t current_data = 0;
	int i;
	size_t buf_pos = 0;
	size_t list_pos = 0;

#define TO_BUFFER \
  do { \
    buffer[buf_pos++] = current_data; \
    if(buf_pos >= buffer_size) goto done; \
  } while(0)

	for (list_pos = 0; list_pos < nens; list_pos++) {
		OUTPUT_COLOR_LOOP(list[list_pos].green, TO_BUFFER)
		    OUTPUT_COLOR_LOOP(list[list_pos].red, TO_BUFFER)
		    OUTPUT_COLOR_LOOP(list[list_pos].blue, TO_BUFFER)
	}

	/* Flush remaining */
	TO_BUFFER;

	/* Silence */
	current_data = 0;
	TO_BUFFER;

done:
	if (buf_pos >= buffer_size)
		printk(KERN_ALERT "Buffer was full!\n");

	return buf_pos;		/* Number of elements to be transfered with DMA */
}

#undef TO_BUFFER

#undef OUTPUT_COLOR_LOOP
#undef OUTPUT_BIT

static struct ws2812_color test_color_list_[] = {
	{125,	0,	0},
	{0,	125,	0},
	{0,	0,	125},
	{125,	125,	0},
	{0,	125,	125},
	{125,	125,	125}
};

/* To quickly clear the LEDs */
static struct ws2812_color test_color_list[] = {
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0}
};

/* sysfs code */
static ssize_t pwm_show_led0_color(struct device_driver *driver, char *buf)
{
	/* TODO: container_of to get the pdev, it's associated data, and through it, the pwm */
	struct ws2812_color color;

	if (!global) {
		return snprintf(buf, PAGE_SIZE, "%s", "ERROR\n");
	}

	mutex_lock(&global->mutex);	/* TODO: interruptible? */
	color = global->color;
	mutex_unlock(&global->mutex);

	return snprintf(buf, PAGE_SIZE, "%i %i %i\n", color.red, color.green,
			color.blue);
}

static void output_color(struct bcm2708_pwm *pwm);
static void dma_output_color_list(struct bcm2708_pwm *pwm,
				  struct ws2812_color *list, size_t nens);

/* TODO: check that pos < count! */
static ssize_t pwm_store_led0_color(struct device_driver *driver,
				    const char *buf, size_t count)
{
	struct ws2812_color color;

	if (!global) {
		printk(KERN_WARNING "bcm2708_pwm: "
		       "Trying to set led color, but no PWM device!\n");
		return -ENOMEM;
	}

	if (!buf) {
		printk(KERN_WARNING "bcm2708_pwm: NULL buffer in store!");
		return -ENOMEM;
	}

	if (sscanf(buf, "%hhu %hhu %hhu",
		   &color.red, &color.green, &color.blue) != 3) {
		printk(KERN_WARNING "bcm2708_pwm: "
		       "Failed to parse '%s' as an RGB triplet\n",
		       buf);
		count = -ENOMEM;	/* TODO: find a better error code */
		goto out;
	}

	printk(KERN_INFO "bcm2708_pwm: Setting LED color to %i %i %i\n",
	       color.red, color.green, color.blue);

	mutex_lock(&global->mutex);
	global->color = color;
	output_color(global);

out:
	mutex_unlock(&global->mutex);
	return count;
}

/* TODO: perhaps dynamically alloc memory? */
/* 16 elements matching my NeoPixel ring for now */
/* TODO a PAGE_SIZE of RGB triplets ought to suffice for most purposes, but otherwise
 * I will need to make a character device... find this out */
/* Format: R,G,B[ R,G,B]* */
static struct ws2812_color sysfs_color_list[16];
static ssize_t pwm_store_led0_color_string(struct device_driver *driver,
					   const char *buf, size_t count)
{
	struct ws2812_color color;
	int elements_read = 0;

	if (!global) {
		printk(KERN_WARNING
		       "Trying to set led color, but no PWM device!\n");
		return -ENOMEM;
	}

	if (!buf) {
		printk(KERN_WARNING "NULL buffer in store!");
		return -ENOMEM;
	}

	while (elements_read < ARRAY_SIZE(sysfs_color_list)) {
		if (sscanf
		    (buf, "%hhu,%hhu,%hhu", &color.red, &color.green,
		     &color.blue) != 3)
			break;

		sysfs_color_list[elements_read++] = color;
		// printk(KERN_ALERT "Read LED color as %i %i %i\n", color.red, color.green, color.blue);

		buf = strchr(buf, ' ');
		if (!buf)
			break;
		buf++;		/* skip the space */
	}

	printk(KERN_INFO "bcm2708_pwm: Setting %i new colors\n", elements_read);

	mutex_lock(&global->mutex);
	dma_output_color_list(global, sysfs_color_list, elements_read);

out:
	mutex_unlock(&global->mutex);
	return count;
}

static DRIVER_ATTR(pwm_led0_color, S_IRUGO | S_IWUSR, pwm_show_led0_color,
		   pwm_store_led0_color);
static DRIVER_ATTR(pwm_led0_color_string, S_IWUSR, NULL,
		   pwm_store_led0_color_string);

static struct attribute *pwm_dev_attrs[] = {
	&driver_attr_pwm_led0_color.attr,
	&driver_attr_pwm_led0_color_string.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pwm_dev);

/* Does some generic preparation: enable clock, clear FIFO.
 * Optionally it enables the PWM's DMA configuration.
 * Then it enables PWM.
 */
static void pwm_prepare_and_start(struct bcm2708_pwm *pwm, int start_dma)
{
	pwm_clock_enable(pwm);
	dev_info(pwm->dev, "Configured clock!\n");

	/* Clear PWM STA bus error bit */
	writel(0x100, pwm->base + 0x4);

	/* Clear fifo */
	set_pwm_ctl(pwm, CTL_CLRFIFO);
	writel(32, pwm->base + 0x10);

	if (start_dma) {
		pwm_enable_dma(pwm);
	}

	set_pwm_ctl(pwm, CTL_MODE1_SERIALIZE | CTL_USEFIFO1 | CTL_PWENABLE1);
	/* TODO: verify status? */
}

static void output_color(struct bcm2708_pwm *pwm)
{
	/* TODO: we should automatically disable it after DMA is done, use IRQ? */
	pwm_disable_dma(pwm);

	pwm_prepare_and_start(pwm, 0 /* no DMA */ );

	dev_dbg(pwm->dev, "Outputting single color %hhu %hhu %hhu\n",
	       pwm->color.red, pwm->color.green, pwm->color.blue);

	output_single_color(pwm, pwm->color);

	set_pwm_ctl(pwm, 0);

	dev_info(pwm->dev, "Disabling clock again\n");
	pwm_clock_disable(pwm);
	dev_info(pwm->dev, "Disabled clock\n");
}

/* DMA-related code */
static int initialize_dma(struct bcm2708_pwm *pwm)
{
	/* We require DMA Channel 5. Try to get it using a HACK
	 * (proper way: add a bcm2708 function to request a specific channel) */
	int ret;
	int acquired_chans[5];
	int acquired_chans_count = 0;
	int i;
	struct device *dev = pwm->dev;

	struct dma_info *dma = &pwm->dma;
	dma->can_dma = 0;

	/* Channel 5 has BCM_DMA_FEATURE_NORMAL_ORD set... */
	do {
		ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_NORMAL_ORD,
					 &(dma->base), &(dma->irq));

		if (ret < 0) {
			dev_warn(dev, "Failed to acquire DMA channel!\n");
			goto release_channels;
		} else if (ret == 5) {
			/* Correct channel found! */
			dma->chan = 5;
			break;
		}

		acquired_chans[acquired_chans_count++] = ret;

		if (acquired_chans_count >= 5) {
			dev_warn(dev,
			       "Looped over more than 5 DMA channels!\n");
			goto release_channels;
		}

	} while (1);

	dma->buf =
	    dma_alloc_coherent(dev, PAGE_SIZE, &(dma->handle), 0 /*flags */ );
	if (!dma->buf) {
		dev_warn(dev, "Failed to allocate coherent DMA buffer!\n");
		goto release_channels;
	}

	dma->can_dma = 1;
	dma->allocated_size = PAGE_SIZE;
	dma->start_offset = sizeof(struct bcm2708_dma_cb);
	dma->usable_size = PAGE_SIZE - dma->start_offset;
	dma->used_size = 0;

	dma->cb = dma->buf;

	ret = 0;

release_channels:
	if (!dma->can_dma && dma->chan > 0)
		bcm_dma_chan_free(dma->chan);

	for (i = 0; i < acquired_chans_count; i++)
		bcm_dma_chan_free(acquired_chans[i]);	/* TODO: check errors */

	return ret;
}

static void release_dma(struct platform_device *pdev, struct bcm2708_pwm *pwm)
{
	struct dma_info *dma = &pwm->dma;

	if (!dma->can_dma)
		return;

	bcm_dma_abort(dma->base);	/* TODO return value */

	bcm_dma_chan_free(dma->chan);
	dma_free_coherent(&pdev->dev, dma->allocated_size, dma->buf,
			  dma->handle);
}

/* This assumes the data is already in the pwm->dma buffer */
static void pwm_dma_output(struct bcm2708_pwm *pwm, size_t len /* in bytes */) {
	struct dma_info *dma = &pwm->dma;
	unsigned long cb_io;
	unsigned long buf_io;
	struct bcm2708_dma_cb *cb;

	if (!dma->can_dma) {
		dev_warn(pwm->dev, "Cannot DMA! Aborting list output\n");
		return;
	}

	buf_io = dma->handle + dma->start_offset;
	cb_io = dma->handle;	/* Because the CB is currently at the page start */

	cb = dma->cb;
	/* write 32 bit words, with no destination address increase, no bursts */
	cb->info = BCM2708_DMA_WAIT_RESP
	    | BCM2708_DMA_D_DREQ | BCM2708_DMA_S_INC | BCM2708_DMA_PER_MAP(5);
	cb->src = buf_io;
	cb->dst = PWM_BASE_PERIPHERAL_SPACE + 0x18;	/* TODO factor out 0x18 */
	/* transfer len 32 bit words, DMA engine uses bytes */
	cb->length = len;
	cb->stride = 0;
	cb->next = 0;		/* for now no cb chaining */
	cb->pad[0] = 0;
	cb->pad[1] = 0;

	dev_dbg(pwm->dev, "Starting dma transfer of length %li\n", cb->length);

	/* For now, since we use coherent memory, no need to flush CB */

	/* Abort if busy, TODO check return value */
	bcm_dma_abort(dma->base);

	pwm_prepare_and_start(pwm, 1 /* DMA */ );

	bcm_dma_start(dma->base, cb_io);

	/* TODO: I want to set CTL_PWENABLE1 to 0 again when we are done, and disable the clock! */
}

static void dma_output_color_list(struct bcm2708_pwm *pwm,
				  struct ws2812_color *list, size_t nens)
{
	struct dma_info *dma = &pwm->dma;
	size_t buf_size;
	uint32_t *buf_virt;
	size_t len;

	if (!dma->can_dma) {
		dev_warn(pwm->dev, "Cannot DMA! Aborting list output\n");
		return;
	}

	/* We write uint32_t's, but the actual size for DMA is in bytes */
	buf_size = dma->usable_size / sizeof(uint32_t);
	buf_virt = (uint32_t *) (dma->buf + dma->start_offset);

	len = write_ws2812_list_to_buffer(list, nens, buf_virt, buf_size);

	pwm_dma_output(pwm, len * sizeof(uint32_t));
}

static int setup_device(struct platform_device *pdev, struct bcm2708_pwm *pwm)
{
	int err = 0;
	int gpio = GPIO_PIN;
	struct gpio_chip *gc;
	struct ws2812_color color;

	struct resource *regs_pwm;
	struct resource *regs_cm_pwm;

	mutex_init(&pwm->mutex);

	/* Light purple, rather than a bright primary color: less chance of
	 * it showing up on the display by accident rather than on purpose */
	color.red = 125;
	color.green = 0;
	color.blue = 125;
	pwm->color = color;
	
	pwm->dev = &pdev->dev;

	regs_pwm =
	    platform_get_resource_byname(pdev, IORESOURCE_MEM, "io-memory-pwm");
	if (!regs_pwm) {
		dev_warn(pwm->dev, "Failed to acquire IO resources for PWM\n");
		err = -ENOMEM;
		goto out;
	}

	regs_cm_pwm =
	    platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "io-memory-cm-pwm");
	if (!regs_cm_pwm) {
		dev_warn(pwm->dev,
		       "Failed to acquire IO resources for CM_PWM\n");
		err = -ENOMEM;
		goto out;
	}

	err = gpio_request(gpio, "pwm");
	if (err) {
		dev_warn(pwm->dev, "Failed to acquire gpio pin\n");
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
		dev_warn(pwm->dev, "Failed to remap IO resources for PWM\n");
		err = -ENOMEM;
		goto out_free_gpio;
	}

	pwm->clock.ctl =
	    ioremap(regs_cm_pwm->start, resource_size(regs_cm_pwm));
	if (!pwm->base) {
		dev_warn(pwm->dev, "Failed to remap IO resources for CM PWM\n");
		err = -ENOMEM;
		goto out_free_iomem_pwm;
	}

	pwm->clock.div = pwm->clock.ctl + CM_PWM_DIV_OFFSET;

	dev_info(pwm->dev, "pwm->base = %p (PWM_BASE %p)\n", pwm->base,
	       (void *)PWM_BASE);
	dev_info(pwm->dev, "pwmclock = %p (PWM_BASE %p)\n", pwm->clock.ctl,
	       (void *)CM_PWM_CTL);

	dev_info(pwm->dev, "Configuring clock...\n");
	pwm_clock_set_div(pwm, CLOCK_FREQ);

	/* TODO: at least when streaming DMA channels, this should not be (all) done here! */
	dev_info(pwm->dev, "Initializing DMA...\n");
	initialize_dma(pwm);
	dev_info(pwm->dev, "Initialized DMA\n");

	/* Output initial color */
	//output_color(pwm);
	dma_output_color_list(pwm, test_color_list,
			      ARRAY_SIZE(test_color_list));

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

static int release_device(struct platform_device *pdev, struct bcm2708_pwm *pwm)
{
	dev_info(pwm->dev, "Releasing DMA...\n");
	release_dma(pdev, pwm);

	dev_info(pwm->dev, "Releasing device...\n");
	set_pwm_ctl(pwm, 0);

	iounmap(pwm->base);
	gpio_free(pwm->gpio);

	dev_info(pwm->dev, "Released device...\n");

	return 0;
}

/* Character device */
struct bcm2708_pwm_cdev {
	struct cdev cdev;
	struct mutex mutex;

	dev_t dev;

	struct bcm2708_pwm *pwm;
};

/* TODO: assume, for now, we only have one */
static struct bcm2708_pwm_cdev pwm_cdev_;
static struct bcm2708_pwm_cdev* pwm_cdev = NULL;

int pwm_cdev_open(struct inode *inode, struct file *filp)
{
	dev_info(pwm_cdev->pwm->dev, "cdev opened\n");
	return 0;
}

int pwm_cdev_release(struct inode *inode, struct file *filp)
{
	dev_info(pwm_cdev->pwm->dev, "cdev closed\n");
	return 0;
}

static struct file_operations pwm_cdev_fops = {
	.owner	= THIS_MODULE,
	.open	= pwm_cdev_open,
	.release= pwm_cdev_release
};

static int init_pwm_cdev(struct bcm2708_pwm *pwm)
{
	int err;
	dev_t dev;

	err = alloc_chrdev_region(&dev, 0, 1, "bcm2708_pwm");
	if (err) {
		dev_warn(pwm->dev, "Could not allocate chrdev region\n");
		return err;
	}

	cdev_init(&pwm_cdev_.cdev, &pwm_cdev_fops);
	pwm_cdev_.cdev.owner = THIS_MODULE;

	err = cdev_add(&pwm_cdev_.cdev, dev, 1);
	if (err) {
		dev_warn(pwm->dev, "Could not add cdev\n");
		goto out;
	}

	mutex_init(&pwm_cdev_.mutex);
	pwm_cdev_.pwm = pwm;
	pwm_cdev_.dev = dev;

	pwm_cdev = &pwm_cdev_;
	return 0;

out:
	unregister_chrdev_region(dev, 1);
	return err;
}

static void release_pwm_cdev(struct bcm2708_pwm_cdev* pwm_cdev)
{
	cdev_del(&pwm_cdev->cdev);
	unregister_chrdev_region(pwm_cdev->dev, 1);
}

/* platform device */
/* TODO: this should be moved into linux/arch/arm/mach-bcm2708/bcm2708.c */
/* TODO: device tree? */
/* TODO: look at whether or not the current gpio code uses pinmux/pinctrl, and if not, if I should modify it */

static struct resource bcm2708_pwm_resources[] = {
	{
	 .start = PWM_BASE,
	 .end = PWM_BASE + 0x32,
	 .flags = IORESOURCE_MEM,
	 .name = "io-memory-pwm"
	},
	{
	 .start = CM_PWM_CTL,
	 .end = CM_PWM_CTL + 0x8,
	 .flags = IORESOURCE_MEM,
	 .name = "io-memory-cm-pwm"
	}
};

static struct platform_device *bcm2708_pwm_device;

/* platform driver */

static int bcm2708_pwm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bcm2708_pwm *pwm = &global_;

	ret = setup_device(pdev, pwm);
	if (ret)
		goto err;

	ret = init_pwm_cdev(pwm);
	if (ret) {
		release_device(pdev, pwm);
		goto err;
	}

	global = pwm;

err:
	return ret;
}

static int bcm2708_pwm_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "pwm_remove\n");

	if (global) {
		release_pwm_cdev(pwm_cdev);
		release_device(pdev, global);
	}

	dev_info(&pdev->dev, "done\n");

	return 0;
}

static const struct of_device_id bcm2708_pwm_match[] = {
	{.compatible = "brcm,bcm2708_pwm",},
	{}
};

MODULE_DEVICE_TABLE(of, bcm2708_pwm_match);

static struct platform_driver bcm2708_pwm_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .groups = pwm_dev_groups,
		   },
	.probe = bcm2708_pwm_probe,
	.remove = bcm2708_pwm_remove,
};

static __init int pwm_bcm2708_init(void)
{
	int ret = 0;

	printk(KERN_INFO "pwm_bcm2708: Hello... registring platform driver\n");

	bcm2708_pwm_device = platform_device_alloc("bcm2708_pwm", 0);
	if (!bcm2708_pwm_device) {
		ret = -ENOMEM;
		printk(KERN_ALERT "pwm_bcm2708: Unable to allocate platform device!\n");
		goto out;
	}

	/* From bcm2708.c */
#define DMA_MASK_BITS_COMMON 32

	bcm2708_pwm_device->dev.coherent_dma_mask
	    = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

	ret = platform_device_add_resources(bcm2708_pwm_device,
					    bcm2708_pwm_resources,
					    ARRAY_SIZE(bcm2708_pwm_resources));

	if (ret) {
		printk(KERN_ALERT "pwm_bcm2708: Unable to add platform resources!\n");
		goto out;
	}

	ret = platform_device_add(bcm2708_pwm_device);
	if (ret) {
		printk(KERN_ALERT "pwm_bcm2708: Unable to add platform device!\n");
		goto out;
	}

	ret = platform_driver_register(&bcm2708_pwm_driver);
	if (!ret)
		goto out;	/* TODO: unregister device */

out:
	return ret;
}

static __exit void pwm_bcm2708_exit(void)
{
	/* TODO: don't unregister if not (succesfully) registered */
	printk(KERN_INFO "pwm_bcm2708: Goodbye\n");
	platform_driver_unregister(&bcm2708_pwm_driver);
	printk(KERN_INFO "pwm_bcm2708: Driver unregistered\n");
	platform_device_unregister(bcm2708_pwm_device);
	printk(KERN_INFO "pwm_bcm2708: Device unregistered\n");
}

module_init(pwm_bcm2708_init);
module_exit(pwm_bcm2708_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart Coppens <kde@bartcoppens.be>");
MODULE_DESCRIPTION("Broadcom BCM2708 PWM driver");

// kate: indent-width 8

