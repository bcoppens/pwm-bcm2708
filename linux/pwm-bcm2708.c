#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/platform_data/bcm2708.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/wait.h>

#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/platform.h>

#include <asm/uaccess.h>

#include <asm-generic/gpio.h>

#define DRV_NAME	"bcm2708_pwm"

/* Can be found in register documentation wiki */
#define PWM_BASE (BCM2708_PERI_BASE + 0x20c000)

/* TODO: automatically convert this from PWM_BASE */
#define PWM_BASE_PERIPHERAL_SPACE 0x7e20c000

#define GPIO_PIN 18
#define ALT_FUN GPIO_FSEL_ALT5
#define PWM_CHANNEL 0

/* TODO: these should be in a header shared with userspace? */
#define PWM_BCM2708_IOCTL_MAGIC 0xB4
#define PWM_BCM2708_IOCTL_GET_FREQUENCY \
	_IO(PWM_BCM2708_IOCTL_MAGIC, 0)
#define PWM_BCM2708_IOCTL_SET_FREQUENCY \
	_IOW(PWM_BCM2708_IOCTL_MAGIC, 1, uint32_t)
#define PWM_BCM2708_IOCTL_MAX 1

/* TODO: check whether or not we can/should use clk_get(&pdev->dev, clock_name) */
struct clock {
	void __iomem *ctl;
	void __iomem *div;

	uint32_t current_freq;
	int enabled;
};

/* TODO: look at the semaphore/mutex, and the atomic. It needs a thorough review! */

/* For now, we always allocate a DMA of PAGE_SIZE. TODO: improve this,
 * possibly using scatter/gather (_sg) calls */
/* TODO: look at if Streaming DMA mappings are useful here,
 * rather than coherent DMA mappings ...
 *
 * For now, we put the DMA CB in the beginning of the DMA region...
 */
struct dma_region {
	size_t start_offset;	/* sizeof(struct bcm2708_dma_cb) */

	size_t allocated_size;
	size_t usable_size;
	size_t used_size;

	void *buf;
	dma_addr_t handle;	/* IO address of buf */

	struct bcm2708_dma_cb *cb;
};

struct dma_info {
	spinlock_t lock;

	int can_dma;

	/* possibly 2 regions for double buffering */
	struct dma_region regions[2];
	int use_double_buffering;
	int current_buffer_index;

	int chan;
	int irq;		// TODO don't we need to explicitly acquire it?

	void __iomem *base;

	int use_interrupt;

	int busy;
};

struct bcm2708_pwm {
	/* TODO: this should probably be a semaphore now that we access it from interrupt context */
	struct mutex mutex;

	struct device* dev;

	void __iomem *base;
	int channel;

	int gpio;

	struct clock clock;	/* PWM-clock, so it's OK we manage this ourselves */

	struct dma_info dma;

	wait_queue_head_t io_queue;
	int blocking_io;
};

#if 0
static void debug_state(struct bcm2708_pwm *pwm) {
	static int dbg = 0;
	dbg++;
	if (dbg > 3)
		return;
	dev_info(pwm->dev, "Clock div: %x\n", readl(pwm->clock.div));
	dev_info(pwm->dev, "Clock ctl: %x\n", readl(pwm->clock.ctl));
	dev_info(pwm->dev, "PWM dmac: %x\n", readl(pwm->base + 0x8));
	dev_info(pwm->dev, "PWM ctl: %x\n", readl(pwm->base));
	dev_info(pwm->dev, "PWM sta: %x\n", readl(pwm->base + 0x4));
	dev_info(pwm->dev, "DMA ctl: %x\n", readl(pwm->dma.base));
	dev_info(pwm->dev, "DMA ti: %x\n", readl(pwm->dma.base + 0x8));
	dev_info(pwm->dev, "DMA debug: %x\n", readl(pwm->dma.base + 0x20));
}
#endif

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
	uint32_t divider;
	uint32_t div;

	if (pwm->clock.current_freq == target_frequency)
		return;

	if (target_frequency == 0) {
		pwm->clock.current_freq = 0;
		/* Since this shouldn't be called/reached when clock is busy,
		 * and the clock won't be re-enabled with freq = 0, we don't
		 * need to do anything else */
		return;
	}

	/* Use integer divider on the 19.2MHz oscillator, no MASH */
	divider = OSCILLATOR_FREQUENCY / target_frequency;
	div = CM_PASSWD | (divider << 12);

	writel(div, pwm->clock.div);
	pwm->clock.current_freq = target_frequency;

	dev_info(pwm->dev, "Set the divider to %x...\n", divider);
}

static void pwm_clock_enable(struct bcm2708_pwm *pwm)
{
	if (pwm->clock.enabled)
		return;

	if (pwm->clock.current_freq == 0) {
		dev_info(pwm->dev, "Won't enable a clock with freq=0...\n");
		return;
	}

	writel(CM_PASSWD | CM_CTL_CLOCK_OSC, pwm->clock.ctl);
	dev_dbg(pwm->dev, "Set the oscillator...\n");

	writel(CM_PASSWD | CM_CTL_CLOCK_OSC | CM_CTL_ENAB, pwm->clock.ctl);
	pwm->clock.enabled = 1;

	dev_dbg(pwm->dev, "Enabled clock...\n");
}

static void pwm_clock_disable(struct bcm2708_pwm *pwm)
{
	writel(CM_PASSWD, pwm->clock.ctl);
	pwm->clock.enabled = 0;
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

/* sysfs code */
static ssize_t pwm_show_current_freq(struct device_driver *driver, char *buf)
{
	/* TODO: container_of to get the pdev, it's associated data, and through it, the pwm */

	if (!global) {
		return snprintf(buf, PAGE_SIZE, "%s", "ERROR\n");
	}

	return snprintf(buf, PAGE_SIZE, "%i\n", global->clock.current_freq);
}

static DRIVER_ATTR(pwm_current_freq, S_IRUGO, pwm_show_current_freq,
		   NULL);

static struct attribute *pwm_dev_attrs[] = {
	&driver_attr_pwm_current_freq.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pwm_dev);

/* Does some generic preparation: enable clock, clear FIFO.
 * Optionally it enables the PWM's DMA configuration.
 * Then it enables PWM.
 */
#define BCM2708_PWM_STA_WERR_CLEAR (1<<2)
#define BCM2708_PWM_STA_RERR_CLEAR (1<<3)
#define BCM2708_PWM_STA_BERR_CLEAR (1<<8)
static void pwm_prepare_and_start(struct bcm2708_pwm *pwm, int start_dma)
{
	pwm_clock_enable(pwm);
	dev_dbg(pwm->dev, "Configured clock!\n");

	/* Some udelays here, inspired by Garff's code, because it *does*
	 * seem to lock up sometimes ... */

	/* Clear STA error bits */
	writel(  BCM2708_PWM_STA_WERR_CLEAR
	       | BCM2708_PWM_STA_RERR_CLEAR
	       | BCM2708_PWM_STA_BERR_CLEAR, pwm->base + 0x4);
	udelay(60);

	/* Clear fifo */
	set_pwm_ctl(pwm, CTL_CLRFIFO);
	writel(32, pwm->base + 0x10);
	udelay(60);

	if (start_dma) {
		pwm_enable_dma(pwm);
		udelay(60);
	}

	set_pwm_ctl(pwm, CTL_MODE1_SERIALIZE | CTL_USEFIFO1 | CTL_PWENABLE1);
	/* TODO: verify status? */
}

/* DMA-related code */

static int use_double_buffering = 0; /* doesn't yet work completely */
module_param(use_double_buffering, int, 0);


/* Returns a pointer to the dma region into which we need to write for the
 * next dma transfer */
static struct dma_region *dma_target_buffer(struct dma_info *dma) {
	if (!dma->use_double_buffering)
		return &dma->regions[0];
	return &dma->regions[1 - dma->current_buffer_index];
}

/* Manually acknowledge the IRQ to the DMA controller. This code should
 * really also be in mach-bcm2708/dma.c/h! (TODO)  */
#define BCM2708_DMA_INT_CLEAR (1 << 2)
#define BCM2708_DMA_END_CLEAR (1 << 1)
static void bcm_dma_irq_ack(void __iomem *dma_chan_base) {
	if (readl(dma_chan_base + BCM2708_DMA_CS) & BCM2708_DMA_INT_CLEAR)
		writel(BCM2708_DMA_INT_CLEAR | BCM2708_DMA_END_CLEAR,
		       dma_chan_base + BCM2708_DMA_CS);
}

irqreturn_t pwm_dma_interrupt_handler(int irq, void *dev_id) {
	/* TODO: should we use dev_id to pass along the dma struct? */

	spin_lock(&(global->dma.lock));
	bcm_dma_irq_ack(global->dma.base);

	global->dma.busy = 0;
	spin_unlock(&(global->dma.lock));

	wake_up_interruptible(&global->io_queue);

	return IRQ_HANDLED;
}

static int initialize_dma(struct bcm2708_pwm *pwm)
{
	/* We require DMA Channel 5. Try to get it using a HACK
	 * (proper way: add a bcm2708 function to request a specific channel) */
	int ret;
	int acquired_chans[5];
	int acquired_chans_count = 0;
	int i;
	int allocate_buffers;
	int allocated_buffers = 0;
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

	dev_info(dev, "Using double buffering: %i\n", use_double_buffering);
	dma->use_double_buffering = use_double_buffering;
	if (dma->use_double_buffering)
		allocate_buffers = 2;
	else
		allocate_buffers = 1;

	for (i = 0; i < allocate_buffers; i++) {
		struct dma_region *region = &dma->regions[i];

		region->buf =
			dma_alloc_coherent(dev, PAGE_SIZE, &(region->handle), 0);
		if (!region->buf) {
			dev_warn(dev, "Failed to allocate coherent DMA buffer!\n");
			goto release_channels;
		}

		region->allocated_size = PAGE_SIZE;
		region->start_offset = sizeof(struct bcm2708_dma_cb);
		region->usable_size = PAGE_SIZE - region->start_offset;
		region->used_size = 0;
		region->cb = region->buf;

		allocated_buffers++;
	}

	dma->can_dma = 1;
	dma->current_buffer_index = 0;

	spin_lock_init(&dma->lock);

	/* In case the state is not clean on module load: reset it */
	bcm_dma_irq_ack(dma->base);
	bcm_dma_abort(dma->base);

	dma->busy = 0;

	if (dma->irq) {
		/* TODO: fast irq? TODO I'm assuming: not shared? */
		if (dma->irq == IRQ_DMA5) {
			if (request_irq(dma->irq, pwm_dma_interrupt_handler,
					0, /* flags: not fast, not shared */
					DRV_NAME,
					NULL)) {
				dev_info(dev, "Could not get IRQ\n");
				dma->irq = 0;
			}
		} else {
			dev_info(dev, "Got IRQ: %i\n", dma->irq);
			dma->irq = 0;
		}
	}

	ret = 0;

release_channels:
	for (i = 0; i < allocated_buffers; i++) {
		struct dma_region *region = &dma->regions[i];
		dma_free_coherent(dev, region->allocated_size, region->buf,
			  region->handle);
	}

	if (!dma->can_dma && dma->chan > 0)
		bcm_dma_chan_free(dma->chan);

	for (i = 0; i < acquired_chans_count; i++)
		bcm_dma_chan_free(acquired_chans[i]);	/* TODO: check errors */

	return ret;
}

static void release_dma(struct platform_device *pdev, struct bcm2708_pwm *pwm)
{
	struct dma_info *dma = &pwm->dma;
	int i;

	if (!dma->can_dma)
		return;

	if (dma->irq)
		free_irq(dma->irq, NULL);

	bcm_dma_abort(dma->base);	/* TODO return value */

	bcm_dma_chan_free(dma->chan);

	for (i = 0; i < 1 + dma->use_double_buffering; i++) {
		struct dma_region *region = &dma->regions[i];
		dma_free_coherent(&pdev->dev, region->allocated_size,
				  region->buf, region->handle);
	}
}

/* This assumes the data is already in the pwm->dma buffer */
static void pwm_dma_output(struct bcm2708_pwm *pwm, size_t len /* in bytes */) {
	struct dma_info *dma = &pwm->dma;
	unsigned long cb_io;
	unsigned long buf_io;
	struct bcm2708_dma_cb *cb;
	unsigned long int_flags;
	struct dma_region *region;

	if (!dma->can_dma) {
		dev_warn(pwm->dev, "Cannot DMA! Aborting list output\n");
		return;
	}

	spin_lock_irqsave(&dma->lock, int_flags);

	region = &dma->regions[dma->current_buffer_index];

	buf_io = region->handle + region->start_offset;
	cb_io = region->handle;	/* Because the CB is currently at the page start */

	cb = region->cb;

	/* write 32 bit words, with no destination address increase, no bursts */
	cb->info = BCM2708_DMA_WAIT_RESP
	    | BCM2708_DMA_D_DREQ | BCM2708_DMA_S_INC | BCM2708_DMA_PER_MAP(5);

	if (dma->irq && pwm->blocking_io) {
		cb->info |= BCM2708_DMA_INT_EN;
	} else {
		dev_info(pwm->dev, "Not using interrupts\n");
	}

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

	pwm_prepare_and_start(pwm, 1 /* DMA */ );

	bcm_dma_start(dma->base, cb_io);

	dma->busy = 1;

	if (dma->use_double_buffering)
		dma->current_buffer_index = 1 - dma->current_buffer_index;

	spin_unlock_irqrestore(&dma->lock, int_flags);

	/* TODO: I want to set CTL_PWENABLE1 to 0 again when we are done, and disable the clock! */
}

static int setup_device(struct platform_device *pdev, struct bcm2708_pwm *pwm)
{
	int err = 0;
	int gpio = GPIO_PIN;
	struct gpio_chip *gc;

	struct resource *regs_pwm;
	struct resource *regs_cm_pwm;

	mutex_init(&pwm->mutex);

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

	init_waitqueue_head(&pwm->io_queue);

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
	pwm_clock_disable(pwm);

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
	struct class *class;
	dev_t dev;

	struct mutex mutex;

	struct bcm2708_pwm *pwm;
};

/* TODO: assume, for now, we only have one */
static struct bcm2708_pwm_cdev pwm_cdev_;
static struct bcm2708_pwm_cdev* pwm_cdev = NULL;

int pwm_cdev_open(struct inode *inode, struct file *filp)
{
	struct bcm2708_pwm *pwm;
	dev_info(pwm_cdev->pwm->dev, "cdev opened\n");

	pwm = pwm_cdev->pwm;
	mutex_lock(&pwm->mutex);
	pwm->blocking_io = 1; /* TODO: make configurable from userspace */
	mutex_unlock(&pwm->mutex);

	return 0;
}

int pwm_cdev_release(struct inode *inode, struct file *filp)
{
	dev_info(pwm_cdev->pwm->dev, "cdev closed\n");
	return 0;
}

/* Quick method for testing, no ioctl yet */
/* TODO: locking of pwm_cdev! */
ssize_t pwm_cdev_write(struct file *filp, const char __user *buf,
		      size_t count, loff_t* offp)
{
	struct dma_info *dma;
	struct bcm2708_pwm *pwm;
	struct dma_region *target_region;
	size_t buf_size;
	uint32_t *buf_virt;
	size_t out = count;

	if (!pwm_cdev) {
		printk(KERN_WARNING "bcm2708_pwm: no pwm cdevice!\n");
		return -ENOMEM;
	}

	pwm = pwm_cdev->pwm;

	dma = &pwm->dma;

	if (!dma->can_dma) {
		dev_warn(pwm->dev, "Cannot DMA! Aborting list output\n");
		out = -EFAULT;
		goto out;
	}

	target_region = dma_target_buffer(dma);
	buf_size = target_region->usable_size;
	buf_virt = target_region->buf + target_region->start_offset;

	if (count > buf_size) {
		out = -ENOMEM;
		goto out;
	}

	/* If we use double buffering, the next buffer can be overwritten
	   before the previous transfer has finished. */
	if (dma->use_double_buffering) {
		if (copy_from_user(buf_virt, buf, count)) {
			out = -EFAULT;
			goto out;
		}
	}

	spin_lock(&dma->lock);
	while (dma->busy) {
		spin_unlock(&dma->lock);
		if (wait_event_interruptible(pwm->io_queue,
					     !dma->busy))
			return -ERESTARTSYS; /* Signal caught */
		spin_lock(&dma->lock);
	}
	spin_unlock(&dma->lock);

	/* No double buffering => we can only overwrite the region
	 * when any previous transfer has finished */
	if (!dma->use_double_buffering) {
		if (copy_from_user(buf_virt, buf, count)) {
			out = -EFAULT;
			goto out;
		}
	}

	pwm_dma_output(pwm, count);

	/* Don't update offp for now */

out:
	return out;
}

long pwm_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct bcm2708_pwm *pwm;
	long ret = 0;

	pwm = pwm_cdev->pwm;

	if (_IOC_TYPE(cmd) != PWM_BCM2708_IOCTL_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > PWM_BCM2708_IOCTL_MAX)
		return -ENOTTY;

	switch(cmd) {
	case PWM_BCM2708_IOCTL_GET_FREQUENCY:
		return pwm->clock.current_freq;

	case PWM_BCM2708_IOCTL_SET_FREQUENCY:
		mutex_lock(&pwm->mutex);
		/* TODO: should fail when clock is busy! */
		pwm_clock_set_div(pwm, (uint32_t)arg);
		mutex_unlock(&pwm->mutex);
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static struct file_operations pwm_cdev_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl	= pwm_cdev_ioctl,
	.open	= pwm_cdev_open,
	.release= pwm_cdev_release,
	.write	= pwm_cdev_write
};

static int init_pwm_cdev(struct bcm2708_pwm *pwm)
{
	int err;
	dev_t dev;

	pwm_cdev_.class = class_create(THIS_MODULE, "bcm2708_pwm");
	if(IS_ERR(pwm_cdev_.class)) {
		dev_warn(pwm->dev, "Could not allocate class\n");
		return -ENOMEM;
	}


	err = alloc_chrdev_region(&dev, 0, 1, "bcm2708_pwm");
	if (err) {
		dev_warn(pwm->dev, "Could not allocate chrdev region\n");
		goto out_class;
	}

	cdev_init(&pwm_cdev_.cdev, &pwm_cdev_fops);
	pwm_cdev_.cdev.owner = THIS_MODULE;

	err = cdev_add(&pwm_cdev_.cdev, dev, 1);
	if (err) {
		dev_warn(pwm->dev, "Could not add cdev\n");
		goto out;
	}

	/* TODO: add more attributes to match on! */
	if (!device_create(pwm_cdev_.class, NULL, pwm_cdev_.cdev.dev,
			   NULL, "bcm2708_pwm0")) {
		dev_warn(pwm->dev, "Could not create class device\n");
		err = -ENOMEM;
		goto out_cdev;
	}

	mutex_init(&pwm_cdev_.mutex);
	pwm_cdev_.pwm = pwm;
	pwm_cdev_.dev = dev;

	pwm_cdev = &pwm_cdev_;
	return 0;

out_cdev:
	cdev_del(&pwm_cdev_.cdev);

out_class:
	class_destroy(pwm_cdev_.class);

out:
	unregister_chrdev_region(dev, 1);
	return err;
}

static void release_pwm_cdev(struct bcm2708_pwm_cdev* pwm_cdev)
{
	device_destroy(pwm_cdev->class, pwm_cdev->dev);
	class_destroy(pwm_cdev->class);
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

