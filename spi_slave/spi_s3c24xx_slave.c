/* linux/drivers/char/s3c24xx_spi.c

   National Semiconductor SCx200 GPIO driver.  Allows a user space
   process to play with the GPIO pins.

   Copyright (c) 2001,2002 Christer Weinigel <wingel@nano-system.com> */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <mach/spi.h>
#include <mach/dma.h>
#include <mach/regs-gpio.h>
#include <plat/regs-spi.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/cdev.h>
#define INT_OP 1
#define XFER_DMAADDR_INVALID DMA_BIT_MASK(32)
void __iomem		*regs;
struct clk		*clk;
#if INT_OP
static irqreturn_t s3c24xx_spi_irq(int irq, void *dev);
#endif
static DEFINE_MUTEX(tlclk_mutex);
static DECLARE_WAIT_QUEUE_HEAD(wq);
static DEFINE_SPINLOCK(event_lock);

static int got_event;		/* if events processing have been done */
int r_len=0,w_len=0;
char *g_buf=NULL;
#define DRVNAME "s3c24xx_spi_slave"

static struct platform_device *pdev;

MODULE_AUTHOR("Christer Weinigel <wingel@nano-system.com>");
MODULE_DESCRIPTION("NatSemi/AMD SCx200 GPIO Pin Driver");
MODULE_LICENSE("GPL");

static int major = 0;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

#define MAX_PINS 32		/* 64 later, when known ok */

void my_s3c24xx_spi_gpiocfg_bus1_gpg5_6_7(int enable)
{
	if (enable) {
		s3c_gpio_cfgpin(S3C2410_GPG(7), S3C2410_GPG7_SPICLK1);
		s3c_gpio_cfgpin(S3C2410_GPG(6), S3C2410_GPG6_SPIMOSI1);
		s3c_gpio_cfgpin(S3C2410_GPG(5), S3C2410_GPG5_SPIMISO1);
		s3c2410_gpio_pullup(S3C2410_GPG(5), 0);
		s3c2410_gpio_pullup(S3C2410_GPG(6), 0);
	} else {
		s3c_gpio_cfgpin(S3C2410_GPG(7), S3C2410_GPIO_INPUT);
		s3c_gpio_cfgpin(S3C2410_GPG(5), S3C2410_GPIO_INPUT);
		s3c_gpio_setpull(S3C2410_GPG(5), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S3C2410_GPG(6), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S3C2410_GPG(7), S3C_GPIO_PULL_NONE);
	}
}
static int s3c24xx_spi_open(struct inode *inode, struct file *file)
{
	unsigned m = iminor(inode);
	//file->private_data = &s3c24xx_spi_ops;
	if (m >= MAX_PINS)
		return -EINVAL;
	return nonseekable_open(inode, file);
}

static int s3c24xx_spi_release(struct inode *inode, struct file *file)
{
	return 0;
}
static ssize_t s3c24xx_spi_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t read=0;
	int copy_len=0;
	unsigned long remaining;
	if (mutex_lock_interruptible(&tlclk_mutex))
		return -EINTR;		
	wait_event_interruptible(wq, got_event);
	if(w_len>r_len)
	{
		if(count>(w_len-r_len))
			copy_len=w_len-r_len;
		else
			copy_len=count;
	}
	else
	{
		if(count>(1024+w_len-r_len))
			copy_len=1024+w_len-r_len;
		else
			copy_len=count;
	}
	read=copy_len;
	printk("can read %d bytes\n",copy_len);

	remaining = copy_to_user(buf, g_buf, copy_len);
	got_event = 0;
	if (remaining)
	{
		mutex_unlock(&tlclk_mutex);
		return -EFAULT;
	}

	*ppos += copy_len;
	if((r_len+copy_len)<1024)
		r_len=r_len+copy_len;
	else
		r_len=r_len+copy_len-1024;
	mutex_unlock(&tlclk_mutex);
	return read;
}

static const struct file_operations s3c24xx_spi_fileops = {
	.owner   = THIS_MODULE,
	.read    = s3c24xx_spi_read,
	.open    = s3c24xx_spi_open,
	.release = s3c24xx_spi_release,
};

static struct cdev s3c24xx_spi_cdev;  /* use 1 cdev for all pins */

static struct resource s3c2440_spi1_resource[] = {
	[0] = {
		.start = S3C24XX_PA_SPI+S3C2410_SPI1,
		.end   = S3C24XX_PA_SPI+S3C2410_SPI1 + S3C24XX_SZ_SPI -1,
		.flags = IORESOURCE_MEM
	},		
	[1] = {
		.start = DMACH_SPI1,
		.end   = DMACH_SPI1,
		.flags = IORESOURCE_DMA,
	},
};
#if !INT_OP
static struct s3c2410_dma_client s3c24xx_spi_dma_client = {
	.name = "samsung-spi-dma",
};
static void s3c24xx_spi_dma_rxcb(struct s3c2410_dma_chan *chan, void *buf_id,
		int size, enum s3c2410_dma_buffresult res)
{
	//void __iomem *regs = buf_id;
	unsigned long flags;

	spin_lock_irqsave(&event_lock, flags);

	if (res == S3C2410_RES_OK)
	{
		//sdd->state &= ~RXBUSY;
		//info read process
		printk("dma rcv done\n");
		got_event = 1;
		wake_up(&wq);
		//s3c2410_dma_ctrl(DMACH_SPI1, S3C2410_DMAOP_START);
	}
	else
	{
		printk("DmaAbrtRx-%d\n", size);
		s3c2410_dma_ctrl(DMACH_SPI1,S3C2410_DMAOP_FLUSH);
	}
	
	spin_unlock_irqrestore(&event_lock, flags);
}

static int acquire_dma(unsigned int rx_dmach,unsigned long reg)
{
	if (s3c2410_dma_request(rx_dmach,
				&s3c24xx_spi_dma_client, NULL) < 0) {
		printk("cannot get RxDMA\n");
		return 0;
	}
	s3c2410_dma_set_buffdone_fn(rx_dmach, s3c24xx_spi_dma_rxcb);
	s3c2410_dma_devconfig(rx_dmach, S3C2410_DMASRC_HW,reg + S3C2410_SPRDAT);
	return 1;
}
#else
static irqreturn_t s3c24xx_spi_irq(int irq, void *dev)
{
	void __iomem	*regs = dev;
	unsigned long flags;
	unsigned char spsta = readb(regs + S3C2410_SPSTA);
	spin_lock_irqsave(&event_lock, flags);

	if (spsta & S3C2410_SPSTA_DCOL) {
		printk("data-collision\n");
		goto irq_done;
	}
	if (spsta & S3C2410_SPSTA_READY) 
	{		
			if(w_len==1023)
			{
				w_len=0;
			}
			g_buf[w_len++]=readb(regs + S3C2410_SPRDAT);
			//printk(" %x",g_buf[w_len-1]);
			//if((w_len%16)==0)
			//printk("\n");
		got_event = 1;
		wake_up(&wq);
	}

irq_done:
	spin_unlock_irqrestore(&event_lock, flags);
	return IRQ_HANDLED;
}
#endif
static int __init s3c24xx_spi_init(void)
{
	int rc;
#if INT_OP
	int err = 0;
#else
	dma_addr_t		rx_dma;
#endif
	dev_t devid;
	struct resource 	*ioarea;

	/* support dev_dbg() with pdev->dev */
	pdev = platform_device_alloc(DRVNAME, 0);
	if (!pdev)
		return -ENOMEM;

	rc = platform_device_add(pdev);
	if (rc)
		goto undo_malloc;

	clk = clk_get(&pdev->dev, "spi");
	if (IS_ERR(clk)) {
		printk("No clock for device\n");
		return 2;
	}
	g_buf=kmalloc(1024*sizeof(unsigned char),GFP_KERNEL);
	memset(g_buf,0,1024);

	ioarea = request_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]),DRVNAME);
	if (ioarea == NULL) {
		printk("Cannot reserve region\n");
		return 0;
	}
	regs = ioremap(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]));
	if (regs == NULL) {
		printk("Cannot map IO\n");
		return 1;
	}
	clk_enable(clk);
	writeb(0x00,regs+S3C2410_SPPRE);
#if INT_OP
	err = request_irq(IRQ_SPI1, (void *)s3c24xx_spi_irq, 0, DRVNAME, (void *)regs);
	if (err) {
		printk("Cannot claim IRQ\n");
		return 2;
	}
	writeb((S3C2410_SPCON_SMOD_INT|S3C2410_SPCON_CPOL_HIGH|S3C2410_SPCON_CPHA_FMTB),regs+S3C2410_SPCON);
#else	
	acquire_dma(DMACH_SPI1,(unsigned long)regs);
	rx_dma = dma_map_single(&pdev->dev, g_buf,1024, DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdev->dev, rx_dma)) {
		printk("dma_map_single Rx failed\n");
		rx_dma = XFER_DMAADDR_INVALID;
		iounmap((void *) regs);
		release_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]));
		return -ENOMEM;
	}
	writeb((S3C2410_SPCON_SMOD_DMA|S3C2410_SPCON_CPOL_LOW|S3C2410_SPCON_CPHA_FMTB),regs+S3C2410_SPCON);
	s3c2410_dma_config(DMACH_SPI1, 32 / 8);
	s3c2410_dma_enqueue(DMACH_SPI1, (void *)regs,	rx_dma, 64);
	s3c2410_dma_ctrl(DMACH_SPI1, S3C2410_DMAOP_START);
#endif
	writeb(S3C2410_SPPIN_RESERVED,regs+S3C2410_SPPIN);
	my_s3c24xx_spi_gpiocfg_bus1_gpg5_6_7(1);
	
	//s3c24xx_spi_ops.dev = &pdev->dev;
	if (major) {
		devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid, MAX_PINS, "s3c24xx_spi");
	} else {
		rc = alloc_chrdev_region(&devid, 0, MAX_PINS, "s3c24xx_spi");
		major = MAJOR(devid);
	}
	if (rc < 0) {
		dev_err(&pdev->dev, "SCx200 chrdev_region err: %d\n", rc);
		goto undo_platform_device_add;
	}

	cdev_init(&s3c24xx_spi_cdev, &s3c24xx_spi_fileops);
	cdev_add(&s3c24xx_spi_cdev, devid, MAX_PINS);

	return 0; /* succeed */

undo_platform_device_add:
	platform_device_del(pdev);
undo_malloc:
	platform_device_put(pdev);

	return rc;
}

static void __exit s3c24xx_spi_cleanup(void)
{
#if INT_OP
	free_irq(IRQ_SPI1, regs);
#else
	s3c2410_dma_free(DMACH_SPI1, &s3c24xx_spi_dma_client);
#endif

	iounmap((void *) regs);
	release_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]));
	cdev_del(&s3c24xx_spi_cdev);
	/* cdev_put(&s3c24xx_spi_cdev); */

	unregister_chrdev_region(MKDEV(major, 0), MAX_PINS);
	platform_device_unregister(pdev);
}

module_init(s3c24xx_spi_init);
module_exit(s3c24xx_spi_cleanup);
