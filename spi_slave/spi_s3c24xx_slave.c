/* linux/drivers/char/s3c24xx_spi.c

   National Semiconductor SCx200 GPIO driver.  Allows a user space
   process to play with the GPIO pins.

   Copyright (c) 2001,2002 Christer Weinigel <wingel@nano-system.com> */
	   
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
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
//void __iomem		*regs;
//struct clk		*clk;
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
#if INT_OP
#ifdef CONFIG_SPI_S3C24XX_FIQ
#include <plat/fiq.h>
#include <asm/fiq.h>
#include "spi_s3c24xx_fiq.h"
//unsigned char		 fiq_claimed;
//struct fiq_handler	 fiq_handler;
//unsigned char		*rx;
//const unsigned char	*tx;
//int			 len;

enum spi_fiq_mode {
	FIQ_MODE_NONE	= 0,
	FIQ_MODE_TX	= 1,
	FIQ_MODE_RX	= 2,
	FIQ_MODE_TXRX	= 3,
};
//enum spi_fiq_mode mode;
//enum spi_fiq_mode	 fiq_mode;
//unsigned char		 fiq_inuse;
/* Support for FIQ based pseudo-DMA to improve the transfer speed.
 *
 * This code uses the assembly helper in spi_s3c24xx_spi.S which is
 * used by the FIQ core to move data between main memory and the peripheral
 * block. Since this is code running on the processor, there is no problem
 * with cache coherency of the buffers, so we can use any buffer we like.
 */

/**
 * struct spi_fiq_code - FIQ code and header
 * @length: The length of the code fragment, excluding this header.
 * @ack_offset: The offset from @data to the word to place the IRQ ACK bit at.
 * @data: The code itself to install as a FIQ handler.
 */
struct spi_fiq_code {
	u32	length;
	u32	ack_offset;
	u8	data[0];
};

extern struct spi_fiq_code s3c24xx_spi_fiq_txrx;
extern struct spi_fiq_code s3c24xx_spi_fiq_tx;
extern struct spi_fiq_code s3c24xx_spi_fiq_rx;
#endif
#endif
static struct platform_device *pdev;
 
void __iomem		*regs;
int			 irq;
int			 len;
int			 count;
#if INT_OP
#ifdef CONFIG_SPI_S3C24XX_FIQ

struct fiq_handler	 fiq_handler;
enum spi_fiq_mode	 fiq_mode;
unsigned char		 fiq_inuse;
unsigned char		 fiq_claimed=0;
#endif
#endif
//void			(*set_cs)(struct s3c2410_spi_info *spi,
//				  int cs, int pol);

/* data buffers */
const unsigned char	*tx;
unsigned char		*rx;
//int r_len;
//int w_len;
//char *g_buf;

struct clk		*clk;
struct resource		*ioarea;
//struct spi_master	*master;
//struct spi_device	*curdev;
//struct device		*dev;
//struct s3c2410_spi_info *pdata;


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
	unsigned long remaining;
	if (mutex_lock_interruptible(&tlclk_mutex))
		return -EINTR;		
	wait_event_interruptible(wq, got_event);
	
	read=1024;
	remaining = copy_to_user(buf, g_buf, 1024);
	got_event = 0;
	if (remaining)
	{
		mutex_unlock(&tlclk_mutex);
		return -EFAULT;
	}

	*ppos += 1024;
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
	//struct s3c24xx_spi *hw= (struct s3c24xx_spi *)buf_id;
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

static int acquire_dma(unsigned int rx_dmach,void *buf_id)
{
	void __iomem *regs= (void __iomem *)buf_id;
	if (s3c2410_dma_request(rx_dmach,
				&s3c24xx_spi_dma_client, NULL) < 0) {
		printk("cannot get RxDMA\n");
		return 0;
	}
	s3c2410_dma_set_buffdone_fn(rx_dmach, s3c24xx_spi_dma_rxcb);
	s3c2410_dma_devconfig(rx_dmach, S3C2410_DMASRC_HW,regs + S3C2410_SPRDAT);
	return 1;
}
#else
#ifdef CONFIG_SPI_S3C24XX_FIQ
/**
 * ack_bit - turn IRQ into IRQ acknowledgement bit
 * @irq: The interrupt number
 *
 * Returns the bit to write to the interrupt acknowledge register.
 */
static inline u32 ack_bit(unsigned int irq)
{
	return 1 << (irq - IRQ_EINT0);
}

/**
 * s3c24xx_spi_tryfiq - attempt to claim and setup FIQ for transfer
 * @hw: The hardware state.
 *
 * Claim the FIQ handler (only one can be active at any one time) and
 * then setup the correct transfer code for this transfer.
 *
 * This call updates all the necessary state information if successful,
 * so the caller does not need to do anything more than start the transfer
 * as normal, since the IRQ will have been re-routed to the FIQ handler.
*/
void s3c24xx_spi_tryfiq(void *hw)
{
	void __iomem *regs= (void __iomem *)hw;
	struct pt_regs pregs;
	enum spi_fiq_mode mode;
	struct spi_fiq_code *code;
	int ret;

	if (!fiq_claimed) {
		/* try and claim fiq if we haven't got it, and if not
		 * then return and simply use another transfer method */

		ret = claim_fiq(&fiq_handler);
		if (ret)
			return;
	}

	//if (hw->tx && !hw->rx)
	//	mode = FIQ_MODE_TX;
	//else if (hw->rx && !hw->tx)
		mode = FIQ_MODE_RX;
	//else
	//	mode = FIQ_MODE_TXRX;

	pregs.uregs[fiq_rspi] = (long)regs;
	pregs.uregs[fiq_rrx]  = (long)g_buf;
	pregs.uregs[fiq_rtx]  = (long)g_buf + 1;
	pregs.uregs[fiq_rcount] = len - 1;
	pregs.uregs[fiq_rirq] = (long)S3C24XX_VA_IRQ;

	set_fiq_regs(&pregs);

	if (fiq_mode != mode) {
		u32 *ack_ptr;

		fiq_mode = mode;

		switch (mode) {
		case FIQ_MODE_TX:
			code = &s3c24xx_spi_fiq_tx;
			break;
		case FIQ_MODE_RX:
			code = &s3c24xx_spi_fiq_rx;
			break;
		case FIQ_MODE_TXRX:
			code = &s3c24xx_spi_fiq_txrx;
			break;
		default:
			code = NULL;
		}

		BUG_ON(!code);

		ack_ptr = (u32 *)&code->data[code->ack_offset];
		*ack_ptr = ack_bit(IRQ_SPI1);

		set_fiq_handler(&code->data, code->length);
	}

	s3c24xx_set_fiq(IRQ_SPI1, true);
	enable_fiq(IRQ_SPI1);
	fiq_mode = mode;
	fiq_inuse = 1;
}

/**
 * s3c24xx_spi_fiqop - FIQ core code callback
 * @pw: Data registered with the handler
 * @release: Whether this is a release or a return.
 *
 * Called by the FIQ code when another module wants to use the FIQ, so
 * return whether we are currently using this or not and then update our
 * internal state.
 */
static int s3c24xx_spi_fiqop(void *pw, int release)
{
	//struct s3c24xx_spi *hw = pw;
	int ret = 0;

	if (release) {
		if (fiq_inuse)
			ret = -EBUSY;

		/* note, we do not need to unroute the FIQ, as the FIQ
		 * vector code de-routes it to signal the end of transfer */

		fiq_mode = FIQ_MODE_NONE;
		fiq_claimed = 0;
	} else {
		fiq_claimed = 1;
	}

	return ret;
}

/**
 * s3c24xx_spi_initfiq - setup the information for the FIQ core
 * @hw: The hardware state.
 *
 * Setup the fiq_handler block to pass to the FIQ core.
 */
static inline void s3c24xx_spi_initfiq(void __iomem *hw)
{
	fiq_handler.dev_id = hw;
	fiq_handler.name = "fiq-spi";//dev_name(hw->dev);
	fiq_handler.fiq_op = s3c24xx_spi_fiqop;
}

/**
 * s3c24xx_spi_usingfiq - return if channel is using FIQ
 * @spi: The hardware state.
 *
 * Return whether the channel is currently using the FIQ (separate from
 * whether the FIQ is claimed).
 */
static inline bool s3c24xx_spi_usingfiq(void)
{
	return fiq_inuse;
}
#else

static inline void s3c24xx_spi_initfiq(void) { }
static inline void s3c24xx_spi_tryfiq(void *f) { }
static inline bool s3c24xx_spi_usingfiq(void) { return false; }

#endif /* CONFIG_SPI_S3C24XX_FIQ */
static irqreturn_t s3c24xx_spi_irq(int irq, void *dev)
{	
	void __iomem *regs= (void __iomem *)dev;
	unsigned long flags;
	unsigned char spsta;
	spin_lock_irqsave(&event_lock, flags);
	spsta = readb(regs + S3C2410_SPSTA);
	if (spsta & S3C2410_SPSTA_DCOL) {
		printk("data-collision\n");
		goto irq_done;
	}
	if (spsta & S3C2410_SPSTA_READY) 
	{		
		fiq_inuse = 0;
		g_buf[len-1]=readb(regs + S3C2410_SPRDAT);
		got_event = 1;
		wake_up(&wq);
		len=1024;
		s3c24xx_spi_tryfiq(regs);
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
	//struct resource 	*ioarea;

	/* support dev_dbg() with pdev->dev */
	pdev = platform_device_alloc(DRVNAME, 0);
	if (!pdev)
		return -ENOMEM;

	rc = platform_device_add(pdev);
	if (rc)
		goto undo_malloc;
	//hw=(struct s3c24xx_spi *)kmalloc(sizeof(struct s3c24xx_spi),GFP_KERNEL);
	my_s3c24xx_spi_gpiocfg_bus1_gpg5_6_7(1);
	clk = clk_get(&pdev->dev, "spi");
	if (IS_ERR(clk)) {
		printk("No clock for device\n");
		return 2;
	}
	g_buf=kmalloc(1024*sizeof(unsigned char),GFP_KERNEL);
	memset(g_buf,0,1024);

	ioarea = request_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]),DRVNAME);
	if (ioarea == NULL) {
		printk("Cannot reserve region1\n");
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
	writeb((S3C2410_SPCON_SMOD_INT|S3C2410_SPCON_CPOL_LOW|S3C2410_SPCON_CPHA_FMTB),regs+S3C2410_SPCON);
	err = request_irq(IRQ_SPI1, (void *)s3c24xx_spi_irq, 0, DRVNAME, regs);
	if (err) {
		printk("Cannot claim IRQ\n");
		return 2;
	}
#else	
	acquire_dma(DMACH_SPI1,(void *)regs);
	rx_dma = dma_map_single(&pdev->dev, g_buf,1024, DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdev->dev, rx_dma)) {
		printk("dma_map_single Rx failed\n");
		rx_dma = XFER_DMAADDR_INVALID;
		iounmap((void *) regs);
		release_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]));
		return -ENOMEM;
	}
	writeb((S3C2410_SPCON_SMOD_DMA|S3C2410_SPCON_CPOL_HIGH|S3C2410_SPCON_CPHA_FMTB),regs+S3C2410_SPCON);
	s3c2410_dma_config(DMACH_SPI1, 32 / 8);
	s3c2410_dma_enqueue(DMACH_SPI1, (void *)regs,	rx_dma, 64);
	s3c2410_dma_ctrl(DMACH_SPI1, S3C2410_DMAOP_START);
#endif
	writeb(S3C2410_SPPIN_RESERVED,regs+S3C2410_SPPIN);

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
#if INT_OP
#ifdef CONFIG_SPI_S3C24XX_FIQ
len=1024;
s3c24xx_spi_initfiq(regs);
s3c24xx_spi_tryfiq(regs);
#endif
#endif
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
	clk_disable(clk);
	release_mem_region(s3c2440_spi1_resource[0].start, resource_size(&s3c2440_spi1_resource[0]));
	cdev_del(&s3c24xx_spi_cdev);
	/* cdev_put(&s3c24xx_spi_cdev); */

	unregister_chrdev_region(MKDEV(major, 0), MAX_PINS);
	platform_device_unregister(pdev);
}

module_init(s3c24xx_spi_init);
module_exit(s3c24xx_spi_cleanup);
