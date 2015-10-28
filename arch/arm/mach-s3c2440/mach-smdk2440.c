/* linux/arch/arm/mach-s3c2440/mach-smdk2440.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.fluff.org/ben/smdk2440/
 *
 * Thanks to Dimity Andric and TomTom for the loan of an SMDK2440.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <plat/mci.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>

#include <mach/idle.h>
#include <mach/fb.h>
#include <plat/iic.h>

#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <plat/s3c2410.h>
#include <plat/s3c244x.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <net/smdk2410.h>
#include <plat/common-smdk.h>
#include <linux/spi/spi.h>
#include <linux/spi/libertas_spi.h>
#include <mach/spi.h>
static struct map_desc smdk2440_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	},{
		.virtual = vSMDK2410_ETH_IO, 
		.pfn = pSMDK2410_ETH_IO, 
		.length = SZ_1M, 
		.type = MT_DEVICE,
		}
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg smdk2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

static struct s3c2410fb_display smdk2440_lcd_cfg __initdata = {

	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 240,
	.height		= 320,

	.pixclock	= 166667, /* HCLK 60 MHz, divisor 10 */
	.xres		= 240,
	.yres		= 320,
	.bpp		= 16,
	.left_margin	= 20,
	.right_margin	= 8,
	.hsync_len	= 4,
	.upper_margin	= 8,
	.lower_margin	= 7,
	.vsync_len	= 4,
};

static struct s3c2410fb_mach_info smdk2440_fb_info __initdata = {
	.displays	= &smdk2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

#if 0
	/* currently setup by downloader */
	.gpccon		= 0xaa940659,
	.gpccon_mask	= 0xffffffff,
	.gpcup		= 0x0000ffff,
	.gpcup_mask	= 0xffffffff,
	.gpdcon		= 0xaa84aaa0,
	.gpdcon_mask	= 0xffffffff,
	.gpdup		= 0x0000faff,
	.gpdup_mask	= 0xffffffff,
#endif

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

/* MMC/SD  */

static struct s3c24xx_mci_pdata mini2440_mmc_cfg __initdata = {
   .gpio_detect   = S3C2410_GPG(8),
   .gpio_wprotect = S3C2410_GPH(8),
   .set_power     = NULL,
   .ocr_avail     = MMC_VDD_32_33|MMC_VDD_33_34,
};
/* WiFi */
static int z2_lbs_spi_setup(struct spi_device *spi)
{
	int ret = 0;
	//s3c24xx_spi_gpiocfg_bus0_gpe11_12_13(spi,true);
		s3c_gpio_cfgpin(S3C2410_GPG(7), S3C2410_GPG7_SPICLK1);
		s3c_gpio_cfgpin(S3C2410_GPG(6), S3C2410_GPG6_SPIMOSI1);
		s3c_gpio_cfgpin(S3C2410_GPG(5), S3C2410_GPG5_SPIMISO1);
		s3c2410_gpio_pullup(S3C2410_GPG(5), 0);
		s3c2410_gpio_pullup(S3C2410_GPG(6), 0);
	//s3c_gpio_cfgpin(S3C2410_GPE(13), S3C2410_GPE13_SPICLK0);
	//s3c_gpio_cfgpin(S3C2410_GPE(12), S3C2410_GPE12_SPIMOSI0);
	//s3c_gpio_cfgpin(S3C2410_GPE(11), S3C2410_GPE11_SPIMISO0);
	//s3c2410_gpio_pullup(S3C2410_GPE(11), 0);
	//s3c2410_gpio_pullup(S3C2410_GPE(13), 0);
	//s3c2410_gpio_pullup(S3C2410_GPE(12), 0);
	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_2;

	spi_setup(spi);

	return ret;
};

static int z2_lbs_spi_teardown(struct spi_device *spi)
{
	return 0;

};

static struct s3c2410_spi_info z2_lbs_chip_info = {
	.num_cs		= 1,
	.bus_num	= 1,
	.pin_cs	= S3C2410_GPG(3),
	.use_fiq = 1,
	.gpio_setup = s3c24xx_spi_gpiocfg_bus1_gpg5_6_7,//s3c24xx_spi_gpiocfg_bus0_gpe11_12_13,
};

static struct libertas_spi_platform_data z2_lbs_pdata = {
	.use_dummy_writes	= 1,
	.setup			= z2_lbs_spi_setup,
	.teardown		= z2_lbs_spi_teardown,
};

static struct spi_board_info spi_board_info[] __initdata = {
{
	.modalias		= "libertas_spi",
	.platform_data		= &z2_lbs_pdata,
	.controller_data	= &z2_lbs_chip_info,
	.irq			= IRQ_EINT1,
	.max_speed_hz		= 3000000,
	.bus_num		= 1,
	.chip_select		= 0,
	//.mode			= SPI_CPHA|SPI_CPOL,
},
};

static void __init z2_spi_init(void)
{
	s3c_device_spi1.dev.platform_data= &z2_lbs_chip_info;
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}
static struct platform_device *smdk2440_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&s3c_device_rtc,
	&s3c_device_sdi,
	&s3c_device_spi1,
};

static void __init smdk2440_map_io(void)
{
	s3c24xx_init_io(smdk2440_iodesc, ARRAY_SIZE(smdk2440_iodesc));
	s3c24xx_init_clocks(16934400);
	s3c24xx_init_uarts(smdk2440_uartcfgs, ARRAY_SIZE(smdk2440_uartcfgs));
}

static void __init smdk2440_machine_init(void)
{
	s3c24xx_fb_set_platdata(&smdk2440_fb_info);
	s3c_i2c0_set_platdata(NULL);
	s3c24xx_mci_set_platdata(&mini2440_mmc_cfg);

	z2_spi_init();
	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));
	smdk_machine_init();
}

MACHINE_START(S3C2440, "SMDK2440")
	/* Maintainer: Ben Dooks <ben-linux@fluff.org> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2440_map_io,
	.init_machine	= smdk2440_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
