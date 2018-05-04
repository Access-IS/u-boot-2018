/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>
 *
 * Based on SPL code from Solidrun tree, which is:
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *
 * based on mx6cuboxi from solidrun
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/io.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <netdev.h>
#include <micrel.h>
#include <phy.h>
#include <input.h>
#include <i2c.h>
#include <spl.h>
#include <pwm.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
	
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)


#define ECSPI3_CS0              IMX_GPIO_NR(4, 24)



int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};


static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}


static iomux_v3_cfg_t const ecspi3_pads[] = {
	IOMUX_PADS(PAD_DISP0_DAT3__GPIO4_IO24  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
        return (bus == 2 && cs == 0) ? ECSPI3_CS0 : -1;
}

void setup_spi(void)
{
        SETUP_IOMUX_PADS(ecspi3_pads);

        enable_spi_clk(true, 2);

        /* set cs0 to high */
        gpio_direction_output(ECSPI3_CS0, 1);

}

/*  pwm features addition
 */

#define PWM_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const pwm_pads[] = {
	IOMUX_PADS(PAD_GPIO_1__PWM2_OUT  | MUX_PAD_CTRL(PWM_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__PWM3_OUT  | MUX_PAD_CTRL(PWM_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__PWM4_OUT   | MUX_PAD_CTRL(PWM_PAD_CTRL)),
};


void setup_pwm_led (void) {
		SETUP_IOMUX_PADS(pwm_pads);
		pwm_init(1, 0, 0);
		pwm_config(1, 836601, 3333333);
		pwm_enable(1);
		pwm_init(2, 0, 0);
		pwm_config(2, 1666667, 3333333);
	//	pwm_enable(2);
		pwm_init(3, 0, 0);
		pwm_config(3, 836601, 3333333);
		pwm_enable(3);
		
}

int board_early_init_f(void)
{
	int ret = 0;
	setup_iomux_uart();
	setup_pwm_led();

	return ret;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
	setup_spi();
	return 0;
}


int checkboard(void)
{
	puts("Board: Access-IS Swallow\n");
	return 0;
}

int board_late_init(void)
{
	return 0;
}

