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

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)


#define ETH_PHY_RESET	IMX_GPIO_NR(4, 15)
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

// ethernet
static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8035 reset */
	IOMUX_PADS(PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	/* AR8035 interrupt */
	IOMUX_PADS(PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* 25MHz clock from IMX6 to AR8035 */
	IOMUX_PADS(PAD_GPIO_16__ENET_REF_CLK	  | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	  | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8035 CLK_25M to IMX6 ref clock */
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL_CLK)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
};


static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	gpio_direction_output(ETH_PHY_RESET, 0);
	mdelay(10);
	gpio_set_value(ETH_PHY_RESET, 1);
	udelay(100);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#define ETH_PHY_MASK	((1 << 0x0) | (1 << 0x4))
int board_eth_init(bd_t *bis)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct mii_dev *bus;
	struct phy_device *phydev;

	int ret = enable_fec_anatop_clock(0, ENET_25MHZ);
	if (ret)
		return ret;

	/* set gpr1[ENET_CLK_SEL] */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);

	setup_iomux_enet();

	bus = fec_get_miibus(IMX_FEC_BASE, -1);
	if (!bus)
		return -EINVAL;

	phydev = phy_find_by_mask(bus, ETH_PHY_MASK, PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}

	debug("using phy at address %d\n", phydev->addr);
	ret = fec_probe(bis, -1, IMX_FEC_BASE, bus, phydev);
	if (ret)
		goto free_phydev;

	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
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
	puts("Board: Access-IS ATR200B\n");
	return 0;
}

int board_late_init(void)
{
	
	return 0;
}

