/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * based on SolidRun mx6 
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __ATR200B_CONFIG_H
#define __ATR200B_CONFIG_H

#include <config_distro_defaults.h>
#include "mx6_common.h"
#define CONFIG_PWM_IMX
#define CONFIG_IMX_THERMAL

#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)
#define CONFIG_MXC_UART
#define CONFIG_IMX6_PWM_PER_CLK  66000000

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* Ethernet Configuration */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS


/* USB */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_PREBOOT \
		"setenv stdin  serial; " \
		"setenv stdout serial; " \
		"setenv stderr serial; " \
/* Command definition */

#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV	"ttymxc0"
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC2 */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_EXTRA_ENV_SETTINGS "" 

#define CONFIG_BOOTCOMMAND "if test -x $kernelsize; "\
                    "then sf probe; sf read 0x12000000 0xc0000 $kernelsize; " \
                    "sf read 0x18000000 0x80000 $dtbsize;" \
                    "bootz 0x12000000 - 0x18000000 ;" \
                    "else saveenv ; bootz 0x12000000 - 0x18000000 ;fi" 

#define BOOT_TARGET_DEVICES(func) 


/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define CONFIG_SYS_SDRAM_BASE          MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			0x40000 /* 256k */

#define CONFIG_ENV_OFFSET		0x40000	/*2nd sector */
#define CONFIG_ENV_SECT_SIZE		0x40000	/* 256k */

/* SPI */

#define CONFIG_CMD_SF
#define CONFIG_MXC_SPI
#define CONFIG_SYS_MAX_FLASH_BANKS_DETECT
#undef	CONFIG_CMD_IMLS
#define CONFIG_SPI_FLASH_BAR
#define CONFIG_SF_DEFAULT_BUS        2
#define CONFIG_SF_DEFAULT_SPEED	40000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)

#endif                         /* __ATR200B_CONFIG_H */
