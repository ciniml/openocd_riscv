/***************************************************************************
 *   Copyright (C) 2013 by Paul Fertser, fercerpav@gmail.com               *
 *                                                                         *
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *   Based on at91rm9200.c (c) Anders Larsen                               *
 *   and RPi GPIO examples by Gert van Loo & Dom                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static bb_value_t bcm2835gpio_read(void);
static int bcm2835gpio_write(int tck, int tms, int tdi);

static int bcm2835_swdio_read(void);
static void bcm2835_swdio_drive(bool is_output);
static int bcm2835gpio_swd_write(int swclk, int swdio);

static int bcm2835gpio_init(void);
static int bcm2835gpio_quit(void);

static struct bitbang_interface bcm2835gpio_bitbang = {
	.read = bcm2835gpio_read,
	.write = bcm2835gpio_write,
	.swdio_read = bcm2835_swdio_read,
	.swdio_drive = bcm2835_swdio_drive,
	.swd_write = bcm2835gpio_swd_write,
	.blink = NULL
};

// /* GPIO numbers for each signal. Negative values are invalid */
// static int tck_gpio = -1;
// static int tck_gpio_mode;
// static int tms_gpio = -1;
// static int tms_gpio_mode;
// static int tdi_gpio = -1;
// static int tdi_gpio_mode;
// static int tdo_gpio = -1;
// static int tdo_gpio_mode;
// static int trst_gpio = -1;
// static int trst_gpio_mode;
// static int srst_gpio = -1;
// static int srst_gpio_mode;
// static int swclk_gpio = -1;
// static int swclk_gpio_mode;
// static int swdio_gpio = -1;
// static int swdio_gpio_mode;
// static int swdio_dir_gpio = -1;
// static int swdio_dir_gpio_mode;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static uint32_t gpio_read_raw(void) { return *pio_base; }
static void gpio_write_raw(uint32_t v) { *pio_base = v; }
static void gpio_set_tri(uint32_t v, uint32_t m) { *(pio_base + 1) = (*(pio_base + 1) & ~m) | (v & m); }
//static uint32_t gpio2_read_raw(void) { return *(pio_base + 2); }
static void gpio2_write_raw(uint32_t v) { *(pio_base + 2) = v; }
static void gpio2_set_tri(uint32_t v, uint32_t m) { *(pio_base + 3) = (*(pio_base + 3) & ~m) | (v & m); }

static bb_value_t bcm2835gpio_read(void)
{
	return (gpio_read_raw() & (1u << 3)) ? BB_HIGH : BB_LOW;
}

static int bcm2835gpio_write(int tck, int tms, int tdi)
{
	gpio_write_raw((tck << 2) | (tms << 1) | (tdi << 0));

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int bcm2835gpio_swd_write(int swclk, int swdio)
{
	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int bcm2835gpio_reset(int trst, int srst)
{
	gpio2_set_tri(0, 1);
	gpio2_write_raw(trst || srst);

	return ERROR_OK;
}

static void bcm2835_swdio_drive(bool is_output)
{
}

static int bcm2835_swdio_read(void)
{
	return 0;
}

static int bcm2835gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int bcm2835gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int bcm2835gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionums)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tck)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tms)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tdo)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tdi)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_srst)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_trst)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionums)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionum_swclk)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionum_swdio)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_dir_gpionum_swdio)
{
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "BCM2835 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_peripheral_base)
{
	return ERROR_OK;
}

static const struct command_registration bcm2835gpio_subcommand_handlers[] = {
	{
		.name = "jtag_nums",
		.handler = &bcm2835gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "tck_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "tms_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "tdo_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "tdi_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "swd_nums",
		.handler = &bcm2835gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "swclk_num",
		.handler = &bcm2835gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "swdio_num",
		.handler = &bcm2835gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "swdio_dir_num",
		.handler = &bcm2835gpio_handle_swd_dir_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio direction control pin (set=output mode, clear=input mode)",
		.usage = "[swdio_dir]",
	},
	{
		.name = "srst_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "trst_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "speed_coeffs",
		.handler = &bcm2835gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "peripheral_base",
		.handler = &bcm2835gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration bcm2835gpio_command_handlers[] = {
	{
		.name = "bcm2835gpio",
		.mode = COMMAND_ANY,
		.help = "perform bcm2835gpio management",
		.chain = bcm2835gpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const bcm2835_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface bcm2835gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver bcm2835gpio_adapter_driver = {
	.name = "bcm2835gpio",
	.transports = bcm2835_transports,
	.commands = bcm2835gpio_command_handlers,

	.init = bcm2835gpio_init,
	.quit = bcm2835gpio_quit,
	.reset = bcm2835gpio_reset,
	.speed = bcm2835gpio_speed,
	.khz = bcm2835gpio_khz,
	.speed_div = bcm2835gpio_speed_div,

	.jtag_ops = &bcm2835gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool bcm2835gpio_jtag_mode_possible(void)
{
	return 1;
}

static bool bcm2835gpio_swd_mode_possible(void)
{
	return 0;
}

static int bcm2835gpio_init(void)
{
	bitbang_interface = &bcm2835gpio_bitbang;

	LOG_INFO("XDMA GPIO JTAG/SWD bitbang driver");

	if (transport_is_jtag() && !bcm2835gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !bcm2835gpio_swd_mode_possible()) {
		LOG_ERROR("Require swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/xdma0_user", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}
	
	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, 0);

	if (pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	// static volatile uint32_t *pads_base;
	// pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
	// 			MAP_SHARED, dev_mem_fd, BCM2835_PADS_GPIO_0_27);

	// if (pads_base == MAP_FAILED) {
	// 	LOG_ERROR("mmap: %s", strerror(errno));
	// 	close(dev_mem_fd);
	// 	return ERROR_JTAG_INIT_FAILED;
	// }

	// /* set 4mA drive strength, slew rate limited, hysteresis on */
	// pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] = 0x5a000008 + 1;

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (transport_is_jtag()) {
		// tdo_gpio_mode = MODE_GPIO(tdo_gpio);
		// tdi_gpio_mode = MODE_GPIO(tdi_gpio);
		// tck_gpio_mode = MODE_GPIO(tck_gpio);
		// tms_gpio_mode = MODE_GPIO(tms_gpio);

		bcm2835gpio_write(0, 1, 0);
		gpio_set_tri(0, 0x7);
		
		gpio2_set_tri(0, 1);
		bcm2835gpio_reset(0, 0);
	}

	// if (transport_is_swd()) {
	// 	swclk_gpio_mode = MODE_GPIO(swclk_gpio);
	// 	swdio_gpio_mode = MODE_GPIO(swdio_gpio);

	// 	GPIO_CLR = 1<<swdio_gpio | 1<<swclk_gpio;

	// 	OUT_GPIO(swclk_gpio);
	// 	OUT_GPIO(swdio_gpio);
	// }

	// if (srst_gpio != -1) {
	// 	srst_gpio_mode = MODE_GPIO(srst_gpio);
	// 	GPIO_SET = 1 << srst_gpio;
	// 	OUT_GPIO(srst_gpio);
	// }

	// if (swdio_dir_gpio != -1) {
	// 	swdio_dir_gpio_mode = MODE_GPIO(swdio_dir_gpio);
	// 	GPIO_SET = 1 << swdio_dir_gpio;
	// 	OUT_GPIO(swdio_dir_gpio);
	// }

	// LOG_DEBUG("saved pinmux settings: tck %d tms %d tdi %d "
	// 	  "tdo %d trst %d srst %d", tck_gpio_mode, tms_gpio_mode,
	// 	  tdi_gpio_mode, tdo_gpio_mode, trst_gpio_mode, srst_gpio_mode);

	return ERROR_OK;
}

static int bcm2835gpio_quit(void)
{
	return ERROR_OK;
}
