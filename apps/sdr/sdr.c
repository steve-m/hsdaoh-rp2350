/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 * Implementation for the Raspberry Pi RP2350 HSTX peripheral
 *
 * External dual 10-bit ADC example, connected to the PIO
 * used for hsdaohSDR
 *
 * Copyright (c) 2024-2025 by Steve Markgraf <steve@steve-m.de>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/pll.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

#include <stdio.h>

#include "picohsdaoh.h"
#include "adc_16bit_input.pio.h"
#include "adc_20bit_input.pio.h"

#define I2C_TIMEOUT 50000	/* 50 ms */

/* The PIO is running with sys_clk/1, and needs 4 cycles per sample,
 * so the ADC clock is sys_clk/4 */
#define SYS_CLK_8BIT		328000	// 82 MHz ADC clock
#define SYS_CLK_10BIT		264000	// 66 MHz ADC clock
#define HSTX_CLK_MHZ		432

// If we want to push everything to the limits:
#define SYS_CLK_MAX_SRATE	336000 // 84 MHz ADC clock @ 8 bit IQ
#define HSTX_CLK_MAX_SRATE	440

// For alignment of 5x16 bit words in the payload, so that every line starts with word 0
#define ADC_DATA_LEN_10_BIT	(RBUF_SLICE_LEN - 5)
#define ADC_DATA_LEN_8_BIT	(RBUF_SLICE_LEN - 3)

#define PIO_INPUT_PIN_BASE 27
#define PIO_OUTPUT_CLK_PIN 26

#define DMACH_PIO_PING 0
#define DMACH_PIO_PONG 1

bool enable_8bit_mode = false;

static bool pio_dma_pong = false;
uint16_t ringbuffer[RBUF_DEFAULT_TOTAL_LEN];
int ringbuf_head = 2;

void __scratch_y("") pio_dma_irq_handler()
{
	uint ch_num = pio_dma_pong ? DMACH_PIO_PONG : DMACH_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	pio_dma_pong = !pio_dma_pong;

	ringbuf_head = (ringbuf_head + 1) % RBUF_DEFAULT_SLICES;

	ch->write_addr = (uintptr_t)&ringbuffer[ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = enable_8bit_mode ? ADC_DATA_LEN_8_BIT : ADC_DATA_LEN_10_BIT;

	hsdaoh_update_head(0, ringbuf_head);
}

uint offset;
uint offset2;
uint sm_data;
PIO pio = pio0;

void init_pio(void)
{
	/* move up GPIO base of PIO to access all ADC pins */
	pio_set_gpio_base(pio, 16);

	offset = pio_add_program(pio, &adc_16bit_input_program);
	offset2 = pio_add_program(pio, &adc_20bit_input_program);

	sm_data = pio_claim_unused_sm(pio, true);
}

void __no_inline_not_in_flash_func(init_pio_dma)(void)
{
	ringbuf_head = 2;
	pio_dma_pong = false;

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_PIO_PING,
		&c,
		&ringbuffer[0 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		enable_8bit_mode ? ADC_DATA_LEN_8_BIT : ADC_DATA_LEN_10_BIT,
		false
	);
	c = dma_channel_get_default_config(DMACH_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_PIO_PONG,
		&c,
		&ringbuffer[1 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		enable_8bit_mode ? ADC_DATA_LEN_8_BIT : ADC_DATA_LEN_10_BIT,
		false
	);

	dma_hw->ints0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	dma_hw->inte0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	irq_set_exclusive_handler(DMA_IRQ_0, pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_start(DMACH_PIO_PING);
}

void __no_inline_not_in_flash_func(init_pio_input)(void)
{
//	irq_set_enabled(DMA_IRQ_0, false);
//	dma_channel_start(DMACH_PIO_PING);
//	dma_channel_start(DMACH_PIO_PONG);
//	pio_sm_set_enabled(pio, sm_data, false);

	set_sys_clock_khz(enable_8bit_mode ? SYS_CLK_8BIT : SYS_CLK_10BIT, true);

	init_pio_dma();

	if (enable_8bit_mode) {
		hsdaoh_add_stream(0, PIO_8BIT_IQ, (SYS_CLK_8BIT/4) * 1000, ADC_DATA_LEN_8_BIT, RBUF_DEFAULT_SLICES, ringbuffer);
		adc_16bit_input_program_init(pio, sm_data, offset, PIO_INPUT_PIN_BASE, PIO_OUTPUT_CLK_PIN);

	} else {
		hsdaoh_add_stream(0, PIO_10BIT_IQ, (SYS_CLK_10BIT/4) * 1000, ADC_DATA_LEN_10_BIT, RBUF_DEFAULT_SLICES, ringbuffer);
		adc_20bit_input_program_init(pio, sm_data, offset2, PIO_INPUT_PIN_BASE, PIO_OUTPUT_CLK_PIN);
	}
}

void __no_inline_not_in_flash_func(handle_tuner_interface)(void)
{
	/* handle tty -> I2C interface for I2C tuner access */
	uint8_t i2c_addr, reg, val;
	int ret;

	while (1) {
		char c = getchar();

		switch (c) {
		case 'a':
			hsdaoh_remove_stream(0);
			pio_sm_set_enabled(pio, sm_data, false);
			irq_set_enabled(DMA_IRQ_0, false);
			dma_channel_abort(DMACH_PIO_PING);
			dma_channel_abort(DMACH_PIO_PONG);
			pio_sm_clear_fifos(pio, sm_data);
			enable_8bit_mode = !enable_8bit_mode;
			init_pio_input();
			break;
		case 'r':
			i2c_addr = getchar();
			reg = getchar();

			// special handling for RT7x0 read
			if (i2c_addr == 0x7a) {
				uint8_t regnull = 0;
				uint8_t bytes_to_read = reg + 1;
				uint8_t response[256];
				ret = i2c_write_timeout_us(i2c_default, i2c_addr, &regnull, 1, false, I2C_TIMEOUT);
				if (ret == 1) {
					ret = i2c_read_timeout_us(i2c_default, i2c_addr, response, bytes_to_read, false, I2C_TIMEOUT);
					if (ret == bytes_to_read)
						ret = 1;

					val = response[reg];
				}

			} else {
				ret = i2c_write_timeout_us(i2c_default, i2c_addr, &reg, 1, false, I2C_TIMEOUT);
				if (ret == 1)
					ret = i2c_read_timeout_us(i2c_default, i2c_addr, &val, 1, false, I2C_TIMEOUT);
			}

			putchar_raw((ret == 1) ? 1 : 0); // I2C ACK
			putchar_raw(val);

			break;
		case 'w':
			i2c_addr = getchar();
			val = getchar();
			reg = getchar();
			uint8_t wr[2] = { reg, val };
			ret = i2c_write_timeout_us(i2c_default, i2c_addr, wr, sizeof(wr), false, I2C_TIMEOUT);
			putchar_raw((ret == sizeof(wr)) ? 1 : 0);
			break;

		default:
			break;
		}

	}
}

#define OVERVOLT 1

int main()
{
#ifdef OVERVOLT
	/* set maximum 'allowed' voltage without voiding warranty */
	//vreg_set_voltage(VREG_VOLTAGE_MAX);
	vreg_disable_voltage_limit();
	vreg_set_voltage(VREG_VOLTAGE_1_65);
	sleep_us(SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US);
#endif
	hsdaoh_set_sys_clock_khz(enable_8bit_mode ? SYS_CLK_8BIT : SYS_CLK_10BIT);
	int usbdiv = HSTX_CLK_MHZ / 48;

#ifdef USE_SYS_CLK_FOR_USB
	/* set USB clock to clk_sys/n */
	usbdiv = SYS_CLK/48000;
	hw_write_masked(&clocks_hw->clk[clk_usb].ctrl,
			CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_USB_CTRL_AUXSRC_LSB,
			CLOCKS_CLK_USB_CTRL_AUXSRC_BITS);
#endif

	hw_write_masked(&clocks_hw->clk[clk_usb].div,
			usbdiv << CLOCKS_CLK_USB_DIV_INT_LSB,
			CLOCKS_CLK_USB_DIV_INT_BITS);

	/* Initialize USB PLL for HSTX clock */
	pll_init(pll_usb, 1, HSTX_CLK_MHZ * 4 * MHZ, 2, 2);

	/* set HSTX divider to 1 */
	hw_write_masked(
		&clocks_hw->clk[clk_hstx].div,
		1 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
		CLOCKS_CLK_HSTX_DIV_INT_BITS
	);

	/* set HSTX clock source to PLL_USB */
	hw_write_masked(&clocks_hw->clk[clk_hstx].ctrl,
			CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_HSTX_CTRL_AUXSRC_LSB,
			CLOCKS_CLK_HSTX_CTRL_AUXSRC_BITS);

	stdio_init_all();

	hsdaoh_init(GPIO_DRIVE_STRENGTH_12MA, GPIO_SLEW_RATE_FAST);
	hsdaoh_start();

	init_pio();
	init_pio_input();

	i2c_init(i2c_default, 100 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	handle_tuner_interface();
}
