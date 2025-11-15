/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 * Implementation for the Raspberry Pi RP2350 HSTX peripheral
 *
 * Internal ADC example, overclocked to 3,33 MHz sample rate
 * When using the USB PLL for the ADC, almost 8 MHz sample rate
 * can be achieved!
 *
 * Copyright (c) 2024 by Steve Markgraf <steve@steve-m.de>
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
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pll.h"

#include "picohsdaoh.h"

#define SYS_CLK		320000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 1

#define DMACH_ADC_PING 0
#define DMACH_ADC_PONG 1

static bool dma_adc_pong = false;
uint16_t ringbuffer[RBUF_DEFAULT_TOTAL_LEN];
int ringbuf_head = 2;

void __scratch_y("") adc_dma_irq_handler()
{
	uint ch_num = dma_adc_pong ? DMACH_ADC_PONG : DMACH_ADC_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	dma_adc_pong = !dma_adc_pong;

	ringbuf_head = (ringbuf_head + 1) % RBUF_DEFAULT_SLICES;

	ch->write_addr = (uintptr_t)&ringbuffer[ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = RBUF_MAX_DATA_LEN;

	hsdaoh_update_head(0, ringbuf_head);
}

void init_adc_input(void)
{
	adc_init();
	adc_select_input(CAPTURE_CHANNEL);

	adc_fifo_setup(
		true,	// Write each completed conversion to the sample FIFO
		true,	// Enable DMA data request (DREQ)
		1,	// DREQ (and IRQ) asserted when at least 1 sample present
		false,	// Disable the ERR bit
		false	// No shift of samples, use full 12 bit resolution
	);

	adc_set_clkdiv(0);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_ADC_PING);
	channel_config_set_chain_to(&c, DMACH_ADC_PONG);
	channel_config_set_dreq(&c, DREQ_ADC);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_ADC_PING,
		&c,
		&ringbuffer[0 * RBUF_SLICE_LEN],
		&adc_hw->fifo,
		RBUF_MAX_DATA_LEN,
		false
	);
	c = dma_channel_get_default_config(DMACH_ADC_PONG);
	channel_config_set_chain_to(&c, DMACH_ADC_PING);
	channel_config_set_dreq(&c, DREQ_ADC);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_ADC_PONG,
		&c,
		&ringbuffer[1 * RBUF_SLICE_LEN],
		&adc_hw->fifo,
		RBUF_MAX_DATA_LEN,
		false
	);

	dma_hw->ints0 |= (1u << DMACH_ADC_PING) | (1u << DMACH_ADC_PONG);
	dma_hw->inte0 |= (1u << DMACH_ADC_PING) | (1u << DMACH_ADC_PONG);
	irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_start(DMACH_ADC_PING);
	adc_run(true);
}

int main()
{
	/* set maximum 'allowed' voltage without voiding warranty */
	vreg_set_voltage(VREG_VOLTAGE_MAX);
	sleep_us(SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US);

	hsdaoh_set_sys_clock_khz(SYS_CLK);

	/* set HSTX clock to sysclk/3 */
	hw_write_masked(
		&clocks_hw->clk[clk_hstx].div,
		3 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
		CLOCKS_CLK_HSTX_DIV_INT_BITS
	);

	/* switch ADC clock source to sys_clk */
	hw_write_masked(
		&clocks_hw->clk[clk_adc].ctrl,
		CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB,
		CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS
	);

	stdio_init_all();

	hsdaoh_init(GPIO_DRIVE_STRENGTH_4MA, GPIO_SLEW_RATE_SLOW);
	hsdaoh_add_stream(0, 1, (SYS_CLK/8) * 1000, RBUF_MAX_DATA_LEN, RBUF_DEFAULT_SLICES, ringbuffer);
	hsdaoh_start();
	init_adc_input();

	while (1)
		__wfi();
}
