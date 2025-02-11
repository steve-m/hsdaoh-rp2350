/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 * Implementation for the Raspberry Pi RP2350 HSTX peripheral
 *
 * Dual External 12-bit ADC example, connected to the PIO
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

#include "picohsdaoh.h"
#include "adc_24bit_input.pio.h"
#include "pcm1802_fmt00.pio.h"

#if (PICO_PIO_USE_GPIO_BASE != 1)
#warning "PICO_PIO_USE_GPIO_BASE is not set to 1, this application will not work correctly!"
#endif

/* The PIO is running with sys_clk/2, and needs 4 cycles per sample,
 * so the ADC clock is sys_clk/8 */
#define SYS_CLK		320000	// 40 MHz ADC clock

// For alignment of 3x32 bit words in the payload, so that every line starts with word 0
#define ADC_DATA_LEN	(RBUF_SLICE_LEN - 6)

// Same here for 2x32 bit words
#define AUDIO_DATA_LEN	(RBUF_SLICE_LEN - 4)

// ADC is attached to GP0 - GP11 with clock on GP20
#define PIO_INPUT_PIN_BASE 23
#define PIO_OUTPUT_CLK_PIN 20

#define DMACH_PIO_PING 0
#define DMACH_PIO_PONG 1

static bool pio_dma_pong = false;
uint16_t ringbuffer[RBUF_TOTAL_LEN];
int ringbuf_head = 2;

void __scratch_y("") pio_dma_irq_handler()
{
	uint ch_num = pio_dma_pong ? DMACH_PIO_PONG : DMACH_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	pio_dma_pong = !pio_dma_pong;

	ringbuf_head = (ringbuf_head + 1) % RBUF_SLICES;

	ch->write_addr = (uintptr_t)&ringbuffer[ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = ADC_DATA_LEN/2;

	hsdaoh_update_head(0, ringbuf_head);
}

void init_pio_input(void)
{
	PIO pio = pio0;

	/* move up GPIO base of PIO to access all ADC pins */
	pio_set_gpio_base(pio, 16);

	uint offset = pio_add_program(pio, &adc_24bit_input_program);
	uint sm_data = pio_claim_unused_sm(pio, true);

	adc_24bit_input_program_init(pio, sm_data, offset, PIO_INPUT_PIN_BASE, PIO_OUTPUT_CLK_PIN);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_PIO_PING,
		&c,
		&ringbuffer[0 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		ADC_DATA_LEN/2,
		false
	);
	c = dma_channel_get_default_config(DMACH_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_PIO_PONG,
		&c,
		&ringbuffer[1 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		ADC_DATA_LEN/2,
		false
	);

	dma_hw->ints0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	dma_hw->inte0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	irq_set_exclusive_handler(DMA_IRQ_0, pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_start(DMACH_PIO_PING);
}

#define PCM1802_DATA_PIN	0

#define DMACH_AUDIO_PIO_PING 2
#define DMACH_AUDIO_PIO_PONG 3

static bool audio_pio_dma_pong = false;
uint16_t audio_ringbuffer[RBUF_TOTAL_LEN];
int audio_ringbuf_head = 2;

void __scratch_y("") audio_pio_dma_irq_handler()
{
	uint ch_num = audio_pio_dma_pong ? DMACH_AUDIO_PIO_PONG : DMACH_AUDIO_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	audio_pio_dma_pong = !audio_pio_dma_pong;

	audio_ringbuf_head = (audio_ringbuf_head + 1) % RBUF_SLICES;

	ch->write_addr = (uintptr_t)&audio_ringbuffer[audio_ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = AUDIO_DATA_LEN/2;

	hsdaoh_update_head(1, audio_ringbuf_head);
}

void init_audio_pio_input(void)
{
	PIO pio = pio1;

	pio_set_gpio_base(pio, 0);

	uint offset = pio_add_program(pio, &pcm1802_fmt00_program);
	uint sm_data = pio_claim_unused_sm(pio, true);
	pcm1802_fmt00_program_init(pio, sm_data, offset, PCM1802_DATA_PIN);

	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_AUDIO_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_AUDIO_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_AUDIO_PIO_PING,
		&c,
		&audio_ringbuffer[0 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN/2,
		false
	);
	c = dma_channel_get_default_config(DMACH_AUDIO_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_AUDIO_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		DMACH_AUDIO_PIO_PONG,
		&c,
		&audio_ringbuffer[1 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN/2,
		false
	);

	dma_hw->ints1 |= (1u << DMACH_AUDIO_PIO_PING) | (1u << DMACH_AUDIO_PIO_PONG);
	dma_hw->inte1 |= (1u << DMACH_AUDIO_PIO_PING) | (1u << DMACH_AUDIO_PIO_PONG);
	irq_set_exclusive_handler(DMA_IRQ_1, audio_pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);

	dma_channel_start(DMACH_AUDIO_PIO_PING);
}

#define OVERVOLT 1

int main()
{
#ifdef OVERVOLT
	/* set maximum 'allowed' voltage without voiding warranty */
	vreg_set_voltage(VREG_VOLTAGE_MAX);
	sleep_ms(1);
#endif

	set_sys_clock_khz(SYS_CLK, true);

	/* set HSTX clock to sysclk/1 */
	hw_write_masked(
		&clocks_hw->clk[clk_hstx].div,
		1 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
		CLOCKS_CLK_HSTX_DIV_INT_BITS
	);

	pll_init(pll_usb, 1, 1536 * MHZ, 4, 2);
//	pll_init(pll_usb, 1, (983 * MHZ) + (40 * KHZ), 4, 2);

	/* set USB clock to clk_usb/4 */
	hw_write_masked(&clocks_hw->clk[clk_usb].div, 4 << CLOCKS_CLK_USB_DIV_INT_LSB, CLOCKS_CLK_USB_DIV_INT_BITS);

	/* set GPOUT0 clock to USB PLL/10 -> 19.2 MHz, resulting in 75 kHz ADC sample rate (19.2M/256) */
	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 10);

	stdio_init_all();

	hsdaoh_init(GPIO_DRIVE_STRENGTH_4MA, GPIO_SLEW_RATE_SLOW);
	hsdaoh_add_stream(0, 1, (SYS_CLK/8) * 1000, ADC_DATA_LEN, ringbuffer);
	hsdaoh_add_stream(1, 1, 75000, AUDIO_DATA_LEN, audio_ringbuffer);
	hsdaoh_start();
	init_pio_input();
	init_audio_pio_input();

	while (1)
		__wfi();
}
