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
#define AUDIO_DATA_LEN		(RBUF_SLICE_LEN - 4)
#define AUDIO_RBUF_SLICES	8

// ADC is attached to GP0 - GP11 with clock on GP20
#define PIO_INPUT_PIN_BASE 23
#define PIO_OUTPUT_CLK_PIN 20

#define DMACH_PIO_PING 0
#define DMACH_PIO_PONG 1

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

#define PCM1802_DATA_PIN_1	3
#define AUDIO1_STREAM_ID	2
#define DMACH_AUDIO1_PIO_PING	2
#define DMACH_AUDIO1_PIO_PONG	3

#define PCM1802_DATA_PIN_2	0
#define AUDIO2_STREAM_ID	3
#define DMACH_AUDIO2_PIO_PING	4
#define DMACH_AUDIO2_PIO_PONG	5

static bool audio1_pio_dma_pong = false;
uint16_t audio1_ringbuffer[AUDIO_RBUF_SLICES * RBUF_SLICE_LEN];
int audio1_ringbuf_head = 2;

static bool audio2_pio_dma_pong = false;
uint16_t audio2_ringbuffer[AUDIO_RBUF_SLICES * RBUF_SLICE_LEN];
int audio2_ringbuf_head = 2;

void __scratch_y("") audio1_pio_dma_irq_handler()
{
	uint ch_num = audio1_pio_dma_pong ? DMACH_AUDIO1_PIO_PONG : DMACH_AUDIO1_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	audio1_pio_dma_pong = !audio1_pio_dma_pong;

	audio1_ringbuf_head = (audio1_ringbuf_head + 1) % AUDIO_RBUF_SLICES;

	ch->write_addr = (uintptr_t)&audio1_ringbuffer[audio1_ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = AUDIO_DATA_LEN/2;

	hsdaoh_update_head(AUDIO1_STREAM_ID, audio1_ringbuf_head);
}

void __scratch_y("") audio2_pio_dma_irq_handler()
{
	uint ch_num = audio2_pio_dma_pong ? DMACH_AUDIO2_PIO_PONG : DMACH_AUDIO2_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	audio2_pio_dma_pong = !audio2_pio_dma_pong;

	audio2_ringbuf_head = (audio2_ringbuf_head + 1) % AUDIO_RBUF_SLICES;

	ch->write_addr = (uintptr_t)&audio2_ringbuffer[audio2_ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = AUDIO_DATA_LEN/2;

	hsdaoh_update_head(AUDIO2_STREAM_ID, audio2_ringbuf_head);
}

void init_audio_pio_dma(PIO pio, uint sm_data, uint dmach_ping, uint dmach_pong, uint16_t *rbuf)
{
	dma_channel_config c;
	c = dma_channel_get_default_config(dmach_ping);
	channel_config_set_chain_to(&c, dmach_pong);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		dmach_ping,
		&c,
		&rbuf[0 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN/2,
		false
	);
	c = dma_channel_get_default_config(dmach_pong);
	channel_config_set_chain_to(&c, dmach_ping);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(
		dmach_pong,
		&c,
		&rbuf[1 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		AUDIO_DATA_LEN/2,
		false
	);

	uint dma_irq = (dmach_ping == DMACH_AUDIO1_PIO_PING) ? DMA_IRQ_1 : DMA_IRQ_2;
	if (dma_irq == DMA_IRQ_1) {
		dma_hw->ints1 |= (1u << dmach_ping) | (1u << dmach_pong);
		dma_hw->inte1 |= (1u << dmach_ping) | (1u << dmach_pong);
		irq_set_exclusive_handler(dma_irq, audio1_pio_dma_irq_handler);
	} else {
		dma_hw->ints2 |= (1u << dmach_ping) | (1u << dmach_pong);
		dma_hw->inte2 |= (1u << dmach_ping) | (1u << dmach_pong);
		irq_set_exclusive_handler(dma_irq, audio2_pio_dma_irq_handler);
	}
	irq_set_enabled(dma_irq, true);

	dma_channel_start(dmach_ping);
}

void init_audio_pio_input(void)
{
	PIO pio = pio1;
	pio_set_gpio_base(pio, 0);
	uint offset = pio_add_program(pio, &pcm1802_fmt00_program);

	uint sm_data = pio_claim_unused_sm(pio, true);
	pcm1802_fmt00_program_init(pio, sm_data, offset, PCM1802_DATA_PIN_1);
	init_audio_pio_dma(pio, sm_data, DMACH_AUDIO1_PIO_PING, DMACH_AUDIO1_PIO_PONG, audio1_ringbuffer);

	sm_data = pio_claim_unused_sm(pio, true);
	pcm1802_fmt00_program_init(pio, sm_data, offset, PCM1802_DATA_PIN_2);
	init_audio_pio_dma(pio, sm_data, DMACH_AUDIO2_PIO_PING, DMACH_AUDIO2_PIO_PONG, audio2_ringbuffer);
}

#define OVERVOLT 1

int main()
{
#ifdef OVERVOLT
	/* set maximum 'allowed' voltage without voiding warranty */
	vreg_set_voltage(VREG_VOLTAGE_MAX);
	sleep_ms(1);
#endif

	hsdaoh_set_sys_clock_khz(SYS_CLK);

	/* set HSTX clock to sysclk/1 */
	hw_write_masked(
		&clocks_hw->clk[clk_hstx].div,
		1 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
		CLOCKS_CLK_HSTX_DIV_INT_BITS
	);

#ifdef GPOUT_AUDIO_CLOCK
	pll_init(pll_usb, 1, 1536 * MHZ, 4, 2);

	/* set USB clock to clk_usb/4 */
	hw_write_masked(&clocks_hw->clk[clk_usb].div, 4 << CLOCKS_CLK_USB_DIV_INT_LSB, CLOCKS_CLK_USB_DIV_INT_BITS);

	/* set GPOUT0 clock to USB PLL/10 -> 19.2 MHz, resulting in 75 kHz ADC sample rate (19.2M/256) */
	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 10);
#endif

	stdio_init_all();
	hsdaoh_init(GPIO_DRIVE_STRENGTH_12MA, GPIO_SLEW_RATE_FAST);
	hsdaoh_add_stream(0, PIO_12BIT_DUAL, (SYS_CLK/8) * 1000, ADC_DATA_LEN, RBUF_DEFAULT_SLICES, ringbuffer);
	hsdaoh_add_stream(AUDIO1_STREAM_ID, PIO_PCM1802_AUDIO, 78125, AUDIO_DATA_LEN, AUDIO_RBUF_SLICES, audio1_ringbuffer);
	hsdaoh_add_stream(AUDIO2_STREAM_ID, PIO_PCM1802_AUDIO, 78125, AUDIO_DATA_LEN, AUDIO_RBUF_SLICES, audio2_ringbuffer);
	hsdaoh_start();
	init_pio_input();
	init_audio_pio_input();

	/* synchronously start data input */
	pio_set_sm_mask_enabled(pio1, 3, true);
	pio_set_sm_mask_enabled(pio0, 1, true);

	while (1)
		__wfi();
}
