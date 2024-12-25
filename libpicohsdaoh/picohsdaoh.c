/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 * Implementation for the Raspberry Pi RP2350 HSTX peripheral
 *
 * Copyright (c) 2024 by Steve Markgraf <steve@steve-m.de>
 *
 * based on the pico-examples/hstx/dvi_out_hstx_encoder example:
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
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

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "pico/multicore.h"

#include "data_packet.h"

#include "pico/stdlib.h"

#include "picohsdaoh.h"

// Section 5.4.2
#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1_WITH_PREAMBLE (TMDS_CTRL_11 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE (TMDS_CTRL_00 | (TMDS_CTRL_01 << 10) | (TMDS_CTRL_01 << 20))
#define VIDEO_LEADING_GUARD_BAND (0x2ccu | (0x133u << 10) | (0x2ccu << 20))

#define HSTX_CMD_RAW		(0x0u << 12)
#define HSTX_CMD_RAW_REPEAT	(0x1u << 12)
#define HSTX_CMD_TMDS		(0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT	(0x3u << 12)
#define HSTX_CMD_NOP		(0xfu << 12)

uint16_t *ring_buf = NULL;
uint16_t idle_line_buf[MODE_H_ACTIVE_PIXELS];
uint32_t info_p[64];
uint32_t info_len;

int fifo_tail = RBUF_SLICES-1;
int fifo_head = 0;

static uint32_t vblank_line_vsync_off[] = {
	HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
	SYNC_V1_H1,
	HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
	SYNC_V1_H0,
	HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
	SYNC_V1_H1,
	HSTX_CMD_NOP
};

static uint32_t vblank_line_vsync_on[] = {
	HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
	SYNC_V0_H1,
	HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
	SYNC_V0_H0,
	HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
	SYNC_V0_H1,
	HSTX_CMD_NOP
};

static uint32_t vactive_line[] = {
	HSTX_CMD_RAW_REPEAT | (MODE_H_FRONT_PORCH),
	SYNC_V1_H1,
	HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH),
	SYNC_V1_H0,
	HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH-W_PREAMBLE-W_GUARDBAND),
	SYNC_V1_H1,
	HSTX_CMD_RAW_REPEAT | W_PREAMBLE,
	SYNC_V1_H1_WITH_PREAMBLE,
	HSTX_CMD_RAW_REPEAT | W_GUARDBAND,
	VIDEO_LEADING_GUARD_BAND,
	HSTX_CMD_TMDS | MODE_H_ACTIVE_PIXELS
};

/* Pre-compute the HDMI info packet that is required to switch the MS2130 to YCbCr422 mode */
void init_info_packet(void)
{
	int len = 0;

	data_packet_t avi_info_frame;
	data_island_stream_t di_str;

	set_AVI_info_frame(&avi_info_frame, SCAN_INFO_NO_DATA, YCBCR422,
			   ITU601, PIC_ASPECT_RATIO_16_9, SAME_AS_PAR, FULL, _1920x1080P60);
	encode_data_island(&di_str, &avi_info_frame, 0, 0);

	info_p[len++] = HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH;
	info_p[len++] = SYNC_V0_H1;
	info_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_SYNC_WIDTH - W_DATA_ISLAND - W_PREAMBLE);
	info_p[len++] = SYNC_V0_H0;
	info_p[len++] = HSTX_CMD_RAW_REPEAT | W_PREAMBLE;
	info_p[len++] = SYNC_V0_H0_WITH_DATA_ISLAND_PREAMBLE;
	info_p[len++] = HSTX_CMD_RAW | W_DATA_ISLAND;

	/* convert from the two symbols per word for each channel to one
	 * symbol per word containing all three channels format */
	for (int i = 0; i < N_DATA_ISLAND_WORDS; i++) {
		info_p[len++] = (di_str.data[0][i] & 0x3ff) |
			       ((di_str.data[1][i] & 0x3ff) << 10) |
			       ((di_str.data[2][i] & 0x3ff) << 20);

		info_p[len++] = (di_str.data[0][i] >> 10) |
			       ((di_str.data[1][i] >> 10) << 10) |
			       ((di_str.data[2][i] >> 10) << 20);
	}

	info_p[len++] = HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS);
	info_p[len++] = SYNC_V0_H1;

	info_len = len;
}

void hsdaoh_update_head(int head)
{
	fifo_head = head;
}

#define DMACH_HSTX_PING 14
#define DMACH_HSTX_PONG 15
#define CRC16_INIT	0xffff

static bool hstx_dma_pong = false;
static uint v_scanline = 2;
static bool vactive_cmdlist_posted = false;
static uint16_t framecnt = 0;

uint8_t metadata[] = {
	/* 0xda7acab1 magic word for hsdaoh */
	0x1, 0xb, 0xa, 0xc, 0xa, 0x7, 0xa, 0xd,
	/* u16: placeholder for framecounter */
	0, 0, 0, 0,
	/* u8: pack_state */
	0, 0,
	/* u8: crc_config: CRC16 of last line */
	1, 0,
};

/* HSTX DMA IRQ handler, reconfigures the channel that just completed while
 * ther other channel is currently busy */
void __scratch_x("") hstx_dma_irq_handler()
{
	uint ch_num = hstx_dma_pong ? DMACH_HSTX_PONG : DMACH_HSTX_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	hstx_dma_pong = !hstx_dma_pong;

	/* for raw commands we need to use 32 bit DMA transfers */
	ch->al1_ctrl = (ch->al1_ctrl & ~DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) | (DMA_SIZE_32 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);

	if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH)) {
		/* on first line of actual VSYNC, output data packet */
		if (v_scanline == MODE_V_FRONT_PORCH) {
			dma_sniffer_disable();
			ch->read_addr = (uintptr_t)info_p;
			ch->transfer_count = info_len;

			/* update frame counter in metadata */
			metadata[8] = framecnt  & 0xf;
			metadata[9] = (framecnt >> 4) & 0xf;
			metadata[10] = (framecnt >> 8) & 0xf;
			metadata[11] = (framecnt >> 12) & 0xf;
			framecnt++;
		} else {
			ch->read_addr = (uintptr_t)vblank_line_vsync_on;
			ch->transfer_count = count_of(vblank_line_vsync_on);
		}

	} else if (v_scanline < MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH) {
		ch->read_addr = (uintptr_t)vblank_line_vsync_off;
		ch->transfer_count = count_of(vblank_line_vsync_off);
	} else if (!vactive_cmdlist_posted) {
		ch->read_addr = (uintptr_t)vactive_line;
		ch->transfer_count = count_of(vactive_line);
		vactive_cmdlist_posted = true;
	} else {
		/* Output of actual data in active video lines */
		uint16_t *next_line;
		int next_tail = (fifo_tail + 1) % RBUF_SLICES;

		if (fifo_head == next_tail) {
			/* No data to send, use idle line */
			next_line = idle_line_buf;
			next_line[RBUF_SLICE_LEN - 1] = 0;
		} else {
			next_line = &ring_buf[fifo_tail * RBUF_SLICE_LEN];
			fifo_tail = next_tail;
			next_line[RBUF_SLICE_LEN - 1] = RBUF_DATA_LEN;
		}

		uint16_t cur_active_line = v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES);

		/* fill in metadata word (last word of line) */
		if (cur_active_line < sizeof(metadata))
			next_line[RBUF_SLICE_LEN - 1] |= (metadata[cur_active_line] << 12);

		/* on the second last word of the line, insert the CRC16 of the entire previous line */
		next_line[RBUF_SLICE_LEN - 2] = dma_sniffer_get_data_accumulator() & 0xffff;

		/* (re)initialize DMA CRC sniffer */
		dma_sniffer_set_data_accumulator(CRC16_INIT);
		dma_sniffer_enable(ch_num, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);

		/* switch to 16 bit DMA transfer size for the actual data,
		 * because for YCbCr422 TMDS channel 0 is unused */
		ch->al1_ctrl = (ch->al1_ctrl & ~DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) | (DMA_SIZE_16 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
		ch->read_addr = (uintptr_t)next_line;
		ch->transfer_count = MODE_H_ACTIVE_PIXELS;
		vactive_cmdlist_posted = false;
	}

	if (!vactive_cmdlist_posted)
		v_scanline = (v_scanline + 1) % MODE_V_TOTAL_LINES;
}

void core1_entry()
{
	irq_set_exclusive_handler(DMA_IRQ_3, hstx_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_3, true);

	while (1)
		__wfi();
}

void hsdaoh_start(void)
{
	multicore_launch_core1(core1_entry);
	dma_channel_start(DMACH_HSTX_PING);
}

void hsdaoh_init(uint16_t *ringbuf)//struct hsdaoh_inst *inst, uint16_t *ringbuf)
{
	ring_buf = ringbuf;

	init_info_packet();

	/* Configure HSTX's TMDS encoder for YCbCr422 stream: L0 is unused in this
	 * mode on the MS2130 and carries the same data as L1. This way we can
	 * conveniently use 16-bit DMA transfers to transparently transfer data to
	 * libhsdaoh on the host */
	hstx_ctrl_hw->expand_tmds =
			7  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
			8 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   |
			7  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
			0  << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   |
			7  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
			0  << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;


	/* Both to-be TMDS encoded pixels and raw control words arrive as one word
	 * per symbol. While the raw control words arrive as 32-bit to carry the 3x 10
	 * bit data for the three lanes, the actual data arrives as a 16-bit word
	 * that gets duplicated before entering the HSTX */
	hstx_ctrl_hw->expand_shift =
		1 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
		0 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
		1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
		0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

	/* Serial output config: clock period of 5 cycles, pop from command
	 * expander every 5 cycles, shift the output shiftreg by 2 every cycle. */
	hstx_ctrl_hw->csr = 0;
	hstx_ctrl_hw->csr =
		HSTX_CTRL_CSR_EXPAND_EN_BITS |
		5u << HSTX_CTRL_CSR_CLKDIV_LSB |
		5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
		2u << HSTX_CTRL_CSR_SHIFT_LSB |
		HSTX_CTRL_CSR_EN_BITS;


	// HSTX outputs 0 through 7 appear on GPIO 12 through 19.
	// Pinout on Pico DVI sock:
	//
	//   GP12 D0+  GP13 D0-
	//   GP14 CK+  GP15 CK-
	//   GP16 D2+  GP17 D2-
	//   GP18 D1+  GP19 D1-

	// Assign clock pair to two neighbouring pins:
	hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
	hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
	for (uint lane = 0; lane < 3; ++lane) {
		// For each TMDS lane, assign it to the correct GPIO pair based on the
		// desired pinout:
		static const int lane_to_output_bit[3] = {0, 6, 4};
		int bit = lane_to_output_bit[lane];
		// Output even bits during first half of each HSTX cycle, and odd bits
		// during second half. The shifter advances by two bits each cycle.
		uint32_t lane_data_sel_bits =
			(lane * 10	) << HSTX_CTRL_BIT0_SEL_P_LSB |
			(lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
		// The two halves of each pair get identical data, but one pin is inverted.
		hstx_ctrl_hw->bit[bit	] = lane_data_sel_bits;
		hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
	}

	for (int i = 12; i <= 19; ++i)
		gpio_set_function(i, 0); // HSTX

	/* Both channels are set up identically, to transfer a whole scanline and
	 * then chain to the opposite channel. Each time a channel finishes, we
	 * reconfigure the one that just finished, meanwhile the opposite channel
	 * is already making progress. */
	dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_HSTX_PING);
	channel_config_set_chain_to(&c, DMACH_HSTX_PONG);
	channel_config_set_dreq(&c, DREQ_HSTX);
	channel_config_set_sniff_enable(&c, true);
	dma_channel_configure(
		DMACH_HSTX_PING,
		&c,
		&hstx_fifo_hw->fifo,
		vblank_line_vsync_off,
		count_of(vblank_line_vsync_off),
		false
	);
	c = dma_channel_get_default_config(DMACH_HSTX_PONG);
	channel_config_set_chain_to(&c, DMACH_HSTX_PING);
	channel_config_set_dreq(&c, DREQ_HSTX);
	channel_config_set_sniff_enable(&c, true);
	dma_channel_configure(
		DMACH_HSTX_PONG,
		&c,
		&hstx_fifo_hw->fifo,
		vblank_line_vsync_off,
		count_of(vblank_line_vsync_off),
		false
	);

	dma_hw->ints3 = (1u << DMACH_HSTX_PING) | (1u << DMACH_HSTX_PONG);
	dma_hw->inte3 = (1u << DMACH_HSTX_PING) | (1u << DMACH_HSTX_PONG);

	/* give the DMA the priority over the CPU on the bus */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
}
