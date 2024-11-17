#ifndef _PICOHSDAOH_H
#define _PICOHSDAOH_H

// trimmed to absolute minimum timings the MS2130 accepts
#define MODE_H_FRONT_PORCH	6
#define MODE_H_SYNC_WIDTH	45
#define MODE_H_BACK_PORCH	11
#define MODE_H_ACTIVE_PIXELS	1920

#define MODE_V_FRONT_PORCH	1
#define MODE_V_SYNC_WIDTH	2
#define MODE_V_BACK_PORCH	1
#define MODE_V_ACTIVE_LINES	1080

#define MODE_H_TOTAL_PIXELS ( \
	MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
	MODE_H_BACK_PORCH  + MODE_H_ACTIVE_PIXELS \
)
#define MODE_V_TOTAL_LINES  ( \
	MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
	MODE_V_BACK_PORCH  + MODE_V_ACTIVE_LINES \
)

// CRC word and length/metadata word, so 2 reserved words
// padded to 3 so that for the 12bit/16bit packed format
// each line starts with word 0
#define NUM_RESERVED_WORDS	3
#define RBUF_SLICES		16
#define RBUF_SLICE_LEN		MODE_H_ACTIVE_PIXELS
#define RBUF_DATA_LEN		(RBUF_SLICE_LEN - NUM_RESERVED_WORDS)
#define RBUF_TOTAL_LEN		(RBUF_SLICE_LEN * RBUF_SLICES)

void hsdaoh_start(void);
void hsdaoh_init(uint16_t *ringbuf);
void hsdaoh_update_head(int head);

#endif
