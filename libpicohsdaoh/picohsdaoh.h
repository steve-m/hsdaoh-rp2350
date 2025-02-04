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
#define NUM_RESERVED_WORDS	2
#define RBUF_SLICES		16
#define RBUF_SLICE_LEN		MODE_H_ACTIVE_PIXELS
#define RBUF_MAX_DATA_LEN	(RBUF_SLICE_LEN - NUM_RESERVED_WORDS)
#define RBUF_TOTAL_LEN		(RBUF_SLICE_LEN * RBUF_SLICES)

#define MAX_STREAMS		8

enum crc_config {
	CRC_NONE,		/* No CRC, just 16 bit idle counter */
	CRC16_1_LINE,		/* Line contains CRC of the last line */
	CRC16_2_LINE		/* Line contains CRC of the line before the last line */
};

typedef struct
{
	uint32_t magic;
	uint16_t framecounter;
	uint8_t  reserved1;
	uint8_t  crc_config;
	uint16_t version;
	uint32_t flags;
	uint8_t  reserved2[116];
	uint8_t  stream_cnt;
} __attribute__((packed, aligned(1))) metadata_t;

void hsdaoh_start(void);
void hsdaoh_init(void);
void hsdaoh_update_head(int stream_id, int head);
int hsdaoh_add_stream(uint16_t stream_id, uint16_t format, uint32_t samplerate, uint length, uint16_t *ringbuf);

#endif
