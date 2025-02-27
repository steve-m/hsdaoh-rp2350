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

// stream ID, CRC word and length/metadata word, so 3 reserved words
#define NUM_RESERVED_WORDS	3
#define RBUF_DEFAULT_SLICES	16
#define RBUF_SLICE_LEN		MODE_H_ACTIVE_PIXELS
#define RBUF_MAX_DATA_LEN	(RBUF_SLICE_LEN - NUM_RESERVED_WORDS)
#define RBUF_DEFAULT_TOTAL_LEN	(RBUF_SLICE_LEN * RBUF_DEFAULT_SLICES)

#define MAX_STREAMS		4

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

#define FLAG_STREAM_ID_PRESENT	(1 << 0)
#define FLAG_FORMAT_ID_PRESENT	(1 << 1)

enum
{
	RAW_8BIT,
	RAW_16BIT,
	RAW_24BIT,
	RAW_32BIT,
	RAW_64BIT,
	PIO_1BIT,
	PIO_2BIT,
	PIO_3BIT,
	PIO_4BIT,
	PIO_5BIT,
	PIO_6BIT,
	PIO_7BIT,
	PIO_8BIT,
	PIO_8BIT_DUAL,
	PIO_8BIT_IQ,
	PIO_9BIT,
	PIO_10BIT,
	PIO_10BIT_DUAL,
	PIO_10BIT_IQ,
	PIO_11BIT,
	PIO_12BIT,
	PIO_12BIT_DUAL,
	PIO_12BIT_IQ,
	PIO_13BIT,
	PIO_14BIT,
	PIO_14BIT_DUAL,
	PIO_14BIT_IQ,
	PIO_15BIT,
	PIO_16BIT,
	PIO_16BIT_DUAL,
	PIO_16BIT_IQ,
	PIO_17BIT,
	PIO_18BIT,
	PIO_19BIT,
	PIO_20BIT,
	PIO_24BIT,
	PIO_24BIT_IQ,
	PIO_28BIT,
	PIO_32BIT,
	PIO_32BIT_IQ,
	PIO_PCM1802_AUDIO,
};

void hsdaoh_start(void);
void hsdaoh_init(int dstrength, int slewrate);
void hsdaoh_update_head(int stream_id, int head);
int hsdaoh_add_stream(uint16_t stream_id, uint16_t format, uint32_t samplerate, uint length, uint slices, uint16_t *ringbuf);

#endif
