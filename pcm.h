#ifndef _PCM_H
#define _PCM_H

#include "wm9714.h"

struct s3c6410_runtime_data {
    spinlock_t lock;
    int state;
    unsigned int dma_loaded;
    unsigned int dma_limit;
    unsigned int dma_period;
    dma_addr_t dma_start;
    dma_addr_t dma_pos;
    dma_addr_t dma_end;
    struct s3c_dma_params *params;
};

struct s3c_dma_params {
	struct s3c2410_dma_client *client;	/* stream identifier */
	int channel;				/* Channel ID */
	dma_addr_t dma_addr;
	int dma_size;			/* Size of the DMA transfer */
};


int snd_wm9714_new_pcm(struct wm9714_chip *chip);

#endif
