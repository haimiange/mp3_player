#ifndef _WM9714_H
#define _WM9714_H

#define SNDRV_DEFAULT_IDX -1
#define BUS_ADDR       0x7f001000 /* Base address of ac97 bus*/
#define AC_GLBCTRL     0x00       /* AC97 Global Control Register */
#define AC_GLBSTAT     0x04       /* AC97 Global Status Register */
#define AC_CODEC_CMD   0x08       /* AC97 Codec Command Register */
#define AC_CODEC_STAT  0x0C       /* AC97 Codec Status Register*/
#define AC_PCMADDR     0x10 /* AC97 PCM Out/In Channel FIFO Address Register*/
#define AC_MICADDR     0x14 /* AC97 Mic In Channel FIFO Address Register*/
#define AC_PCMDATA     0x18 /* AC97 PCM Out/In Channel FIFO Data Register*/
#define AC_MICDATA     0x1C /* AC97 MIC In Channel FIFO Data Register*/
#define REG_SIZE       0x20
#define AC97_IRQ       IRQ_AC97
#define PCMOUT_CH      DMACH_AC97_PCMOUT
#define PCMIN_CH       DMACH_AC97_PCMIN
#define MICIN_CH       DMACH_AC97_MICIN
#define GPDCON         0x7f008060
#define GPECON         0x7F008080
#define AC97_GPIO      GPDCON	/* GPDCON or GPECON */
#define GPIO_SIZE      0x14

struct wm9714_chip{
    struct mutex lock;
    struct snd_card *card;
    struct snd_pcm *pcm;
    struct snd_ac97 *ac97;
    void __iomem *reg;
    unsigned long bus_addr;
    size_t reg_size;
    enum dam_ch channel;
    /* enum dma_ch dmach_pcmout; */
    /* enum dma_ch dmach_pcmin; */
    /* enum dma_ch dmach_micin; */
};

/* unsigned short s3c_ac97_read(struct wm9714_chip *chip,unsigned short reg); */
/* void s3c_ac97_write(struct wm9714_chip *chip, unsigned short reg, */
/* 		    unsigned short val); */

#endif
