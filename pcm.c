#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/ac97_codec.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <plat/audio.h>

#include "wm9714.h"
#include "pcm.h"

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

static struct s3c2410_dma_client pcm_out_client = {
    .name = "wm9714-pcm-out",
};
static struct s3c2410_dma_client mic_in_client = {
    .name = "wm9714-mic-in",
};

static struct snd_pcm_hardware snd_wm9714_playback_hw = {
    .info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
	     SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000_48000,
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 2,
    .channels_max = 2,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 1,
    .periods_max = 1024,
};

static struct snd_pcm_hardware snd_wm9714_capture_hw = {
    .info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
	     SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000_48000,
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 2,
    .channels_max = 2,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 1,
    .periods_max = 1024,
};

static int snd_wm9714_playback_open(struct snd_pcm_substream*substream)
{
    struct wm9714_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd;
    
    runtime->hw = snd_wm9714_playback_hw;
    prtd = kzalloc(sizeof(struct s3c6410_runtime_data), GFP_KERNEL);
    if (prtd == NULL){
	printk(KERN_ERR "%s: No memory for runtime_data\n",__FUNCTION__);
	return -ENOMEM;
    }
    prtd->params = kzalloc(sizeof(struct s3c_dma_params),GFP_KERNEL);
    if (prtd->params == NULL){
	printk(KERN_ERR "%s: No memory for dma_params\n",__FUNCTION__);
	kfree(prtd);
	return -ENOMEM;
    }
    spin_lock_init(&prtd->lock);
    prtd->params->channel = chip->dmach_pcmout;
    prtd->params->client = &pcm_out_client;
    prtd->params->dma_addr = chip->bus_addr + AC_PCMDATA;
    prtd->params->dma_size = 4;
	
    runtime->private_data = prtd;
    return 0;
}

static int snd_wm9714_capture_open(struct snd_pcm_substream*substream)
{
    struct wm9714_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd;
    
    runtime->hw = snd_wm9714_capture_hw;
    prtd = kzalloc(sizeof(struct s3c6410_runtime_data), GFP_KERNEL);
    if (prtd == NULL){
	printk(KERN_ERR "%s: No memory for runtime_data\n",__FUNCTION__);
	return -ENOMEM;
    }
    prtd->params = kzalloc(sizeof(struct s3c_dma_params),GFP_KERNEL);
    if (prtd->params == NULL){
	printk(KERN_ERR "%s: No memory for dma_params\n",__FUNCTION__);
	kfree(prtd);
	return -ENOMEM;
    }
    spin_lock_init(&prtd->lock);
    prtd->params->channel = DMACH_AC97_MICIN;
    prtd->params->client = &mic_in_client;
    prtd->params->dma_addr = chip->bus_addr + AC_MICDATA;
    prtd->params->dma_size = 4;
	
    runtime->private_data = prtd;
    return 0;
}

static int snd_wm9714_playback_close(struct snd_pcm_substream*substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd = runtime->private_data;
      
    kfree(prtd->params);
    kfree(prtd);

    return 0;
}

static int snd_wm9714_capture_close(struct snd_pcm_substream*substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd = runtime->private_data;
    
    kfree(prtd->params);
    kfree(prtd);

    return 0;
}

static int snd_wm9714_pcm_hw_params(struct snd_pcm_substream* substream,
				    struct snd_pcm_hw_params *hw_params)
{
    struct wm9714_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_ac97_bus *ac97_bus = chip->ac97->bus;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd = runtime->private_data;
    unsigned long totbytes = params_buffer_bytes(hw_params);
    u16 reg;
    int ret = 0;

    pr_debug("Entered %s\n", __FUNCTION__);   
    snd_pcm_lib_malloc_pages(substream,totbytes); 
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        ret = s3c2410_dma_request(prtd->params->channel,&pcm_out_client,NULL);
    } else {
        ret = s3c2410_dma_request(prtd->params->channel,&mic_in_client,NULL);
    }
    if (ret < 0) {
	printk(KERN_ERR "failed to get dma channel\n");
	return ret;
    }
    spin_lock_irq(&prtd->lock);
    prtd->dma_loaded = 0;
    prtd->dma_limit = runtime->hw.periods_min;
    prtd->dma_period = params_period_bytes(hw_params);
    prtd->dma_start = runtime->dma_addr;
    prtd->dma_pos = prtd->dma_start;
    prtd->dma_end = prtd->dma_start + totbytes;
    spin_unlock_irq(&prtd->lock);

    /* codec part */
    reg = ac97_bus->ops->read(chip->reg + AC_CODEC_CMD,
			AC97_CENTER_LFE_MASTER) & 0xfff3;
    switch (params_format(hw_params)) {
    case SNDRV_PCM_FORMAT_S16_LE:
	break;
    case SNDRV_PCM_FORMAT_S20_3LE:
	reg |= 0x0004;
	break;
    case SNDRV_PCM_FORMAT_S24_LE:
	reg |= 0x0008;
	break;
    case SNDRV_PCM_FORMAT_S32_LE:
	reg |= 0x000c;
	break;
    }
    /* enable PCM interface in master mode */
    ac97_bus->ops->write(chip->reg + AC_CODEC_CMD,AC97_CENTER_LFE_MASTER, reg);
    return 0;
}

static int snd_wm9714_pcm_hw_free(struct snd_pcm_substream*substream)
{
    struct s3c6410_runtime_data *prtd = substream->runtime->private_data;
    
    pr_debug("Entered %s\n", __FUNCTION__);

    /* TODO - do we need to ensure DMA flushed */
    snd_pcm_set_runtime_buffer(substream, NULL);
    if (prtd->params){
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	    s3c2410_dma_free(prtd->params->channel,&pcm_out_client);
	} else {
	    s3c2410_dma_free(prtd->params->channel,&mic_in_client);
	}
    }
    snd_pcm_lib_free_pages(substream);
    return 0;
}

static int snd_wm9714_pcm_prepare(struct snd_pcm_substream*substream)
{
    struct wm9714_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_ac97_bus *ac97_bus = chip->ac97->bus;
    struct s3c6410_runtime_data *prtd = substream->runtime->private_data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    int ret = 0;
    int reg;
    u16 vra;
    u32 ac;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	s3c2410_dma_devconfig(prtd->params->channel,S3C2410_DMASRC_MEM,
			      prtd->params->dma_addr); 
    s3c2410_dma_config(prtd->params->channel,prtd->params->dma_size);
    /* flush the DMA channel */
    s3c2410_dma_ctrl(prtd->params->channel, S3C2410_DMAOP_FLUSH);
    prtd->dma_loaded = 0;
    prtd->dma_pos = prtd->dma_start;
    /* enqueue dma buffers */
    s3c2410_dma_enqueue(prtd->params->channel,substream,
			runtime->dma_addr,4);
    } else {
    s3c2410_dma_devconfig(prtd->params->channel,S3C2410_DMASRC_HW,
			      prtd->params->dma_addr); 
    s3c2410_dma_config(prtd->params->channel,prtd->params->dma_size);
    /* flush the DMA channel */
    s3c2410_dma_ctrl(prtd->params->channel, S3C2410_DMAOP_FLUSH);
    prtd->dma_loaded = 0;
    prtd->dma_pos = prtd->dma_start;
    /* enqueue dma buffers */
    s3c2410_dma_enqueue(prtd->params->channel,substream,
			runtime->dma_addr,2);
    }   
    vra = ac97_bus->ops->read(chip->reg + AC_CODEC_CMD,AC97_EXTENDED_STATUS);
    ac97_bus->ops->write(chip->reg + AC_CODEC_CMD,AC97_EXTENDED_STATUS,
		   vra | 0x1);
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	reg = AC97_PCM_FRONT_DAC_RATE;
    else
	reg = AC97_PCM_LR_ADC_RATE;
    ac97_bus->ops->write(chip->reg + AC_CODEC_CMD,reg, runtime->rate);
    ac = readl(chip->reg);
    ac |= 0x220c;
    writel(ac,reg);
    return ret;
}

static int snd_wm9714_pcm_trigger(struct snd_pcm_substream*substream,
				  int cmd)
{
    struct s3c6410_runtime_data *prtd = substream->runtime->private_data;
    int ret = 0;
    
    spin_lock(&prtd->lock);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	prtd->state |= ST_RUNNING;
	s3c2410_dma_ctrl(prtd->params->channel, S3C2410_DMAOP_START);
	s3c2410_dma_ctrl(prtd->params->channel, S3C2410_DMAOP_STARTED);
	break;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	prtd->state &= ~ST_RUNNING;
	s3c2410_dma_ctrl(prtd->params->channel, S3C2410_DMAOP_STOP);
	break;

    default:
	ret = -EINVAL;
	break;
    }

    spin_unlock(&prtd->lock);

    return ret;
}

static snd_pcm_uframes_t
snd_wm9714_pcm_pointer(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct s3c6410_runtime_data *prtd = runtime->private_data;
    unsigned long res;
    dma_addr_t src, dst;

    pr_debug("Entered %s\n", __FUNCTION__);

    spin_lock(&prtd->lock);
    s3c2410_dma_getposition(prtd->params->channel, &src, &dst);

    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	res = dst - prtd->dma_start;
    else
	res = src - prtd->dma_start;

    spin_unlock(&prtd->lock);

    pr_debug("Pointer %x %x\n", src, dst);

    if (res >= snd_pcm_lib_buffer_bytes(substream)) {
	if (res == snd_pcm_lib_buffer_bytes(substream))
	    res = 0;
    }

    return bytes_to_frames(substream->runtime, res);
}

static struct snd_pcm_ops snd_wm9714_playback_ops =
{
    .open = snd_wm9714_playback_open,
    .close = snd_wm9714_playback_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_wm9714_pcm_hw_params,
    .hw_free = snd_wm9714_pcm_hw_free,
    .prepare = snd_wm9714_pcm_prepare,
    .trigger = snd_wm9714_pcm_trigger,
    .pointer = snd_wm9714_pcm_pointer,
};

static struct snd_pcm_ops snd_wm9714_capture_ops =
{
    .open = snd_wm9714_capture_open,
    .close = snd_wm9714_capture_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_wm9714_pcm_hw_params,
    .hw_free = snd_wm9714_pcm_hw_free,
    .prepare = snd_wm9714_pcm_prepare,
    .trigger = snd_wm9714_pcm_trigger,
    .pointer = snd_wm9714_pcm_pointer,
};

int snd_wm9714_new_pcm(struct wm9714_chip *chip)
{
    int ret;
    struct snd_pcm *pcm;

    printk(KERN_ERR "%s: begin\n",__FUNCTION__);
    ret = snd_pcm_new(chip->card,"wm9714-codec",0,1,1,&pcm);
    if(ret < 0){
	printk(KERN_ERR "%s: Unable to create pcm\n",__FUNCTION__);
	return ret;
    }
    pcm->private_data = chip;
    strcpy(pcm->name,"wm9714-codec");
    chip->pcm = pcm;
    snd_pcm_set_ops(pcm,SNDRV_PCM_STREAM_PLAYBACK,&snd_wm9714_playback_ops);
    snd_pcm_set_ops(pcm,SNDRV_PCM_STREAM_CAPTURE,&snd_wm9714_capture_ops);
    snd_pcm_lib_preallocate_pages_for_all(pcm,SNDRV_DMA_TYPE_DEV,
					  chip->card->dev,
					  64 *1024, 64 *1024);
    printk(KERN_ERR "%s: end\n",__FUNCTION__);
    return 0;
}
