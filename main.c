#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <asm/system.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <linux/clk.h>

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
#include <plat/regs-ac97.h>
#include "pcm.h"
#include "wm9714.h"

#define AC_CMD_ADDR(x) (x << 16)
#define AC_CMD_DATA(x) (x & 0xffff)

static struct platform_device *s3c6410_wm9714_device;

unsigned short snd_wm9714_ac97_read(struct snd_ac97 *ac97,unsigned short reg)
{
    /* return snd_ac97_read(ac97,reg); */
    return 0;
}

void snd_wm9714_ac97_write(struct snd_ac97 *ac97,unsigned short reg,
			   unsigned short val)
{
    /* snd_ac97_write(ac97,reg,val); */
}

static struct snd_ac97_bus_ops ac97_ops = {
    .read = snd_wm9714_ac97_read,
    .write = snd_wm9714_ac97_write,
};
    

static int __devinit s3c6410_wm9714_probe(struct platform_device *dev)  
{
    int ret;
    struct snd_card *card;
    struct wm9714_chip *chip;
    struct snd_ac97_bus *ac97_bus;
    struct snd_ac97_template ac97;

    chip = kzalloc(sizeof(struct wm9714_chip),GFP_KERNEL);
    if(chip == NULL){
	ret = -ENOMEM;
	printk(KERN_ERR "%s: No memory for wm9714 chip struct\n",__func__);
	return ret;
    }
    mutex_init(&chip->lock);
    chip->bus_addr = BUS_ADDR;
    chip->reg_size = REG_SIZE;
    chip->channel = PCMOUT_CH;
    
    /* chip->dmach_pcmout = PCMOUT_CH; */
    /* chip->dmach_pcmin = PCMIN_CH; */
    /* chip->dmach_micin = MICIN_CH; */
    printk(KERN_ERR "%s: ioremap\n",__func__);
    chip->reg = NULL;
    chip->reg = ioremap(chip->bus_addr,chip->reg_size);
    if (chip->reg == NULL) {
	printk(KERN_ERR "%s: Unable to ioremap register region\n",__func__);
	ret = -ENXIO;
	goto err1;
    }
    printk(KERN_ERR "%s: card create\n",__func__);
    ret = snd_card_create(SNDRV_DEFAULT_IDX,"wm9714-codec",THIS_MODULE,
			  0, &card);  
    if (ret != 0){
	printk(KERN_ERR "%s: Card create failed\n",__func__);
        goto err2;
    }
    chip->card = card;
    printk(KERN_ERR "%s: Ac97 bus create\n",__func__);
    ret = snd_ac97_bus(card,0,&ac97_ops,chip,&ac97_bus);
    if(ret != 0){
    	printk(KERN_ERR "%s: Ac97 bus create failed\n",__func__);
    	goto err3;
    }
    memset(&ac97,0,sizeof(ac97));
    ac97.private_data = chip;
    printk(KERN_ERR "%s: Ac97 mixer\n",__func__);
    ret = snd_ac97_mixer(ac97_bus,&ac97,&chip->ac97);
    printk(KERN_ERR "%s: ret = %d\n",__func__,ret);
    if(ret != 0){
    	printk(KERN_ERR "%s: Mixer create failed\n",__func__);
    	goto err3;
    }
    printk(KERN_ERR "%s Pcm create\n",__func__);
    ret = snd_wm9714_new_pcm(chip);
    if(ret != 0){
    	printk(KERN_ERR "%s: Pcm create failed\n",__func__);
    	goto err3;
    }
    printk(KERN_ERR "%s is end\n",__func__);
    platform_set_drvdata(dev, chip);
    /* return 0; */
    
err3:
    snd_card_free(card);
err2:
    iounmap(chip->reg);
err1:
    kfree(chip);
    return ret;
}  
  
static int __devexit s3c6410_wm9714_remove(struct platform_device *dev)  
{
    struct wm9714_chip *chip = platform_get_drvdata(dev);

    snd_card_free(chip->card);
    iounmap(chip->reg);
    kfree(chip);
    return 0;
}  
  
static struct platform_driver s3c_wm9714_driver = {
    .probe      = s3c6410_wm9714_probe,  
    .remove     = __devexit_p(s3c6410_wm9714_remove),  
    .driver     = {  
        .name   = "s3c6410-wm9714",  
        .owner  = THIS_MODULE,  
    },  
};  
  
static int __init s3c_wm9714_driver_init(void)  
{
    int ret;

    s3c6410_wm9714_device = platform_device_alloc("s3c6410-wm9714", -1);
    if (!s3c6410_wm9714_device)
	return -ENOMEM;
    ret = platform_device_add(s3c6410_wm9714_device);
    if (ret)
	goto err1;
    return platform_driver_register(&s3c_wm9714_driver);

err1:
    platform_device_put(s3c6410_wm9714_device);
    return ret;
}  
  
static void __exit s3c_wm9714_driver_exit(void)  
{
    platform_device_put(s3c6410_wm9714_device);
    platform_driver_unregister(&s3c_wm9714_driver);  
}  
  
module_init(s3c_wm9714_driver_init);  
module_exit(s3c_wm9714_driver_exit);  
  
MODULE_AUTHOR("Liu");  
MODULE_DESCRIPTION("driver for wm9714"); 
MODULE_LICENSE("Dual BSD/GPL");
