#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include "ak4375.h"

/*
 * PLL means BLCk
 * NO PLL means mclk
 * */
#define PLL_BICK_MODE

//#define AK4375_DEBUG			//used at debug mode
#define AUDIO_NAME "#BC akm4375"

#ifdef AK4375_DEBUG
#define b_codec_dbg(format, arg...) \
    printk(KERN_INFO AUDIO_NAME ": " format, ## arg)

#define b_codec_err(format, arg...) \
    printk(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_info(format, arg...) \
    printk(KERN_INFO AUDIO_NAME ": " format, ## arg)

#define b_codec_warn(format, arg...) \
    printk(KERN_WARNING AUDIO_NAME ": " format, ## arg)

#define b_codec_trace() \
    printk("%s(%d)\n", __func__, __LINE__)
#else
#define b_codec_dbg(format, arg...) do {} while (0)
#define b_codec_info(format, arg...) do {} while (0)
#define b_codec_warn(format, arg...) do {} while (0)
#define b_codec_trace(format, arg...) do {} while (0)
#define b_codec_err(format, arg...) \
    printk(KERN_ERR AUDIO_NAME ": " format, ## arg)

#endif


int g_dump_trace = 0;


/* AK4375 Codec Private Data */
struct ak4375_priv {
    struct snd_soc_codec codec;
    u8 reg_cache[AK4375_MAX_REGISTERS];
    int fs1;
    int fs2;
    int rclk;			//Master Clock
    int nSeldain;		//0:Bypass, 1:SRC	(Dependent on a register bit)
    int nBickFreq;		//0:32fs, 1:48fs, 2:64fs
    int nSrcOutFsSel;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
    int nPllMode;		//0:PLL OFF, 1: PLL ON
    int nPllMCKI;		//0:PLL not use, 1: PLL use
    int nSmt;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
    int dfsrc8fs;		//DFTHR bit and SRCO8FS bit
};

struct ak4375_sys_data_s{
    u32 dac_ctl_pin;
    u32 dac_ctl_pin_flags;
    u32 ldo_ctl_pin;
    u32 ldo_ctl_pin_flags;
    struct regulator *vdd;
    struct i2c_client *client;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
};

static struct ak4375_sys_data_s *ak4375_sys_data;
static struct snd_soc_codec *ak4375_codec;
static struct ak4375_priv *ak4375_data;

/* ak4375 register cache & default register settings */
static const u8 ak4375_reg[AK4375_MAX_REGISTERS] = {
    0x00,	/*	0x00	AK4375_00_POWER_MANAGEMENT1			*/
    0x00,	/*	0x01	AK4375_01_POWER_MANAGEMENT2			*/
    0x00,	/*	0x02	AK4375_02_POWER_MANAGEMENT3			*/
    0x00,	/*	0x03	AK4375_03_POWER_MANAGEMENT4			*/
    0x00,	/*	0x04	AK4375_04_OUTPUT_MODE_SETTING		*/
    0x00,	/*	0x05	AK4375_05_CLOCK_MODE_SELECT			*/
    0x00,	/*	0x06	AK4375_06_DIGITAL_FILTER_SELECT		*/
    0x00,	/*	0x07	AK4375_07_DAC_MONO_MIXING			*/
    0x00,	/*	0x08	AK4375_08_JITTER_CLEANER_SETTING1	*/
    0x00,	/*	0x09	AK4375_09_JITTER_CLEANER_SETTING2	*/
    0x00,	/*	0x0A	AK4375_0A_JITTER_CLEANER_SETTING3	*/
    0x19,	/*	0x0B	AK4375_0B_LCH_OUTPUT_VOLUME			*/
    0x19,	/*	0x0C	AK4375_0C_RCH_OUTPUT_VOLUME			*/
    0x75,	/*	0x0D	AK4375_0D_HP_VOLUME_CONTROL			*/
    0x01,	/*	0x0E	AK4375_0E_PLL_CLK_SOURCE_SELECT		*/
    0x00,	/*	0x0F	AK4375_0F_PLL_REF_CLK_DIVIDER1		*/
    0x00,	/*	0x10	AK4375_10_PLL_REF_CLK_DIVIDER2		*/
    0x00,	/*	0x11	AK4375_11_PLL_FB_CLK_DIVIDER1		*/
    0x00,	/*	0x12	AK4375_12_PLL_FB_CLK_DIVIDER2		*/
    0x00,	/*	0x13	AK4375_13_SRC_CLK_SOURCE			*/
    0x00,	/*	0x14	AK4375_14_DAC_CLK_DIVIDER			*/
    0x00,	/*	0x15	AK4375_15_AUDIO_IF_FORMAT			*/
    0x00,	/*	0x16	AK4375_16_DUMMY						*/
    0x00,	/*	0x17	AK4375_17_DUMMY						*/
    0x00,	/*	0x18	AK4375_18_DUMMY						*/
    0x00,	/*	0x19	AK4375_19_DUMMY						*/
    0x00,	/*	0x1A	AK4375_1A_DUMMY						*/
    0x00,	/*	0x1B	AK4375_1B_DUMMY						*/
    0x00,	/*	0x1C	AK4375_1C_DUMMY						*/
    0x00,	/*	0x1D	AK4375_1D_DUMMY						*/
    0x00,	/*	0x1E	AK4375_1E_DUMMY						*/
    0x00,	/*	0x1F	AK4375_1F_DUMMY						*/
    0x00,	/*	0x20	AK4375_20_DUMMY						*/
    0x00,	/*	0x21	AK4375_21_DUMMY						*/
    0x00,	/*	0x22	AK4375_22_DUMMY						*/
    0x00,	/*	0x23	AK4375_23_DUMMY						*/
    0x00,	/*	0x24	AK4375_24_MODE_CONTROL				*/
};

static const struct {
    int readable;   /* Mask of readable bits */
    int writable;   /* Mask of writable bits */
} ak4375_access_masks[] = {
    { 0xFF, 0xFF },	//0x00
    { 0xFF, 0xFF },	//0x01
    { 0xFF, 0xFF },	//0x02
    { 0xFF, 0xFF },	//0x03
    { 0xFF, 0xFF },	//0x04
    { 0xFF, 0xFF },	//0x05
    { 0xFF, 0xFF },	//0x06
    { 0xFF, 0xFF },	//0x07
    { 0xFF, 0xFF },	//0x08
    { 0xFF, 0xFF },	//0x09
    { 0xFF, 0xFF },	//0x0A
    { 0xFF, 0xFF },	//0x0B
    { 0xFF, 0xFF },	//0x0C
    { 0xFF, 0xFF },	//0x0D
    { 0xFF, 0xFF },	//0x0E
    { 0xFF, 0xFF },	//0x0F
    { 0xFF, 0xFF },	//0x10
    { 0xFF, 0xFF },	//0x11
    { 0xFF, 0xFF },	//0x12
    { 0xFF, 0xFF },	//0x13
    { 0xFF, 0xFF },	//0x14
    { 0xFF, 0xFF },	//0x15
    { 0x00, 0x00 },	//0x16	//DUMMY
    { 0x00, 0x00 },	//0x17	//DUMMY
    { 0x00, 0x00 },	//0x18	//DUMMY
    { 0x00, 0x00 },	//0x19	//DUMMY
    { 0x00, 0x00 },	//0x1A	//DUMMY
    { 0x00, 0x00 },	//0x1B	//DUMMY
    { 0x00, 0x00 },	//0x1C	//DUMMY
    { 0x00, 0x00 },	//0x1D	//DUMMY
    { 0x00, 0x00 },	//0x1E	//DUMMY
    { 0x00, 0x00 },	//0x1F	//DUMMY
    { 0x00, 0x00 },	//0x20	//DUMMY
    { 0x00, 0x00 },	//0x21	//DUMMY
    { 0x00, 0x00 },	//0x22	//DUMMY
    { 0x00, 0x00 },	//0x23	//DUMMY
    { 0xFF, 0xFF },	//0x24
};


void dump_trace(void)
{
    if (g_dump_trace) {
        dump_stack();
    }
}


/*
 * Read ak4375 register cache
 */
static inline u32 ak4375_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4375_reg));
    return (u32)cache[reg];
}



static unsigned int ak4375_fake_read(struct snd_soc_codec *codec,  u_int reg)
{
    b_codec_dbg("%s (%d)\n",__FUNCTION__, __LINE__);
    return 0;
}

static int ak4375_fake_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
{
    b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);
    return 0;
}


//#define AK4375_DUMMY
//#define I2C_RW_CACHE_MODE

/*
 * for normal, trust i2c operation, using cache
 * for bad, not using cache, write and read back is needed
 * */
#ifdef I2C_RW_CACHE_MODE
#else
static unsigned int ak4375_read(struct snd_soc_codec *codec,  u_int reg)
{
    int ret;

#ifdef AK4375_DUMMY
    return ak4375_fake_read(codec, reg);
#endif

    if (reg == AK4375_16_DUMMY) { // Dummy Register.
        ret = ak4375_read_reg_cache(codec, reg);
        return ret;
    }

    ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
    if (ret < 0) {
        b_codec_err("%s (%d)\n",__FUNCTION__, __LINE__);
    }

    return ret;
}

static int ak4375_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
{
    b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

#ifdef AK4375_DUMMY
    return ak4375_fake_write(codec, reg, value);
#endif

    if (reg == AK4375_16_DUMMY) {
        return 0;
    }

    if (i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
        b_codec_err("%s(%d) error\n",__FUNCTION__,__LINE__);
        return EIO;
    }

    return 0;
}

static int ak4375_write_mask(struct snd_soc_codec *codec, u32 reg, u32 mask, u32 value)
{
    u32 old;
    u32 new;
    int ret = 0;

    old = ak4375_read(codec, reg);
    new = (old & ~(mask)) | value;
    ret = ak4375_write(codec, reg, new);

    return ret;
}
#endif


void ak4375_ldo_power_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4375_sys_data->ldo_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4375_sys_data->ldo_ctl_pin, 0);
    }
}


void ak4375_dac_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4375_sys_data->dac_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4375_sys_data->dac_ctl_pin, 0);
    }
}

int ak4375_hw_params_set(struct snd_soc_codec *codec, int nfs1);

void ak4375_bclk_mode(struct snd_soc_codec *codec)
{
    printk("\n\n>>> BLCK MODE INIT <<<\n\n");

    /*
     *  AKM FAE suggest move power related regs into on/off
     *  but for 03 set 1/2 vdd time no need
     * */
#if 0
    // 0x00 == 0x00 PLL start for blck pmosc stop
    ak4375_write(codec, AK4375_00_POWER_MANAGEMENT1, 0x01);
#endif

    // LR amp power management
    // 1/2 VDD settting capless right?  QQQQQQQQQQ
    // not open LR amp
    ak4375_write(codec, AK4375_03_POWER_MANAGEMENT4, 0x50);


    // VDD hold setting QQQ
    ak4375_write(codec, AK4375_04_OUTPUT_MODE_SETTING, 0x14);


    //256fs, 48KHZ
    ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, 0x0a);

    //Sharp Roll-Off Filter
    ak4375_write(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x00);

    //LR ch select
    ak4375_write(codec, AK4375_07_DAC_MONO_MIXING, 0x21);

    //FIXME
    ak4375_write(codec, AK4375_08_JITTER_CLEANER_SETTING1, 0x00);


    // 0x00 src alll set default
    ak4375_write(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x00);


    // DAC Input Data Select = SDATA not src
    // DAC Operation Clock  = SRCMCLK
    // Charge Pump Operation Clock = XCKCPSEL
    ak4375_write(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0x00);


    // AK4375_0B_LCH_OUTPUT_VOLUME = 0x19
    // AK4375_0C_RCH_OUTPUT_VOLUME = 0x19
    ak4375_write(codec, AK4375_0B_LCH_OUTPUT_VOLUME, 0x19);
    ak4375_write(codec, AK4375_0C_RCH_OUTPUT_VOLUME, 0x19);

    // 0x0d = 0x75 0db  zero cross time
    ak4375_write(codec, AK4375_0D_HP_VOLUME_CONTROL, 0x73);

    // 0x0e == 0x00 SCLK
    ak4375_write(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x01);

    // PLL
    ak4375_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, 0x00);
    ak4375_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, 0x4f);
    ak4375_write(codec, AK4375_13_SRC_CLK_SOURCE, 0x01);
    ak4375_write(codec, AK4375_14_DAC_CLK_DIVIDER, 0x09);

    // I2S 16bit
    ak4375_write(codec, AK4375_15_AUDIO_IF_FORMAT, 0x01);

    // no desample
    ak4375_write(codec, AK4375_24_MODE_CONTROL, 0x00);
}


void ak4375_mclk_mode(struct snd_soc_codec *codec)
{
    printk("\n\n>>> MCLK MODE INIT <<<\n\n");

    // 0x00 == 0x00 PLL stop pmosc stop
    ak4375_write(codec, AK4375_00_POWER_MANAGEMENT1, 0x00);

    // LR amp power management
    ak4375_write(codec, AK4375_03_POWER_MANAGEMENT4, 0x53);

    // VDD hold setting QQQ
    ak4375_write(codec, AK4375_04_OUTPUT_MODE_SETTING, 0x14);

    //256fs, 48KHZ
    ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, 0x0a);

    //Sharp Roll-Off Filter
    ak4375_write(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x00);

    //LR ch select
    ak4375_write(codec, AK4375_07_DAC_MONO_MIXING, 0x21);

    //FIXME
    ak4375_write(codec, AK4375_08_JITTER_CLEANER_SETTING1, 0x00);


    // 0x00 src alll set default
    ak4375_write(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x00);


    // DAC Input Data Select = SDATA not src
    // DAC Operation Clock  = SRCMCLK
    // Charge Pump Operation Clock = XCKCPSEL
    ak4375_write(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0x00);


    // AK4375_0B_LCH_OUTPUT_VOLUME = 0x19
    // AK4375_0C_RCH_OUTPUT_VOLUME = 0x19
    ak4375_write(codec, AK4375_0B_LCH_OUTPUT_VOLUME, 0x19);
    ak4375_write(codec, AK4375_0C_RCH_OUTPUT_VOLUME, 0x19);

    // 0x0d = 0x75 0db  zero cross time
    ak4375_write(codec, AK4375_0D_HP_VOLUME_CONTROL, 0x73);

    // 0x0e == 0x00
    ak4375_write(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x00);

    // PLL
    ak4375_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, 0x00);
    ak4375_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, 0x00);
    ak4375_write(codec, AK4375_13_SRC_CLK_SOURCE, 0x00);
    ak4375_write(codec, AK4375_13_SRC_CLK_SOURCE, 0x00);
    ak4375_write(codec, AK4375_14_DAC_CLK_DIVIDER, 0x00);

    // I2S 16bit
    ak4375_write(codec, AK4375_15_AUDIO_IF_FORMAT, 0x01);

    // no desample
    ak4375_write(codec, AK4375_24_MODE_CONTROL, 0x00);
}

static int g_codec_hp_state = 0;
enum hp_ctl_enum {
    HP_OFF = 0,
    HP_ON,
};

int akm4375_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("#B get %s  hp state %d\n", __func__, g_codec_hp_state);
    ucontrol->value.integer.value[0] = g_codec_hp_state;
    dump_trace();
    return 0;
}

/*
 * keep it simple and quick
 * check each mdelay and udelay later
 * follow spec P43
 * */
int akm4375_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4375_codec;
    int state = ucontrol->value.enumerated.item[0];

    printk("\n>>> %s  set  %d <<<\n", __func__, state);

    dump_trace();

    if (state == HP_ON)
    {
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x03);

        //spec need 25.9ms@44K1
    }
    else
    {
        //close hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00);
    }

    g_codec_hp_state = state;

    return 0;
}

/*
 *  for ftm loopback test
 * */
static int g_codec_ftm_hp_state = 0;

enum ftm_hp_ctl_enum {
    FTM_HP_OFF = 0,
    FTM_HP_LCH,
    FTM_HP_RCH,
};

int akm4375_ftm_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("#B get %s  hp state %d\n", __func__, g_codec_ftm_hp_state);
    ucontrol->value.integer.value[0] = g_codec_ftm_hp_state;
    dump_trace();
    return 0;
}

int akm4375_ftm_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4375_codec;
    int state = ucontrol->value.enumerated.item[0];

    printk("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    dump_trace();

    if (state == FTM_HP_OFF)
    {
        //close hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00);
    }
    else
    {
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        if (state == FTM_HP_LCH)
        {
            ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x01);
        }
        else
        {
            ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x02);
        }

        //spec need 25.9ms@44K1
    }

    g_codec_ftm_hp_state = state;

    return 0;
}

/*
 * Bruce add for one control on/off
 * headphone on/off
 * */
static const char *hp_ctl_txt[] = {"off", "on"};

static const struct soc_enum hp_control_enum[] = {
    SOC_ENUM_SINGLE_EXT(2, hp_ctl_txt),
};


static const char *ftm_hp_ctl_txt[] = {"off", "lch", "rch"};
static const struct soc_enum ftm_hp_control_enum[] = {
    SOC_ENUM_SINGLE_EXT(3, ftm_hp_ctl_txt),
};

static const struct snd_kcontrol_new ak4375_snd_controls[] = {
    SOC_ENUM_EXT("AKM HP", hp_control_enum[0], akm4375_get_hp, akm4375_set_hp),
    SOC_ENUM_EXT("AKM FTM HP", ftm_hp_control_enum[0], akm4375_ftm_get_hp, akm4375_ftm_set_hp),
};

/* ak4375 dapm widgets */
static const struct snd_soc_dapm_widget ak4375_dapm_widgets[] = {
};

static const struct snd_soc_dapm_route ak4375_intercon[] = {
};

static int ak4375_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
    dump_trace();
    return 0;
}

static int ak4375_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
        unsigned int freq, int dir)
{
    b_codec_dbg("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);
    dump_trace();
    return 0;
}

static int ak4375_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    dump_trace();
    return 0;
}

static int ak4375_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    return true;
}

static int ak4375_readable(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    if (reg >= ARRAY_SIZE(ak4375_access_masks))
        return 0;
    return ak4375_access_masks[reg].readable != 0;
}

/*
 * keep it for future,
 * on/off just flush register table, no +-x/
 * */
#if 0
static int ak4375_set_pllblock(struct snd_soc_codec *codec, int fs)
{
    u8 mode;
    int nMClk, nPLLClk, nRefClk;
    int PLDbit, PLMbit, MDIVbit, DIVbit;
    int nTemp;

    mode = ak4375_read(codec, AK4375_05_CLOCK_MODE_SELECT);
    mode &= ~AK4375_CM;

    if (ak4375_data->nSeldain == 0) {	//SRC bypass
        if ( fs <= 24000 ) {
            mode |= AK4375_CM_1;
            nMClk = 512 * fs;
        }
        else if ( fs <= 96000 ) {
            mode |= AK4375_CM_0;
            nMClk = 256 * fs;
        }
        else {
            mode |= AK4375_CM_3;
            nMClk = 128 * fs;
        }
    }
    else {								//SRC
        if ( fs <= 24000 ) {
            mode |= AK4375_CM_1;
            nMClk = 512 * fs;
        }
        else  {
            mode |= AK4375_CM_0;
            nMClk = 256 * fs;
        }
    }
    ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, mode);

    if ( (fs % 8000) == 0 ) {
        nPLLClk = 122880000;
    }
    else if ( (fs == 11025 ) && ( ak4375_data->nBickFreq == 1 )) {
        nPLLClk = 101606400;
    }
    else {
        nPLLClk = 112896000;
    }

    if ( ak4375_data->nBickFreq == 0 ) {		//32fs
        if ( fs <= 96000 ) PLDbit = 1;
        else PLDbit = 2;
        nRefClk = 32 * fs / PLDbit;
    }
    else if ( ak4375_data->nBickFreq == 1 ) {	//48fs
        if ( fs <= 16000 ) PLDbit = 1;
        else PLDbit = 3;
        nRefClk = 48 * fs / PLDbit;
    }
    else {  									// 64fs
        if ( fs <= 48000 ) PLDbit = 1;
        else if ( fs <= 96000 ) PLDbit = 2;
        else PLDbit = 4;
        nRefClk = 64 * fs / PLDbit;
    }

    PLMbit = nPLLClk / nRefClk;

    if ( ( ak4375_data->nSeldain == 0 ) || ( fs <= 96000 ) ) {
        MDIVbit = nPLLClk / nMClk;
        DIVbit = 0;
    }
    else {
        MDIVbit = 5;
        DIVbit = 1;
    }

    PLDbit--;
    PLMbit--;
    MDIVbit--;

    //PLD15-0
    ak4375_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
    ak4375_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
    //PLM15-0
    ak4375_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
    ak4375_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
    //DIVbit
    nTemp = ak4375_read(codec, AK4375_13_SRC_CLK_SOURCE);
    nTemp &= ~0x10;
    nTemp |= ( DIVbit << 4 );
    ak4375_write(codec, AK4375_13_SRC_CLK_SOURCE, (nTemp|0x01));		//DIV=0or1,SRCCKS=1(SRC Clock Select=PLL) set
    ak4375_write_mask(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x01, 0x01);	//PLS=1(BICK)

    //MDIV7-0
    ak4375_write(codec, AK4375_14_DAC_CLK_DIVIDER, MDIVbit);

    return 0;
}

static int ak4375_set_timer(struct snd_soc_codec *codec)
{
    int ret, curdata;
    int count, tm, nfs;
    int lvdtm, vddtm, hptm;

    lvdtm = 0;
    vddtm = 0;
    hptm = 0;

    if ( ak4375_data->nSeldain == 1 ) nfs = ak4375_data->fs2;
    else 	nfs = ak4375_data->fs1;

    //LVDTM2-0 bits set
    ret = ak4375_read(codec, AK4375_03_POWER_MANAGEMENT4);
    curdata = (ret & 0x70) >> 4;	//Current data Save
    ret &= ~0x70;
    do {
        count = 1000 * (64 << lvdtm);
        tm = count / nfs;
        if ( tm > LVDTM_HOLD_TIME ) break;
        lvdtm++;
    } while ( lvdtm < 7 );			//LVDTM2-0 = 0~7
    if ( curdata != lvdtm) {
        ak4375_write(codec, AK4375_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));
    }

    //VDDTM3-0 bits set
    ret = ak4375_read(codec, AK4375_04_OUTPUT_MODE_SETTING);
    curdata = (ret & 0x3C) >> 2;	//Current data Save
    ret &= ~0x3C;
    do {
        count = 1000 * (1024 << vddtm);
        tm = count / nfs;
        if ( tm > VDDTM_HOLD_TIME ) break;
        vddtm++;
    } while ( vddtm < 8 );			//VDDTM3-0 = 0~8
    if ( curdata != vddtm) {
        ak4375_write(codec, AK4375_04_OUTPUT_MODE_SETTING, (ret | (vddtm<<2)));
    }

    //HPTM2-0 bits set
    ret = ak4375_read(codec, AK4375_0D_HP_VOLUME_CONTROL);
    curdata = (ret & 0xE0) >> 5;	//Current data Save
    ret &= ~0xE0;
    do {
        count = 1000 * (128 << hptm);
        tm = count / nfs;
        if ( tm > HPTM_HOLD_TIME ) break;
        hptm++;
    } while ( hptm < 4 );			//HPTM2-0 = 0~4
    if ( curdata != hptm) {
        ak4375_write(codec, AK4375_0D_HP_VOLUME_CONTROL, (ret | (hptm<<5)));
    }

    return 0;
}


int ak4375_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
    u8 	fs;
    u8  src;

    printk("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

    src = ak4375_read(codec, AK4375_0A_JITTER_CLEANER_SETTING3);
    src = (src & 0x02) >> 1;

    fs = ak4375_read(codec, AK4375_05_CLOCK_MODE_SELECT);
    fs &= ~AK4375_FS;

    //	ak4375_data->fs1 = params_rate(params);

    switch (nfs1) {
        case 8000:
            fs |= AK4375_FS_8KHZ;
            break;
        case 11025:
            fs |= AK4375_FS_11_025KHZ;
            break;
        case 16000:
            fs |= AK4375_FS_16KHZ;
            break;
        case 22050:
            fs |= AK4375_FS_22_05KHZ;
            break;
        case 32000:
            fs |= AK4375_FS_32KHZ;
            break;
        case 44100:
            fs |= AK4375_FS_44_1KHZ;
            break;
        case 48000:
            fs |= AK4375_FS_48KHZ;
            break;
        case 88200:
            fs |= AK4375_FS_88_2KHZ;
            break;
        case 96000:
            fs |= AK4375_FS_96KHZ;
            break;
        case 176400:
            fs |= AK4375_FS_176_4KHZ;
            break;
        case 192000:
            fs |= AK4375_FS_192KHZ;
            break;
        default:
            return -EINVAL;
    }
    ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, fs);

    if ( ak4375_data->nPllMode == 0 ) {	//Not PLL mode
#if 0
        ak4375_set_mcki(codec, nfs1, ak4375_data->rclk);
#endif
    }
    else {								//PLL mode
        ak4375_set_pllblock(codec, nfs1);
    }

    if ( src == 1 ) {				//SRC mode
#if 0
        ak4375_data->nSeldain = 1;
        ak4375_write_mask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0xC2);	//XCKSEL=XCKCPSEL=SELDAIN=1
        ak4375_set_src_mcki(codec, nfs1);
#endif
    }
    else {							//SRC Bypass mode
        ak4375_data->nSeldain = 0;
        ak4375_write_mask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0x00);	//XCKSEL=XCKCPSEL=SELDAIN=0
    }

    ak4375_set_timer(codec);

    return 0;
}
#endif


/*
 * platform data
 * */
static int ak4375_parse_dt(struct device *dev, struct ak4375_sys_data_s *sys_data)
{
    struct device_node *np = dev->of_node;
    sys_data->dac_ctl_pin = of_get_named_gpio_flags(np, "akm,dac-gpio", 0, &(sys_data->dac_ctl_pin_flags));
    if (sys_data->dac_ctl_pin < 0) {
        b_codec_err("sys_data->dac_ctl_pin\n");
        return sys_data->dac_ctl_pin;
    }

    sys_data->ldo_ctl_pin = of_get_named_gpio_flags(np, "akm,ldo-gpio", 0, &(sys_data->ldo_ctl_pin_flags));
    if (sys_data->ldo_ctl_pin < 0) {
        b_codec_err("sys_data->ldo_ctl_pin\n");
        return sys_data->ldo_ctl_pin;
    }

    return 0;
}


static int ak4375_pinctrl_init(struct ak4375_sys_data_s *sys_data)
{
    int ret;

    sys_data->pinctrl = devm_pinctrl_get(&(sys_data->client->dev));
    if (IS_ERR_OR_NULL(sys_data->pinctrl)) {
        ret = PTR_ERR(sys_data->pinctrl);
        b_codec_err("sys_data->pinctrl\n");
        goto err_pinctrl_get;
    }

    sys_data->pinctrl_state_active = pinctrl_lookup_state(sys_data->pinctrl, "ak4375_dac_active");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_active)) {
        ret = PTR_ERR(sys_data->pinctrl_state_active);
        b_codec_err("sys_data->pinctrl_state_active\n");
        goto err_pinctrl_lookup;
    }

    sys_data->pinctrl_state_suspend = pinctrl_lookup_state(sys_data->pinctrl, "ak4375_dac_suspend");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_suspend)) {
        ret = PTR_ERR(sys_data->pinctrl_state_suspend);
        b_codec_err("sys_data->pinctrl_state_suspend\n");
        goto err_pinctrl_lookup;
    }

    ret = pinctrl_select_state(sys_data->pinctrl, sys_data->pinctrl_state_active);
    if (ret) {
        b_codec_err("sys_data->pinctrl_state_active\n");
        return ret;
    }
    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(sys_data->pinctrl);

err_pinctrl_get:
    sys_data->pinctrl = NULL;
    return ret;
}


/*
 * default ouput 0
 * follow akm startup timeline
 * */
static int ak4375_config_pins(struct ak4375_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->dac_ctl_pin, "akm_dac_ctl") < 0) {
        b_codec_err("gpio err sys_data->dac_ctl_pin\n");
        return -1;
    }
    gpio_direction_output(sys_data->dac_ctl_pin, 0);


    if (gpio_request(sys_data->ldo_ctl_pin, "akm_ldo_ctl") < 0) {
        b_codec_err("gpio err sys_data->ldo_ctl_pin\n");
        return -1;
    }
    gpio_direction_output(sys_data->ldo_ctl_pin, 0);

    return  0;
}


static int ak4375_regulator_init(struct ak4375_sys_data_s *sys_data)
{
    int rc = -1;

    /*
     * codec V_L6_1P8 src VREG_L6
     * I2C pull up  V_L6_1P8 src VREG_L6
     * */
    sys_data->vdd = regulator_get(&sys_data->client->dev, "vdd");
    if (IS_ERR(sys_data->vdd)) {
        b_codec_err("%s %d get vdd error\n", __func__, __LINE__);
        goto err_get_vdd;
    }

    if (regulator_count_voltages(sys_data->vdd) > 0)
    {
        rc = regulator_set_voltage(sys_data->vdd, 1800000, 1800000);
        if (rc) {
            b_codec_err("%s %d set vdd error\n", __func__, __LINE__);
            goto err_set_vdd;
        }
    }

    /*
     * VLDO_1V8  <== LDO src (VPH_PWR == VBATT)
     * */
    rc = regulator_enable(sys_data->vdd);
    if (rc) {
        b_codec_err("Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }


    return 0;

err_set_vdd:
    regulator_put(sys_data->vdd);

err_get_vdd:
    if (regulator_count_voltages(sys_data->vdd) > 0) {
        regulator_set_voltage(sys_data->vdd, 0, 1800000);
    }

    return rc;
}




/*
 * Bruce FIXME QAD
 * */
#include "ak4375_debug.c"

static int ak4375_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
    b_codec_dbg("\t[AK4375] %s(%d) cmd=0x%x\n", __FUNCTION__, __LINE__, cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            b_codec_dbg("start resume pause\n");
            break;

        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            b_codec_dbg("stop suspend push\n");
            break;
    }

    dump_trace();
    return 0;
}

static int ak4375_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
    dump_trace();
    return 0;
}

#define AK4375_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
        SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
        SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
        SNDRV_PCM_RATE_192000)

#define AK4375_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4375_dai_ops = {
    .hw_params	= ak4375_hw_params,
    .set_sysclk	= ak4375_set_dai_sysclk,
    .set_fmt	= ak4375_set_dai_fmt,
    .trigger = ak4375_trigger,
    .digital_mute = ak4375_set_dai_mute,
};

struct snd_soc_dai_driver ak4375_dai[] = {
    {
        .name = "ak4375-AIF1",
        .playback = {
            .stream_name = "Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = AK4375_RATES,
            .formats = AK4375_FORMATS,
        },
        .ops = &ak4375_dai_ops,
    },
};


/*
 * check pre-init with spec and FAE
 * dtsi keep releated pin output 0 no bias
 * */
static int ak4375_pre_init(struct snd_soc_codec *codec)
{
    ak4375_dac_on(0);
    mdelay(1);

    // power on AVDD LVDD
    ak4375_ldo_power_on(1);

    mdelay(3);

    // PDN pin to high
    ak4375_dac_on(1);

    // if system loop in suspend/resume
    // the first reg may write error
    // check it with FAE
    mdelay(3);

    if (ak4375_data->nPllMode)
    {
        ak4375_bclk_mode(codec);
    }
    else
    {
        ak4375_mclk_mode(codec);
    }

    return 0;
}

static int ak4375_probe(struct snd_soc_codec *codec)
{
    struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;

    b_codec_dbg("%s(%d)\n", __FUNCTION__, __LINE__);

    ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
        return ret;
    }

    /*
     * just for trace asoc GDS
     * */
    codec->write = ak4375_fake_write;
    codec->read = ak4375_fake_read;
    codec->control_data = ak4375_sys_data->client;

    ak4375->fs1 = 48000;
    ak4375->fs2 = 48000;
    ak4375->rclk = 0;
    ak4375->nSeldain = 0;		//0:Bypass, 1:SRC	(Dependent on a register bit)
    ak4375->nBickFreq = 0;		//0:32fs, 1:48fs, 2:64fs
    ak4375->nSrcOutFsSel = 0;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
#ifdef PLL_BICK_MODE
    ak4375->nPllMode = 1;		//1: PLL ON
#else
    ak4375->nPllMode = 0;		//0:PLL OFF
#endif
    ak4375->nSmt = 0;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
    ak4375->dfsrc8fs = 0;		//0:DAC Filter, 1:Bypass, 2:8fs mode


    b_codec_dbg("\n\n\n DUMP AK4375 CONFIG \n\n\n");
    b_codec_dbg("ak4375->fs1 = %d\n", ak4375->fs1);
    b_codec_dbg("ak4375->fs2 = %d\n", ak4375->fs2);
    b_codec_dbg("ak4375->rclk = %d\n", ak4375->rclk);
    b_codec_dbg("ak4375->nSeldain = %d\n", ak4375->nSeldain);
    b_codec_dbg("ak4375->nBickFreq = %d\n", ak4375->nBickFreq);
    b_codec_dbg("ak4375->nSrcOutFsSel = %d\n", ak4375->nSrcOutFsSel);
    b_codec_dbg("ak4375->nPllMode = %d\n", ak4375->nPllMode);
    b_codec_dbg("ak4375->nSmt = %d\n", ak4375->nSmt);
    b_codec_dbg("ak4375->dfsrc8fs = %d\n", ak4375->dfsrc8fs);

    ak4375_pre_init(ak4375_codec);

    return ret;
}

static int ak4375_suspend(struct snd_soc_codec *codec)
{
    int rc = 0;

    printk("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {
        // codec pdn pin down, extern LDO off
        rc = pinctrl_select_state(ak4375_sys_data->pinctrl, ak4375_sys_data->pinctrl_state_suspend);
        if (rc) {
            b_codec_err("ak4375_sys_data->pinctrl_state_suspend error rc=%d\n", rc);
        }

        //L6 LDO off
        rc = regulator_disable(ak4375_sys_data->vdd);
        if (rc) {
            b_codec_err("Regulator vdd disable failed rc=%d\n", rc);
        }
    }
    else
    {
        // keep everything on
    }

    return 0;
}

static int ak4375_resume(struct snd_soc_codec *codec)
{
    int rc = 0;

    printk("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {
        rc = pinctrl_select_state(ak4375_sys_data->pinctrl, ak4375_sys_data->pinctrl_state_active);
        if (rc) {
            b_codec_err("ak4375_sys_data->pinctrl_state_active error rc=%d\n", rc);
        }

        //L6 LDO on
        rc = regulator_enable(ak4375_sys_data->vdd);
        if (rc) {
            b_codec_err("Regulator vdd enable failed rc=%d\n", rc);
        }

        ak4375_pre_init(ak4375_codec);
    }
    else
    {
        //everything is on
    }

    return 0;
}

static int ak4375_remove(struct snd_soc_codec *codec)
{
    b_codec_err("%s (%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4375 = {
    .probe = ak4375_probe,
    .remove = ak4375_remove,

    .suspend =	ak4375_suspend,
    .resume =	ak4375_resume,


    .reg_cache_size = ARRAY_SIZE(ak4375_reg),
    .reg_word_size = sizeof(u8),
    .reg_cache_default = ak4375_reg,
    .readable_register = ak4375_readable,
    .volatile_register = ak4375_volatile,

    .controls = ak4375_snd_controls,
    .num_controls = ARRAY_SIZE(ak4375_snd_controls),
    .dapm_widgets = ak4375_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(ak4375_dapm_widgets),
    .dapm_routes = ak4375_intercon,
    .num_dapm_routes = ARRAY_SIZE(ak4375_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4375);



static int ak4375_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int rc = 0;
    struct snd_soc_codec *codec;

    b_codec_dbg("ak4375_snd_controls = %d\n", (int)ARRAY_SIZE(ak4375_snd_controls));

    b_codec_trace();

    ak4375_data = kzalloc(sizeof(struct ak4375_priv), GFP_KERNEL);
    if (ak4375_data == NULL) {
         b_codec_err("no memory\n");
        return -ENOMEM;
    }


    b_codec_trace();

    ak4375_sys_data = kzalloc(sizeof(struct ak4375_sys_data_s), GFP_KERNEL);
    if (ak4375_sys_data == NULL) {
        kfree(ak4375_data);
        b_codec_err("no memory\n");
        return -ENOMEM;
    }

    b_codec_trace();


    /*
     * init structure first!!!
     * */
    codec = &ak4375_data->codec;
    ak4375_sys_data->client = i2c;
    codec->control_data = i2c;
    codec->dev = &i2c->dev;
    ak4375_codec = codec;

    b_codec_trace();

    /*
     * get resource from OF!
     * */
    rc = ak4375_parse_dt(&(i2c->dev), ak4375_sys_data);
    if (rc) {
        b_codec_err("ak4375_parse_dt error %d\n", rc);
    }

    b_codec_trace();


    rc = ak4375_regulator_init(ak4375_sys_data);
    if (rc < 0) {
        b_codec_err("get regulator error %d\n", rc);
    }

    b_codec_trace();

    rc = ak4375_pinctrl_init(ak4375_sys_data);
    if (rc < 0) {
        b_codec_err("get pinctl init error %d\n", rc);
    }

    b_codec_trace();


    /*
     *
     * */
    ak4375_config_pins(ak4375_sys_data);
    ak4375_ldo_power_on(0);
    ak4375_dac_on(0);

    b_codec_trace();


    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));
    dev_set_name(&i2c->dev, "%s", "akm-akm4375");
    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));

    /*
     * register sound
     * */
    snd_soc_codec_set_drvdata(codec, ak4375_data);

    rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ak4375, &ak4375_dai[0], ARRAY_SIZE(ak4375_dai));
    if (rc < 0){
        kfree(ak4375_data);
        kfree(ak4375_sys_data);
        b_codec_err("snd_soc_register_codec eror %d\n", rc);
    }

    b_codec_trace();

    /*
     * register debug interface for cmdline mode
     * */
    rc = device_create_file(&(i2c->dev), &dev_attr_ak4375_control);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    b_codec_trace();

    return rc;
}

static int ak4375_i2c_remove(struct i2c_client *client)
{
    b_codec_err("%s(%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

static const struct i2c_device_id ak4375_i2c_id[] = {
    { "ak4375", 0 },
    { }
};

static struct of_device_id ak4375_match_table[] = {
    { .compatible = "akm,akm4375", },
    { },
};

MODULE_DEVICE_TABLE(i2c, ak4375_i2c_id);

static struct i2c_driver ak4375_i2c_driver = {
    .driver = {
        .name = "akm-akm4375",
        .owner = THIS_MODULE,
        .of_match_table = ak4375_match_table,
    },
    .probe = ak4375_i2c_probe,
    .remove = ak4375_i2c_remove,
    .id_table = ak4375_i2c_id,
};

module_i2c_driver(ak4375_i2c_driver);

MODULE_DESCRIPTION("IDOL3 ak4375 codec driver");
MODULE_LICENSE("GPL");
