#include <sound/q6afe-v2.h>
unsigned int ak4375_i2c_raw_read(struct i2c_client *client, unsigned int reg)
{

	int ret;
	ret = i2c_smbus_read_byte_data(client, (u8)(reg & 0xFF));
	if (ret < 0) {
		b_codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int ak4375_i2c_raw_write(struct i2c_client *client, unsigned int reg, unsigned int value)
{
	b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

	if(i2c_smbus_write_byte_data(client, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		b_codec_dbg("%s(%d) error\n",__FUNCTION__,__LINE__);
		return EIO;
	}

    return 0;
}

static int ak4375_i2c_raw_write_mask(struct i2c_client *client, u32 reg, u32 mask, u32 value)
{
    int ret = 0;
    u32 old;
    u32 new;

    old = ak4375_i2c_raw_read(client, reg);
    if (old < 0) {
        b_codec_dbg("mask read %s(%d) error\n", __func__, __LINE__);
    }

    new = (old & ~(mask)) | value;

    ret = ak4375_i2c_raw_write(client, reg, new);
    if (ret < 0) {
        b_codec_dbg("mask write %s(%d) error\n", __func__, __LINE__);
    }

    return ret;
}

#if 0
static int dump_lpa(void)
{
    void __iomem *vaddr = NULL;

	vaddr = ioremap(0x07709000, 4);
	if (!vaddr) {
		return -ENOMEM;
	}
	printk("%s:0x07709000 %x\n", __func__,  ioread32(vaddr));
	iounmap(vaddr);

	vaddr = ioremap(0x0770a000, 4);
	if (!vaddr) {
		return -ENOMEM;
	}
	printk("%s:0x0770a000 %x\n", __func__,  ioread32(vaddr));
	iounmap(vaddr);

	vaddr = ioremap(0x0770b000, 4);
	if (!vaddr) {
		return -ENOMEM;
	}
	printk("%s:0x0770b000 %x\n", __func__,  ioread32(vaddr));
	iounmap(vaddr);

	vaddr = ioremap(0x0770c000, 4);
	if (!vaddr) {
		return -ENOMEM;
	}
	printk("%s:0x0770c000 %x\n", __func__,  ioread32(vaddr));
	iounmap(vaddr);


    return 0;
}
#endif


void ak4375_cmdline_fast_test_idol347(void)
{

    // power on AVDD LVDD
    ak4375_ldo_power_on(1);
    mdelay(10);

    // PDN pin to high
    ak4375_dac_on(1);

    // 0x00 == 0x00 PLL stop pmosc stop
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_00_POWER_MANAGEMENT1, 0x00);

    ak4375_i2c_raw_write_mask(ak4375_sys_data->client, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

    // PRE POWER ON
    ak4375_i2c_raw_write_mask(ak4375_sys_data->client, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
    mdelay(6);															                        //wait 6ms
    udelay(500);														                        //wait 0.5ms
    ak4375_i2c_raw_write_mask(ak4375_sys_data->client, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
    mdelay(1);															                        //wait 1ms

    // AFT POWER ON
    ak4375_i2c_raw_write_mask(ak4375_sys_data->client, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
    mdelay(4);															                //wait 4ms
    udelay(500);														                //wait 0.5ms


    // LR amp power management
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_03_POWER_MANAGEMENT4, 0x03);

    // VDD hold setting QQQ
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_04_OUTPUT_MODE_SETTING, 0x00);

    //256fs, 48KHZ
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_05_CLOCK_MODE_SELECT, 0x0a);

    //Sharp Roll-Off Filter
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_06_DIGITAL_FILTER_SELECT, 0x00);

    //LR ch select
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_07_DAC_MONO_MIXING, 0x21);

    //FIXME
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_08_JITTER_CLEANER_SETTING1, 0x00);


    // 0x00 src alll set default
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_09_JITTER_CLEANER_SETTING2, 0x00);


    // DAC Input Data Select = SDATA not src
    // DAC Operation Clock  = SRCMCLK
    // Charge Pump Operation Clock = XCKCPSEL
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0A_JITTER_CLEANER_SETTING3, 0x00);


    //
    // AK4375_0B_LCH_OUTPUT_VOLUME = 0x19
    // AK4375_0C_RCH_OUTPUT_VOLUME = 0x19
    //
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0B_LCH_OUTPUT_VOLUME, 0x19);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0C_RCH_OUTPUT_VOLUME, 0x19);


    //
    // 0x0d = 0x75 0db  zero cross time
    //
#if 0
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0D_HP_VOLUME_CONTROL, 0x75);
#else
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0D_HP_VOLUME_CONTROL, 0x73);
#endif


    //
    // 0x0e == 0x00
    //
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x00);


    //
    // PLL
    //
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_0F_PLL_REF_CLK_DIVIDER1, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_10_PLL_REF_CLK_DIVIDER2, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_11_PLL_FB_CLK_DIVIDER1, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_12_PLL_FB_CLK_DIVIDER2, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_13_SRC_CLK_SOURCE, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_13_SRC_CLK_SOURCE, 0x00);
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_14_DAC_CLK_DIVIDER, 0x00);


    // I2S 16bit
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_15_AUDIO_IF_FORMAT, 0x01);


    // no desample
    ak4375_i2c_raw_write(ak4375_sys_data->client, AK4375_24_MODE_CONTROL, 0x00);
}

int hex2dec(u8 ch)
{
    if(ch >= '0' && ch <= '9')
        return ch - '0';
    else if(ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if(ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;

    return -1;
}

static ssize_t ak4375_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 i;

    for (i = 0; i < AK4375_MAX_REGISTERS; i++) {
        sprintf(buf, "%s[0x%02x] = 0x%02x\r\n", buf, i, ak4375_i2c_raw_read(ak4375_sys_data->client, i));
    }

    b_codec_dbg("i2c_device_name = %s\n", dev_name(dev));

    return sprintf(buf, "%sregister read done.\r\n", buf);
}



static ssize_t ak4375_control_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
    int ret = 0;
    u8 reg, value = 0;


    if (size != 4) {
        printk("echo -n 030f > ak4375_control\n");
        return -1;
    }

    ret = hex2dec(buf[0]);
    if (ret == -1) {
        printk("store error.\n");
        return -1;
    }
    reg = ret << 4;

    ret = hex2dec(buf[1]);
    if (ret == -1) {
        printk("store error.\n");
        return -1;
    }
    reg |= (ret & 0xf);

    ret = hex2dec(buf[2]);
    if (ret == -1) {
        printk("store error.\n");
        return -1;
    }
    value = ret << 4;

    ret = hex2dec(buf[3]);
    if (ret == -1) {
        printk("store error.\n");
        return -1;
    }
    value |= (ret & 0xf);


    /*
     *  Bruce hack cmd in here to ops gpio ldo or etc
     *  address from 0x25 is free using
     * */
    if (reg == 0xff) {
        ak4375_cmdline_fast_test_idol347();
    }

    if (reg == 0xfe) {
        ak4375_ldo_power_on(value);
        b_codec_dbg("ldo power %s\n", value?"on":"off");
    }

    if (reg == 0xfd) {
        ak4375_dac_on(value);
        b_codec_dbg("DAC %s\n", value?"on":"off");
    }

    if (reg == 0xfc) {
    }


    if (reg == 0xfa) {
        g_dump_trace = value;
    }

    /*
     * real reg
     * */
    if (reg >=0 && reg < AK4375_MAX_REGISTERS) {
        ak4375_i2c_raw_write(ak4375_sys_data->client, reg, value);
        printk("Set  : reg = 0x%02x, value = 0x%02x\n", reg, value);
        printk("Read : reg = 0x%02x, value = 0x%02x\n", reg, ak4375_i2c_raw_read(ak4375_sys_data->client, reg));
    }

    return size;
}


static DEVICE_ATTR(ak4375_control, 0666, ak4375_control_show, ak4375_control_store);
