/*
 * AD5337 I2C DAC driver
 * (c) kernel concepts GmbH 2018
 * Florian Boor <florian@kernelconcepts.de> 
 *
 * based on ad5446.c
 * AD5446 SPI DAC driver
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define MODE_PWRDWN_1k		0x1
#define MODE_PWRDWN_100k	0x2
#define MODE_PWRDWN_TRISTATE	0x3

#define ISR_nLDAC	(1 << 4)
#define ISR_nCLR	(1 << 5)
#define ISR_nPD0	(1 << 6)
#define ISR_nPD1	(1 << 7)

/**
 * struct ad5337_state - driver instance specific data
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref_mv:		actual reference voltage used
 */

struct ad5337_state {
	struct device		*dev;
	const struct ad5337_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned short			vref_mv;
	unsigned			cached_val[2];
	unsigned			pwr_down_mode[2];
	unsigned			pwr_down[2];
};

/**
 * struct ad5337_chip_info - chip specific information
 * @channel:		channel spec for the DAC
 * @ext_vref_mv:	AD5337: the external reference voltage
 * @write:		chip specific helper function to write to the register
 */

struct ad5337_chip_info {
	struct iio_chan_spec	channels[2];
	u16			ext_vref_mv;
	int			(*write)(struct ad5337_state *st, int channel, unsigned val);
};

static const char * const ad5337_powerdown_modes[] = {
	"1kohm_to_gnd", "100kohm_to_gnd", "three_state"
};

static int ad5337_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5337_state *st = iio_priv(indio_dev);

	if ((chan->channel < 0) || (chan->channel > 1))
		return -EINVAL;

	st->pwr_down_mode[chan->channel] = mode + 1;

	return 0;
}

static int ad5337_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad5337_state *st = iio_priv(indio_dev);

	if ((chan->channel < 0) || (chan->channel > 1))
		return -EINVAL;

	return st->pwr_down_mode[chan->channel] - 1;
}

static const struct iio_enum ad5337_powerdown_mode_enum = {
	.items = ad5337_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5337_powerdown_modes),
	.get = ad5337_get_powerdown_mode,
	.set = ad5337_set_powerdown_mode,
};

static ssize_t ad5337_read_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct ad5337_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->pwr_down[chan->channel]);
}

static ssize_t ad5337_write_dac_powerdown(struct iio_dev *indio_dev,
					    uintptr_t private,
					    const struct iio_chan_spec *chan,
					    const char *buf, size_t len)
{
	struct ad5337_state *st = iio_priv(indio_dev);
	bool powerdown;
	int ret;

	if ((chan->channel < 0) || (chan->channel > 1))
		return -EINVAL;

	ret = strtobool(buf, &powerdown);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	st->pwr_down[chan->channel] = powerdown;

	ret = st->chip_info->write(st, chan->channel, st->cached_val[chan->channel]);

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info ad5337_ext_info_powerdown[] = {
	{
		.name = "powerdown",
		.read = ad5337_read_dac_powerdown,
		.write = ad5337_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad5337_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &ad5337_powerdown_mode_enum),
	{ },
};

#define _AD5337_CHANNEL(num, bits, storage, _shift, ext) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = num, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = (bits), \
		.storagebits = (storage), \
		.shift = (_shift), \
		}, \
	.ext_info = (ext), \
}

#define AD5337_CHANNEL(num, bits, storage, shift) \
	_AD5337_CHANNEL(num, bits, storage, shift, ad5337_ext_info_powerdown)

static int ad5337_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5337_state *st = iio_priv(indio_dev);

	if ((chan->channel < 0) || (chan->channel > 1))
		return -EINVAL;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = st->cached_val[chan->channel];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int ad5337_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad5337_state *st = iio_priv(indio_dev);
	int ret = 0;

	if ((chan->channel < 0) || (chan->channel > 1))
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		val <<= chan->scan_type.shift;
		mutex_lock(&indio_dev->mlock);
		st->cached_val[chan->channel] = val;
		if (!st->pwr_down[chan->channel])
			ret = st->chip_info->write(st, chan->channel, val);
		mutex_unlock(&indio_dev->mlock);
		break;
	default:
		ret = -EINVAL;
	}

	return 0;
}

static int ad5337_write(struct ad5337_state *st, int channel, unsigned val)
{
	struct i2c_client *client = to_i2c_client(st->dev);
	/* 
           This one takes three bytes:
           Pointer byte: XX0000BA
	   Data MSB:     PPCLDDDD
           Data LSB:     DDDDXXXX
         */

	uint8_t data[3];
	int ret = 0;

	/* set pointer bits selecting channel */
	data[0] = channel ? (1 << 1) : (1 << 0);

	/* MSB data and flags */
	data[1] = (uint8_t)(val >> 4);
	data[1] |= ISR_nCLR;
	data[1] &= ~ISR_nLDAC;
	
	/* powerdown bits */
	if (st->pwr_down[channel])
		data[1] |= st->pwr_down_mode[channel] << 6;
	else
		data[1] &= ~(ISR_nPD0 | ISR_nPD1);

	/* LSB data */
	data[2] = (uint8_t)((val<<4) & 0xF0);

	ret = i2c_master_send(client, (char *)data, sizeof(data));	

	dev_dbg(st->dev, "DAC write %02x %02x %02x -> %d\n", data[0], data[1], data[2], ret);

	return ret;
}

static const struct iio_info ad5337_info = {
	.read_raw = ad5337_read_raw,
	.write_raw = ad5337_write_raw,
	.driver_module = THIS_MODULE,
};

static int ad5337_probe(struct device *dev, const char *name,
			const struct ad5337_chip_info *chip_info)
{
	struct ad5337_state *st;
	struct iio_dev *indio_dev;
	struct regulator *reg;
	int ret, voltage_uv = 0;

	reg = devm_regulator_get(dev, "vref");
	if (!IS_ERR(reg)) {
		ret = regulator_enable(reg);
		if (ret)
			return ret;

		ret = regulator_get_voltage(reg);
		if (ret < 0)
			goto error_disable_reg;

		voltage_uv = ret;
	}

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_disable_reg;
	}
	st = iio_priv(indio_dev);
	st->chip_info = chip_info;

	dev_set_drvdata(dev, indio_dev);
	st->reg = reg;
	st->dev = dev;

	/* Establish that the iio_dev is a child of the device */
	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &ad5337_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = 2;

	st->pwr_down_mode[0] = MODE_PWRDWN_1k;
	st->pwr_down_mode[1] = MODE_PWRDWN_1k;
	st->pwr_down[0] = 1;
	st->pwr_down[1] = 1;

	if (st->chip_info->ext_vref_mv)
		st->vref_mv = st->chip_info->ext_vref_mv;
	else if (voltage_uv)
		st->vref_mv = voltage_uv / 1000;
	else
		dev_warn(dev, "reference voltage unspecified\n");

	/* Check device communication, initialise state */
	ret = ad5337_write(st, 0, 0);
	if (ret < 0)
		goto error_disable_reg;
	ret = ad5337_write(st, 1, 0);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	dev_info(dev, "IIO: Registered Analog Devices AD533x DAC\n");

	return 0;

error_disable_reg:
	if (!IS_ERR(reg))
		regulator_disable(reg);
	return ret;
}

static int ad5337_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5337_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}


enum ad5337_supported_i2c_device_ids {
	ID_AD5337,
};

static const struct ad5337_chip_info ad5337_i2c_chip_info[] = {
	[ID_AD5337] = {
		.channels[0] = AD5337_CHANNEL(0, 24, 8, 0),
		.channels[1] = AD5337_CHANNEL(1, 24, 8, 0),
		.write = ad5337_write,
	},
};

static int ad5337_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	return ad5337_probe(&i2c->dev, id->name,
		&ad5337_i2c_chip_info[id->driver_data]);
}

static int ad5337_i2c_remove(struct i2c_client *i2c)
{
	return ad5337_remove(&i2c->dev);
}

static const struct i2c_device_id ad5337_i2c_ids[] = {
	{"ad5337", ID_AD5337},
	{}
};
MODULE_DEVICE_TABLE(i2c, ad5337_i2c_ids);

static struct i2c_driver ad5337_i2c_driver = {
	.driver = {
		   .name = "ad5337",
	},
	.probe = ad5337_i2c_probe,
	.remove = ad5337_i2c_remove,
	.id_table = ad5337_i2c_ids,
};

static int __init ad5337_i2c_register_driver(void)
{
	return i2c_add_driver(&ad5337_i2c_driver);
}

static void __exit ad5337_i2c_unregister_driver(void)
{
	i2c_del_driver(&ad5337_i2c_driver);
}

static int __init ad5337_init(void)
{
	return ad5337_i2c_register_driver();
}
module_init(ad5337_init);

static void __exit ad5337_exit(void)
{
	ad5337_i2c_unregister_driver();
}
module_exit(ad5337_exit);

MODULE_AUTHOR("Florian Boor <florian@kernelconcepts.de>");
MODULE_DESCRIPTION("Analog Devices AD533x DAC");
MODULE_LICENSE("GPL v2");

