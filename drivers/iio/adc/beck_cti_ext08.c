/*
 * 
 * Copyright (C) 2018 Florian Boor <florian@kernelconcepts.de>
 * Copyright (C) 2018 Andre Pribil <a.pribil@beck-ipc.com>
 *
 * Driver for Beck IPC CTI EXT08 (ADC and DIG IN/OUT) with SPI interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define CTI_EXT08_MESSAGE_LENGTH			3
#define CTI_EXT08_INPUT_NUM					8

#define CTI_EXT08_SPI_CMD_WRITE				0x84
#define CTI_EXT08_SPI_CMD_READ				0x85

#define CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE	0x01
#define CTI_EXT08_ANALOG_MODE_VAL_CURRENT	0x00

/* EXT08 Registers */
#define CTI_EXT08_REG_VERSION				0x00
#define CTI_EXT08_REG_CONTROL				0x01
#define CTI_EXT08_REG_NO24V					0x02
#define CTI_EXT08_REG_DIGITAL_IN			0x03
#define CTI_EXT08_REG_DIGITAL_OUT			0x04
#define CTI_EXT08_REG_DIGITAL_OUT_STATUS	0x05
#define CTI_EXT08_REG_ANALOG_MODE			0x0F
#define CTI_EXT08_REG_ANALOG_IN0_LO			0x10
#define CTI_EXT08_REG_ANALOG_IN0_HI			0x11
#define CTI_EXT08_REG_HARDWARE_MODE			0x20

#define CTI_EXT08_FIRMWARE_VERSION			0x04
#define CTI_EXT08_HARDWARE_ID_VERSION		0x00



enum {
	cti_ext08_8ana_8dig,
};

struct cti_ext08_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int resolution;
};

struct cti_ext08 {
	struct spi_device *spi;
	struct spi_message msg;
	struct spi_transfer transfer[2];
	u8 tx_buf[CTI_EXT08_MESSAGE_LENGTH] ____cacheline_aligned;
	u8 rx_buf[CTI_EXT08_MESSAGE_LENGTH];

	struct mutex lock;
	const struct cti_ext08_chip_info *chip_info;
	int gpio_reset;
};


static int cti_ext08_read_register(struct cti_ext08 *adc, u8 regAdr, u8 * value)
{
	int ret = -1;
	int retries = 10;

	memset(&adc->tx_buf, 0x00, sizeof(adc->tx_buf));
	memset(&adc->rx_buf, 0x00, sizeof(adc->rx_buf));

	adc->tx_buf[0] = CTI_EXT08_SPI_CMD_READ;
	adc->tx_buf[1] = regAdr;

	while ((ret < 0) && retries--) {
		/* zero length dummy transfer to get CS active */
		ret = spi_sync_transfer(adc->spi, &adc->transfer[1], 1);
		/* wait for conroller to become ready */
		mdelay(5);
		/* read data */
		ret |= spi_sync_transfer(adc->spi, &adc->transfer[0], 1);
	#ifdef DEBUG
		printk(KERN_DEBUG "write %x %x %x\n", adc->tx_buf[0], adc->tx_buf[1], adc->tx_buf[2]);
		printk(KERN_DEBUG "read  %x %x %x\n", adc->rx_buf[0], adc->rx_buf[1], adc->rx_buf[2]);
	#endif
		/* read success? -> get value */
		if (ret == 0) 
			*value = adc->rx_buf[2];
	}

	return ret;
}


static int cti_ext08_write_register(struct cti_ext08 *adc, u8 regAdr, u8 value)
{
	int ret = -1;
	int retries = 10;

	memset(&adc->tx_buf, 0x00, sizeof(adc->tx_buf));
	memset(&adc->rx_buf, 0x00, sizeof(adc->rx_buf));

	adc->tx_buf[0] = CTI_EXT08_SPI_CMD_WRITE;
	adc->tx_buf[1] = regAdr;
	adc->tx_buf[2] = value;

	while ((ret < 0) && retries--) {
		/* zero length dummy transfer to get CS active */
		ret = spi_sync_transfer(adc->spi, &adc->transfer[1], 1);
		/* wait for conroller to become ready */
		mdelay(5);
		/* read data */
		ret |= spi_sync_transfer(adc->spi, &adc->transfer[0], 1);
	#ifdef DEBUG
		printk(KERN_DEBUG "write %x %x %x\n", adc->tx_buf[0], adc->tx_buf[1], adc->tx_buf[2]);
		printk(KERN_DEBUG "read  %x %x %x\n", adc->rx_buf[0], adc->rx_buf[1], adc->rx_buf[2]);
	#endif
	}

	return ret;
}


static int cti_ext08_read_analog_value(struct cti_ext08 *adc, u8 channel, int datatype)
{
	int ret;
	u8 valueLo, valueHi, mode;
	static int anlogMode[CTI_EXT08_INPUT_NUM] = { CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE,
												  CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE };

	/* Mode change? */
	if (anlogMode[channel] != datatype) {
		ret = cti_ext08_read_register(adc, CTI_EXT08_REG_ANALOG_MODE, &mode);
		if (ret)
			return ret;

		if (datatype == CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE) {
			mode |= (1 << channel);
		} else {
			mode &= ~(1 << channel);
		}

		ret = cti_ext08_write_register(adc, CTI_EXT08_REG_ANALOG_MODE, mode);
		if (ret)
			return ret;

		anlogMode[channel] = datatype;

		mdelay(100);
	}

	/* Read low byte */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_ANALOG_IN0_LO + channel*2, &valueLo);
	if (ret)
		return ret;

	/* Read high byte */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_ANALOG_IN0_HI + channel*2, &valueHi);
	if (ret)
		return ret;

	return (((valueHi & 0x7) << 8) + valueLo);
}


static int cti_ext08_read_digital_input(struct cti_ext08 *adc, u8 channel)
{
	int ret;
	u8 value;

	/* Read digital inputs */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_DIGITAL_IN, &value);
	if (ret)
		return ret;

	return (value & (1 << channel)) ? 1 : 0;
}


static int cti_ext08_read_digital_output(struct cti_ext08 *adc, u8 channel)
{
	int ret;
	u8 value;

	/* Read digital outputs status */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_DIGITAL_OUT, &value);
	if (ret)
		return ret;

	return (value & (1 << channel)) ? 1 : 0;
}


static int cti_ext08_write_digital_output(struct cti_ext08 *adc, u8 channel, u8 value)
{
	int ret;
	u8 portValue;

	/* Read digital outputs */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_DIGITAL_OUT, &portValue);
	if (ret)
		return ret;

	portValue &= ~(1 << channel);
	portValue |= (!!value << channel);

	/* Write digital outputs */
	ret = cti_ext08_write_register(adc, CTI_EXT08_REG_DIGITAL_OUT, portValue);
	
	return ret;
}

static int cti_ext08_read_raw(struct iio_dev *indio_dev,
							  struct iio_chan_spec const *chan, int *val,
							  int *val2, long mask)
{
	struct cti_ext08 *adc = iio_priv(indio_dev);
	int ret = -EINVAL;

	if ((chan->channel < 0) || (chan->channel > CTI_EXT08_INPUT_NUM-1))
		return -EINVAL;

	mutex_lock(&adc->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			ret = cti_ext08_read_analog_value(adc, chan->channel, CTI_EXT08_ANALOG_MODE_VAL_VOLTAGE);
		break;
		case IIO_CURRENT:
			ret = cti_ext08_read_analog_value(adc, chan->channel, CTI_EXT08_ANALOG_MODE_VAL_CURRENT);
		break;
		case IIO_DIGITAL:
			if (chan->output)
				ret = cti_ext08_read_digital_output(adc, chan->channel);
			else
				ret = cti_ext08_read_digital_input(adc, chan->channel);
		break;
		default:
		break;
		}

		if (ret < 0)
			goto out;

		*val = ret;
		ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			/* 10100mV, 11 bits */
			*val = 10100;
			*val2 = adc->chip_info->resolution;
			ret = IIO_VAL_FRACTIONAL_LOG2;
		}
		else if (chan->type == IIO_CURRENT) {
			/* 25mA, 11 bits */
			*val = 25;
			*val2 = adc->chip_info->resolution;
			ret = IIO_VAL_FRACTIONAL_LOG2;
		}
		break;
	}

out:
	mutex_unlock(&adc->lock);

	return ret;
}


static int cti_ext08_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int val, int val2,
				long mask)
{
	struct cti_ext08 *adc = iio_priv(indio_dev);
	int ret = -EINVAL;
	
	if (val != 0 && val != 1)
		return -EINVAL;

	mutex_lock(&adc->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_DIGITAL:
			if (chan->output)
				ret = cti_ext08_write_digital_output(adc, chan->channel, val);
		break;
		default:
		break;
		}
	}

	mutex_unlock(&adc->lock);

	return ret;
}


static int cti_ext08_get_no24v_state(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct cti_ext08 *adc = iio_priv(indio_dev);
	int ret;
	u8 value;

	mutex_lock(&adc->lock);

	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_NO24V, &value);
	if (ret < 0) {
		mutex_unlock(&adc->lock);
		return ret;
	}

	mutex_unlock(&adc->lock);

	return (value & 0x1);
}

static const char * const cti_ext08_no24v_states[] = { "Output power supplied", "Output power lost" };

static const struct iio_enum cti_ext08_no24v_state = {
	.items = cti_ext08_no24v_states,
	.num_items = ARRAY_SIZE(cti_ext08_no24v_states),
	.get = cti_ext08_get_no24v_state,
};

static const struct iio_chan_spec_ext_info cti_ext08_ext_info[] = {
	IIO_ENUM("supply_state", IIO_SHARED_BY_DIR, &cti_ext08_no24v_state),
	{},
};

#define CTI_EXT08_VOLTAGE_CHANNEL(num)							\
	{															\
		.type = IIO_VOLTAGE,									\
		.indexed = 1,											\
		.channel = (num),										\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE)	\
	}

#define CTI_EXT08_CURRENT_CHANNEL(num)							\
	{															\
		.type = IIO_CURRENT,									\
		.indexed = 1,											\
		.channel = (num),										\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	}

#define CTI_EXT08_DIGIN_CHANNEL(num)							\
	{															\
		.type = IIO_DIGITAL,									\
		.indexed = 1,											\
		.channel = (num),										\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
		.ext_info = cti_ext08_ext_info,							\
	}

#define CTI_EXT08_DIGOUT_CHANNEL(num)							\
	{															\
		.type = IIO_DIGITAL,									\
		.indexed = 1,											\
		.channel = (num),										\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
		.output = 1,											\
	}

static const struct iio_chan_spec cti_ext08_channels[] = {
	CTI_EXT08_VOLTAGE_CHANNEL(0),
	CTI_EXT08_VOLTAGE_CHANNEL(1),
	CTI_EXT08_VOLTAGE_CHANNEL(2),
	CTI_EXT08_VOLTAGE_CHANNEL(3),
	CTI_EXT08_VOLTAGE_CHANNEL(4),
	CTI_EXT08_VOLTAGE_CHANNEL(5),
	CTI_EXT08_VOLTAGE_CHANNEL(6),
	CTI_EXT08_VOLTAGE_CHANNEL(7),
	CTI_EXT08_CURRENT_CHANNEL(0),
	CTI_EXT08_CURRENT_CHANNEL(1),
	CTI_EXT08_CURRENT_CHANNEL(2),
	CTI_EXT08_CURRENT_CHANNEL(3),
	CTI_EXT08_CURRENT_CHANNEL(4),
	CTI_EXT08_CURRENT_CHANNEL(5),
	CTI_EXT08_CURRENT_CHANNEL(6),
	CTI_EXT08_CURRENT_CHANNEL(7),
	CTI_EXT08_DIGIN_CHANNEL(0),
	CTI_EXT08_DIGIN_CHANNEL(1),
	CTI_EXT08_DIGIN_CHANNEL(2),
	CTI_EXT08_DIGIN_CHANNEL(3),
	CTI_EXT08_DIGIN_CHANNEL(4),
	CTI_EXT08_DIGIN_CHANNEL(5),
	CTI_EXT08_DIGIN_CHANNEL(6),
	CTI_EXT08_DIGIN_CHANNEL(7),
	CTI_EXT08_DIGOUT_CHANNEL(0),
	CTI_EXT08_DIGOUT_CHANNEL(1),
	CTI_EXT08_DIGOUT_CHANNEL(2),
	CTI_EXT08_DIGOUT_CHANNEL(3),
	CTI_EXT08_DIGOUT_CHANNEL(4),
	CTI_EXT08_DIGOUT_CHANNEL(5),
	CTI_EXT08_DIGOUT_CHANNEL(6),
	CTI_EXT08_DIGOUT_CHANNEL(7),
};

static const struct iio_info cti_ext08_info = {
	.read_raw = cti_ext08_read_raw,
	.write_raw = cti_ext08_write_raw,
	.driver_module = THIS_MODULE,
};

static const struct cti_ext08_chip_info cti_ext08_chip_infos[] = {
	[cti_ext08_8ana_8dig] = {
		.channels = cti_ext08_channels,
		.num_channels = ARRAY_SIZE(cti_ext08_channels),
		.resolution = 11
	},
};

static int cti_ext08_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct cti_ext08 *adc;
	const struct cti_ext08_chip_info *chip_info;
	int ret;
	int gpio_reset;
	u8 version;

	/* Get and set reset GPIO */
	gpio_reset = of_get_named_gpio(spi->dev.of_node, "gpio-reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(&spi->dev, "Failed to get reset GPIO from device tree\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(&spi->dev, gpio_reset, GPIOF_OUT_INIT_LOW, "gpio-reset");
	if (ret) {
		dev_err(&spi->dev, "Failed to request GPIO %d: %d\n", gpio_reset, ret);
		return -EINVAL;
	}
	mdelay(100);
	gpio_set_value(gpio_reset, 1);
	/* wait a 500ms here for device to boot... */
	mdelay(500);

	/* Initialise SPI device */
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	adc->gpio_reset = gpio_reset;
	spi->mode = SPI_MODE_0;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &cti_ext08_info;

	chip_info = &cti_ext08_chip_infos[spi_get_device_id(spi)->driver_data];
	indio_dev->channels = chip_info->channels;
	indio_dev->num_channels = chip_info->num_channels;

	adc->chip_info = chip_info;

	adc->transfer[0].tx_buf = &adc->tx_buf;
	adc->transfer[0].len = sizeof(adc->tx_buf);
	adc->transfer[0].rx_buf = adc->rx_buf;
	adc->transfer[1].len = 0;
	adc->transfer[1].cs_change = 1;

	spi_setup(spi);
	mutex_init(&adc->lock);

	/* check hardware version */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_HARDWARE_MODE, &version);
	if (ret < 0) {
		dev_err(&spi->dev, "Could not read Hardware version.");
		return -EINVAL;
	}
	if (version != CTI_EXT08_HARDWARE_ID_VERSION) {
		dev_err(&spi->dev, "Wrong hardware version detected.");
		return -EINVAL;
	}

	/* check firmware version */
	ret = cti_ext08_read_register(adc, CTI_EXT08_REG_VERSION, &version);
	if (ret < 0) {
		dev_err(&spi->dev, "Could not read Firmware version.");
		return -EINVAL;
	}
	if (version < CTI_EXT08_FIRMWARE_VERSION) {
		dev_err(&spi->dev, "Firmware version is not supported.");
		return -EINVAL;
	}

	dev_info(&spi->dev, "CTI EXT08 ADC initialised\n");

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int cti_ext08_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

#if defined(CONFIG_OF)
/* open list for future variants */
static const struct of_device_id cti_ext08_dt_ids[] = {
	/* NOTE: The use of compatibles with no vendor prefix is deprecated. */
	{
		.compatible = "cti_ext08",
		.data = &cti_ext08_chip_infos[cti_ext08_8ana_8dig],
	}, 
	{ }
};
MODULE_DEVICE_TABLE(of, cti_ext08_dt_ids);
#endif

static const struct spi_device_id cti_ext08_id[] = {
	{ "cti_ext08_8ana_8dig", cti_ext08_8ana_8dig },
	{ }
};
MODULE_DEVICE_TABLE(spi, cti_ext08_id);

static struct spi_driver cti_ext08_driver = {
	.driver = {
		.name = "cti_ext08",
		.of_match_table = of_match_ptr(cti_ext08_dt_ids),
	},
	.probe = cti_ext08_probe,
	.remove = cti_ext08_remove,
	.id_table = cti_ext08_id,
};
module_spi_driver(cti_ext08_driver);

MODULE_AUTHOR("Andre Pribil <a.pribil@beck-ipc.com");
MODULE_DESCRIPTION("Beck IPC CTI EXT08");
MODULE_LICENSE("GPL v2");
