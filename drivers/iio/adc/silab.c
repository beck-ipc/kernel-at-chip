/*
 * 
 * Copyright (C) 2018 Florian Boor <florian@kernelconcepts.de>
 *
 * Driver for Beck IPC Silab C8051 ADC with SPI interface
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

#define SILAB_MESSAGE_LENGTH	14
#define SILAB_INPUT_NUM		 4

/*
 * RANGES (needs checking)
 * 31 Pt1000 (-50 .. 150 C) (-5000 .. 15000) in 0.01
 * 32 voltage 0V .. 10 V (0 - 10000) in 0.001
 * 33 resistance 0 .. 1600 ohm (0 - 16000) in 1 ohm
 * 34 current 0 .. 20 mA (0 - 20000) in 0.001
 * 35 resistance 0 .. 5000 ohm (0 - 50000) in 1 ohm
 */

#define SILAB_VAL_TEMPERATURE	0x11
#define SILAB_VAL_VOLTAGE	0x22
#define SILAB_VAL_RESISTANCE1	0x33
#define SILAB_VAL_CURRENT	0x44
#define SILAB_VAL_RESISTANCE2	0x55

struct silab_data_spec {
	int type ;
	int min;
	int max;
};


enum {
	silab4chan,
};

struct silab_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int resolution;
};

struct silab {
	struct spi_device *spi;
	struct spi_message msg;
	struct spi_transfer transfer[2];
	u8 tx_buf[SILAB_MESSAGE_LENGTH] ____cacheline_aligned;
	u8 rx_buf[SILAB_MESSAGE_LENGTH];

	struct mutex lock;
	const struct silab_chip_info *chip_info;
	int gpio_reset;
};


static int silab_read_value(struct silab *adc, u8 channel, int datatype)
{
	int ret = -1;
	int retries = 10;

	memset(&adc->tx_buf, 0x12, sizeof(adc->tx_buf));
	memset(&adc->rx_buf, 0x00, sizeof(adc->rx_buf));

	/* read all channels */
	adc->tx_buf[0] = 0x00;
	adc->tx_buf[1] = datatype;
	adc->tx_buf[2] = datatype;
	adc->tx_buf[3] = datatype;
	adc->tx_buf[4] = datatype;


	while ((ret < 0) && retries--) {
		/* zero length dummy transfer to get CS active */
		ret = spi_sync_transfer(adc->spi, &adc->transfer[1], 1);
		/* wait for conroller to become ready */
		mdelay(5);
		/* read data */
		ret = spi_sync_transfer(adc->spi, &adc->transfer[0], 1);
	#ifdef DEBUG
		printk(KERN_DEBUG "write %x : %x %x %x %x , %x\n", adc->tx_buf[0], adc->tx_buf[1], adc->tx_buf[2], adc->tx_buf[3], adc->tx_buf[4], adc->tx_buf[5]);
		printk(KERN_DEBUG "read %x %x : %x %x : %x %x \n", adc->rx_buf[0], adc->rx_buf[1], adc->rx_buf[2], adc->rx_buf[3], adc->rx_buf[4], adc->rx_buf[5]);
		printk(KERN_DEBUG "read %x %x : %x %x : %x %x : %x %x\n", adc->rx_buf[6], adc->rx_buf[7], adc->rx_buf[8], adc->rx_buf[9], adc->rx_buf[10], adc->rx_buf[11], adc->rx_buf[12], adc->rx_buf[13]);
	#endif
		/* read success? -> calculate value */
		if (ret == 0) {
	                int idx;
		
			idx = ((SILAB_INPUT_NUM - 1 - channel) * 2) + 7;
			ret = (int)((adc->rx_buf[idx] << 8) | (adc->rx_buf[idx + 1]));

			/* sanity checks */
			if (adc->rx_buf[1] != 0xAA)
				ret = -EIO;

			if (adc->tx_buf[1] != adc->rx_buf[3])
				ret = -EIO;

			if (adc->tx_buf[2] != adc->rx_buf[4])
				ret = -EIO;
		}
	}

	return ret;
}

static int silab_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct silab *adc = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&adc->lock);

	if ((chan->channel < 0) || (chan->channel > 3))
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			ret = silab_read_value(adc, chan->channel, SILAB_VAL_VOLTAGE);
		break;
		case IIO_CURRENT:
			ret = silab_read_value(adc, chan->channel, SILAB_VAL_CURRENT);
		break;
		case IIO_RESISTANCE:
			ret = silab_read_value(adc, chan->channel, SILAB_VAL_RESISTANCE2);
		break;
		case IIO_TEMP:
			ret = silab_read_value(adc, chan->channel, SILAB_VAL_TEMPERATURE);
		break;
		default:
		break;
		}

		if (ret < 0)
			goto out;

		/* calculate tempterature */
		if (chan->type == IIO_TEMP)
			ret -= 5000;		

		*val = ret;
		ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		if ((chan->type == IIO_VOLTAGE) || (chan->type == IIO_CURRENT)) {
			*val = 1;
			ret = IIO_VAL_FRACTIONAL_LOG2;
		}
		*val2 = adc->chip_info->resolution;
		break;
	}

out:
	mutex_unlock(&adc->lock);

	return ret;
}

#define SILAB_VOLTAGE_CHANNEL(num)				\
	{							\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = (num),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

#define SILAB_CURRENT_CHANNEL(num)				\
	{							\
		.type = IIO_CURRENT,				\
		.indexed = 1,					\
		.channel = (num),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

#define SILAB_RESISTANCE_CHANNEL(num)				\
	{							\
		.type = IIO_RESISTANCE,				\
		.indexed = 1,					\
		.channel = (num),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

#define SILAB_TEMPERATURE_CHANNEL(num)				\
	{							\
		.type = IIO_TEMP,				\
		.indexed = 1,					\
		.channel = (num),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

static const struct iio_chan_spec silab_channels[] = {
	SILAB_VOLTAGE_CHANNEL(0),
	SILAB_VOLTAGE_CHANNEL(1),
	SILAB_VOLTAGE_CHANNEL(2),
	SILAB_VOLTAGE_CHANNEL(3),
	SILAB_CURRENT_CHANNEL(0),
	SILAB_CURRENT_CHANNEL(1),
	SILAB_CURRENT_CHANNEL(2),
	SILAB_CURRENT_CHANNEL(3),
	SILAB_RESISTANCE_CHANNEL(0),
	SILAB_RESISTANCE_CHANNEL(1),
	SILAB_RESISTANCE_CHANNEL(2),
	SILAB_RESISTANCE_CHANNEL(3),
	SILAB_TEMPERATURE_CHANNEL(0),
	SILAB_TEMPERATURE_CHANNEL(1),
	SILAB_TEMPERATURE_CHANNEL(2),
	SILAB_TEMPERATURE_CHANNEL(3),
};

static const struct iio_info silab_info = {
	.read_raw = silab_read_raw,
	.driver_module = THIS_MODULE,
};

static const struct silab_chip_info silab_chip_infos[] = {
	[silab4chan] = {
		.channels = silab_channels,
		.num_channels = ARRAY_SIZE(silab_channels),
		.resolution = 16
	},
};

static int silab_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct silab *adc;
	const struct silab_chip_info *chip_info;
	int ret;
	int gpio_reset;

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
	mdelay(10);
	gpio_set_value(gpio_reset, 1);
	/* wait a 1100ms here for device to boot... */
	mdelay(1100);

	/* Initialise SPI device */
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	adc->gpio_reset = gpio_reset;
	spi->mode = SPI_MODE_3;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &silab_info;

	chip_info = &silab_chip_infos[spi_get_device_id(spi)->driver_data];
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

	memset(&adc->tx_buf, 0x12, sizeof(adc->tx_buf));

	mdelay(600);

	/* check read, clear Silab buffer */
	ret = silab_read_value(adc, 0, SILAB_VAL_RESISTANCE1);
	ret += silab_read_value(adc, 1, SILAB_VAL_RESISTANCE1);
	ret += silab_read_value(adc, 2, SILAB_VAL_RESISTANCE1);
	ret += silab_read_value(adc, 3, SILAB_VAL_RESISTANCE1);

	dev_info(&spi->dev, "Silab ADC initialised\n");

	ret = iio_device_register(indio_dev);
	if (ret >= 0)
		return 0;

	return ret;
}

static int silab_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

#if defined(CONFIG_OF)
/* open list for future variants */
static const struct of_device_id silab_dt_ids[] = {
	/* NOTE: The use of compatibles with no vendor prefix is deprecated. */
	{
		.compatible = "silab",
		.data = &silab_chip_infos[silab4chan],
	}, {
	}
};
MODULE_DEVICE_TABLE(of, silab_dt_ids);
#endif

static const struct spi_device_id silab_id[] = {
	{ "silab4chan", silab4chan },
	{ }
};
MODULE_DEVICE_TABLE(spi, silab_id);

static struct spi_driver silab_driver = {
	.driver = {
		.name = "silab",
		.of_match_table = of_match_ptr(silab_dt_ids),
	},
	.probe = silab_probe,
	.remove = silab_remove,
	.id_table = silab_id,
};
module_spi_driver(silab_driver);

MODULE_AUTHOR("Florian Boor <florian@kernelconcepts.de");
MODULE_DESCRIPTION("Beck IPC Silab C8051 ADC");
MODULE_LICENSE("GPL v2");
