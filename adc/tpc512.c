#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/gpio.h>

#define TPC512_MANUAL_MODE_CTRL_REG       0x1
#define TPC512_AUTO1_MODE_CTRL_REG        0x2
#define TPC512_AUTO2_MODE_CTRL_REG        0X3
#define TPC512_GPIO_PROGRAM_REG           0X4
#define TPC512_AUTO1_MODE_PROGRAM_REG     0x8
#define TPC512_AUTO2_MODE_PROGRAM_REG     0x9
#define TPC512_ALARM_GRP0_REG             0xC
#define TPC512_ALARM_GRP1_REG             0xD
#define TPC512_ALARM_GRP2_REG             0xE
#define TPC512_ALARM_GRP3_REG             0XF

#define TPC512_MAX_CHAN                 8
#define MANUAL_ADDRESS_SHIFT            7
#define TPC512_REG_SHIFT                12
#define TPC512_MANUAL_CH_ADDR(ch)       ((ch) << MANUAL_ADDRESS_SHIFT)
#define TPC512_REG(reg)                 ((reg) << TPC512_REG_SHIFT)
#define TPC512_MANUAL_CMD(ch)           ((TPC512_REG(TPC512_MANUAL_MODE_CTRL_REG))) | (TPC512_MANUAL_CH_ADDR(ch))  
#define TPC512_DATA_MASK                0x0FFF

enum {
    tpc5120,
    tpc5121,
};

struct tpc512_dev {
    struct spi_device *spi;
    struct regulator *reg;
    struct mutex lock;

    u8 tx_buf[2] ____cacheline_aligned;
    u8 rx_buf[2];
};


#define TPC512_VOLATAGE_CHANNEL(chan) \
    {								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = chan,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = chan,					\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 16,					\
			.storagebits = 16,				\
		},							\
	}


static const struct iio_chan_spec tpc5120_channels[] = {
    TPC512_VOLATAGE_CHANNEL(0),
    TPC512_VOLATAGE_CHANNEL(1),
    TPC512_VOLATAGE_CHANNEL(2),
    TPC512_VOLATAGE_CHANNEL(3),
    TPC512_VOLATAGE_CHANNEL(4),
    TPC512_VOLATAGE_CHANNEL(5),
    TPC512_VOLATAGE_CHANNEL(6),
    TPC512_VOLATAGE_CHANNEL(7),
    TPC512_VOLATAGE_CHANNEL(8),
    TPC512_VOLATAGE_CHANNEL(9),
    TPC512_VOLATAGE_CHANNEL(10),
    TPC512_VOLATAGE_CHANNEL(11),
    TPC512_VOLATAGE_CHANNEL(12),
    TPC512_VOLATAGE_CHANNEL(13),
    TPC512_VOLATAGE_CHANNEL(14),
    TPC512_VOLATAGE_CHANNEL(15),
    IIO_CHAN_SOFT_TIMESTAMP(16),
};

static const struct iio_chan_spec tpc5121_channels[] = {
    TPC512_VOLATAGE_CHANNEL(0),
    TPC512_VOLATAGE_CHANNEL(1),
    TPC512_VOLATAGE_CHANNEL(2),
    TPC512_VOLATAGE_CHANNEL(3),
    TPC512_VOLATAGE_CHANNEL(4),
    TPC512_VOLATAGE_CHANNEL(5),
    TPC512_VOLATAGE_CHANNEL(6),
    TPC512_VOLATAGE_CHANNEL(7),
    IIO_CHAN_SOFT_TIMESTAMP(8),
};

static int tpc512_adc_conversion(struct tpc512_dev *adc, int chan)
{
    int ret;
    u8 tx_data[2] = {0};
    struct spi_message msg;
    struct spi_transfer *transfer;
    struct spi_device *spi = (struct spi_device *)adc->spi;
    transfer = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

	u8 high = ((u8)chan & 0xE) >> 1;
    u8 low = ((u8)chan & 0x1);

    gpio_set_value(spi->cs_gpio, 0);
    tx_data[0] = 0x10 | high;
    tx_data[1] =  low << 7; 
    transfer->tx_buf = tx_data;
    transfer->len = 2;
    spi_message_init(&msg);
    spi_message_add_tail(transfer, &msg);
    ret = spi_sync(spi, &msg);   
    gpio_set_value(spi->cs_gpio, 1);


    gpio_set_value(spi->cs_gpio, 0);
    tx_data[0] = 0x00;
    tx_data[1] = 0x00; 
    transfer->tx_buf = tx_data;
    transfer->len = 2;
    spi_message_init(&msg);
    spi_message_add_tail(transfer, &msg);
    ret = spi_sync(spi, &msg);   
    gpio_set_value(spi->cs_gpio, 1);

    gpio_set_value(spi->cs_gpio, 0);
    transfer->rx_buf = adc->rx_buf;
    transfer->len = 2;
    spi_message_init(&msg);
    spi_message_add_tail(transfer, &msg);
    ret = spi_sync(spi, &msg);   
    
    gpio_set_value(spi->cs_gpio, 1);
    kfree(transfer);

	return (adc->rx_buf[0] << 8 | adc->rx_buf[1]) & 0xFFF;
}


static int tpc512_read_raw(struct iio_dev *iio,
			struct iio_chan_spec const *channel, int *value,
			int *shift, long mask)
{
    struct tpc512_dev *adc = iio_priv(iio);

    switch (mask)
    {
    case IIO_CHAN_INFO_RAW:
        mutex_lock(&adc->lock);
        *value = tpc512_adc_conversion(adc, channel->channel);
        mutex_unlock(&adc->lock);
        if(*value < 0)
            return (*value);

        return IIO_VAL_INT;

    case IIO_CHAN_INFO_SCALE:
        mutex_lock(&adc->lock);
        *value = regulator_get_voltage(adc->reg);
        mutex_unlock(&adc->lock);
        if(*value < 0)
            return *value;

        *value /= 1000;
        *shift = 16;
        return IIO_VAL_FRACTIONAL_LOG2;
    }

    return -EINVAL;
}

static const struct iio_info tpc512_info = {
    .driver_module = THIS_MODULE,
    .read_raw = tpc512_read_raw,
};

static irqreturn_t tpc512_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct tpc512_dev *adc = iio_priv(indio_dev);

    u16 data[16] = {0};
    int scan_index;
    int i = 0;
    
    mutex_lock(&adc->lock);
    for_each_set_bit(scan_index, 
                        indio_dev->active_scan_mask, 
                        indio_dev->masklength)
    {
        const struct iio_chan_spec *scan_chan = &indio_dev->channels[scan_index];
        int ret =  tpc512_adc_conversion(adc, scan_chan->channel);
        if(ret < 0){
            dev_err(&adc->spi->dev, "failed to get conversion data\n");
            goto out;
        }
        data[i] = ret;
        i++;
    }
    iio_push_to_buffers_with_timestamp(indio_dev, data, iio_get_time_ns(indio_dev));
out:
    mutex_unlock(&adc->lock);
    iio_trigger_notify_done(indio_dev->trig);
    return IRQ_HANDLED;
}


static int tpc512_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct tpc512_dev *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	mutex_init(&adc->lock);

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->info = &tpc512_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	switch (spi_get_device_id(spi)->driver_data) {
	case tpc5120:
		indio_dev->channels = tpc5120_channels;
		indio_dev->num_channels = ARRAY_SIZE(tpc5120_channels);
		break;
	case tpc5121:
		indio_dev->channels = tpc5121_channels;
		indio_dev->num_channels = ARRAY_SIZE(tpc5121_channels);
		break;
	default:
		return -EINVAL;
	}

	adc->reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->reg))
		return PTR_ERR(adc->reg);

    ret = regulator_enable(adc->reg);
	if (ret)
		return ret;

	spi_set_drvdata(spi, indio_dev);

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 tpc512_trigger_handler, NULL);
	if (ret)
		goto err_reg_disable;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_buffer_cleanup;

	return 0;
err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
err_reg_disable:
	regulator_disable(adc->reg);

	return ret;
}

static int tpc512_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct tpc512_dev *adc = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(adc->reg);

	return 0;
}

#ifdef CONFIG_OF

static const struct of_device_id tpc512_dt_ids[] = {
	{ .compatible = "3peak,tpc5120", },
	{ .compatible = "3peak,tpc5121", },
	{}
};

MODULE_DEVICE_TABLE(of, tpc512_dt_ids);

#endif

static const struct spi_device_id tpc512_id[] = {
	{ "tpc5120", tpc5120 },
	{ "tpc5121", tpc5121 },
	{}
};
MODULE_DEVICE_TABLE(spi, tpc512_id);

static struct spi_driver tpc512_driver = {
	.driver = {
		.name = "tpc512",
		.of_match_table = of_match_ptr(tpc512_dt_ids),
	},
	.probe = tpc512_probe,
	.remove = tpc512_remove,
	.id_table = tpc512_id,
};
module_spi_driver(tpc512_driver);

MODULE_AUTHOR("Fenix Lee <leelinfae@163.com>");
MODULE_DESCRIPTION("TPC5120/TPC5121 driver");
MODULE_LICENSE("GPL v2");