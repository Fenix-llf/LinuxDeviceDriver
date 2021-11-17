/*
 * Battery charger driver for TI bq24780s
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include <linux/power/bq24780s-charger.h>

/* enable to charge*/
#define BQ24780S_CHG_OPT0				0x12 		//reg
#define BQ24780S_CHG_OPT_CHARGE_DISABLE		(1 << 0)	//disable mask
#define BQ24780S_CHG_OPT0_WDTMR			0x6000

#define BQ24780S_CHG_OPT3				0x37
#define BQ24780S_CHG_OPT_AC_PRESENT			(1 << 11)	//adapet

#define BQ24780S_PROCHOT_OPT1			0x3D
#define BQ24780S_PROCHOT_OPT1_EVENT		0x7F

#define BQ24780S_PROCHOT_STAT			0x3A
#define BQ24780S_INDENT_MASK			(1 << 6)
#define BQ24780S_ICRIT_MASK				(1 << 5)
#define BQ24780S_INOM_MASK				(1 << 4)
#define BQ24780S_IDCHG_MASK				(1 << 3)
#define BQ24780S_VSYS_MASK				(1 << 2)
#define BQ24780S_BATPRES_MASK			(1 << 1)
#define BQ24780S_ACOK_MASK				(1 << 0)

/* battery changre current */
#define BQ24780S_CHARGE_CURRENT			0x14
#define BQ24780S_CHARGE_CURRENT_MASK			0x1fc0

/* battery changre voltage */
#define BQ24780S_CHARGE_VOLTAGE			0x15
#define BQ24780S_CHARGE_VOLTAGE_MASK			0x7ff0

/* adapet input current */
#define BQ24780S_INPUT_CURRENT			0x3f
#define BQ24780S_INPUT_CURRENT_MASK			0x1f80

/* battery dischangre current */
#define BQ24780S_DISCHARGE_CURRENT			0x39
#define BQ24780S_DISCHARGE_CURRENT_MASK 		0x7e00

/* MID & DID, fixed value */
#define BQ24780S_MANUFACTURER_ID			0xFE
#define BQ24780S_DEVICE_ID				0xFF

/* structure of bq24780s */
struct bq24780s {
	struct power_supply		*charger;	//power_supply structure
	struct power_supply_desc	charger_desc;	//description structure
	struct i2c_client		*client;	//i2c client contorller
	struct bq24780s_platform		*pdata;	//view at head file
	struct gpio_desc		 		*acok;
	struct gpio_desc		 		*prochot;
	struct mutex			lock;		//mutex_lock
	struct delayed_work		poll;		//poll work
	u32				poll_interval;		//poll time
	bool				charging;		//charging flag
};

/* power_supply to bq24780s */
static inline struct bq24780s *to_bq24780s(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

/* enable some property bit to used */
/* view at /include/linux/power_supply.h */
static enum power_supply_property bq24780s_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS, //Indicates that status is required
	POWER_SUPPLY_PROP_ONLINE, //Indicates that online is required
};

/* check  property is writeable */
static int bq24780s_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return 1;
	default:
		break;
	}

	return 0;
}

/* write the data of register*/
static inline int bq24780s_write_word(struct i2c_client *client, u8 reg,
				     u16 value)
{
	return i2c_smbus_write_word_data(client, reg, value);
}

/* read the data of register*/
static inline int bq24780s_read_word(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_word_data(client, reg);
}

/*
 update the data of register
 @client:i2c client
 @reg:target register
 @mask:the mask of data
 @value:data
 @retv:0 fot success,else for false
*/
static int bq24780s_update_word(struct i2c_client *client, u8 reg,
			       u16 mask, u16 value)
{
	unsigned int tmp;
	int ret;

	ret = bq24780s_read_word(client, reg);
	if (ret < 0)
		return ret;

	tmp = ret & ~mask;
	tmp |= value & mask;

	return bq24780s_write_word(client, reg, tmp);
}



/* config charging */
static int bq24780s_config_charger(struct bq24780s *charger)
{
	struct bq24780s_platform *pdata = charger->pdata;
	int ret;
	u16 value = 0;

	//config  current of charging
	if (pdata->charge_current) {
		value = pdata->charge_current & BQ24780S_CHARGE_CURRENT_MASK;

		ret = bq24780s_write_word(charger->client,
					 BQ24780S_CHARGE_CURRENT, value);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"Failed to write charger current : %d\n",
				ret);
			return ret;
		}
	}

	//config  voltage of charging
	if (pdata->charge_voltage) {
		value = pdata->charge_voltage & BQ24780S_CHARGE_VOLTAGE_MASK;

		ret = bq24780s_write_word(charger->client,
					 BQ24780S_CHARGE_VOLTAGE, value);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"Failed to write charger voltage : %d\n",
				ret);
			return ret;
		}
	}

	//config  input current of charging
	if (pdata->input_current) {
		value = pdata->input_current & BQ24780S_INPUT_CURRENT_MASK;

		ret = bq24780s_write_word(charger->client,
					 BQ24780S_INPUT_CURRENT, value);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"Failed to write input current : %d\n",
				ret);
			return ret;
		}
	}

	//config  discharge current of charging
	if (pdata->discharge_current) {
		value = pdata->discharge_current & BQ24780S_DISCHARGE_CURRENT_MASK;

		ret = bq24780s_write_word(charger->client,
					 BQ24780S_DISCHARGE_CURRENT, value);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"Failed to write discharger current : %d\n",
				ret);
			return ret;
		}
	}


	return 0;
}

/* Enable charging */
static inline int bq24780s_enable_charging(struct bq24780s *charger)
{
	int ret;

	ret = bq24780s_config_charger(charger);
	if (ret)
		return ret;

	/* config register to enable charging(bit set 0) */
	return bq24780s_update_word(charger->client, BQ24780S_CHG_OPT0,
				   BQ24780S_CHG_OPT_CHARGE_DISABLE, 0);
}

/* Disable charging */
static inline int bq24780s_disable_charging(struct bq24780s *charger)
{
	/* config register to disable charging(bit set 1) */
	return bq24780s_update_word(charger->client, BQ24780S_CHG_OPT0,
				   BQ24780S_CHG_OPT_CHARGE_DISABLE,
				   BQ24780S_CHG_OPT_CHARGE_DISABLE);
}

/* check if adapet is present */
static bool bq24780s_acok_is_present(struct bq24780s *charger)
{
	/* need to edit!!!! */
	if (charger->acok) {
		return gpiod_get_value_cansleep(charger->acok);
	} else {
		int ac = 0;

		ac = bq24780s_read_word(charger->client, BQ24780S_CHG_OPT3);
		if (ac < 0) {
			dev_dbg(&charger->client->dev,
				"Failed to read charger options : %d\n", ac);
			return false;
		}
		return (ac & BQ24780S_CHG_OPT_AC_PRESENT) ? true : false;
	}
}

/* 
check if is charging
@retv:0 means no charging
	  1 means charging
 */
static int bq24780s_charger_is_charging(struct bq24780s *charger)
{
	int ret;

	if (!bq24780s_acok_is_present(charger))
		return 0;

	ret  = bq24780s_read_word(charger->client, BQ24780S_CHG_OPT0);
	if (ret < 0)
		return ret;

	return !(ret & BQ24780S_CHG_OPT_CHARGE_DISABLE);
}

/* 
update  charging status
if is charging , switch to no charging
if is no charging , switch to charging
 */
static void bq24780s_update(struct bq24780s *charger)
{
	mutex_lock(&charger->lock);
	if (charger->charging && bq24780s_acok_is_present(charger)){
		bq24780s_enable_charging(charger);
	}else{
		bq24780s_disable_charging(charger);
	}
	mutex_unlock(&charger->lock);

	power_supply_changed(charger->charger);
}

/* charge interupt handler */
static irqreturn_t bq24780s_charger_acok_isr(int irq, void *devid)
{
	struct power_supply *psy = devid;
	struct bq24780s *charger = to_bq24780s(psy);
	/* swtich charging status ,need to edit ??? */
	bq24780s_update(charger);

	return IRQ_HANDLED;
}


static void bq24780s_poll(struct work_struct *work)
{
	struct bq24780s *dev = container_of(work, struct bq24780s, poll.work);

	bq24780s_update(dev);

	schedule_delayed_work(&dev->poll, 
							msecs_to_jiffies(dev->poll_interval));
}

/*
get a property
@psy:power_supply structure
@psp:power_supply property structure
@val: a union to express POWER_SUPPLY_STATUS
*/
static int bq24780s_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24780s *charger = to_bq24780s(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24780s_acok_is_present(charger) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		switch (bq24780s_charger_is_charging(charger)) {
		case 1:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 0:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
set a property,use to switch charge status
@psy:power_supply structure
@psp:power_supply property structure
@val: a union to express POWER_SUPPLY_STATUS
*/
static int bq24780s_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24780s *charger = to_bq24780s(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_CHARGING:
			mutex_lock(&charger->lock);
			charger->charging = true;
			ret = bq24780s_enable_charging(charger);
			mutex_unlock(&charger->lock);
			if (ret)
				return ret;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			mutex_lock(&charger->lock);
			charger->charging = false;
			ret = bq24780s_disable_charging(charger);
			mutex_unlock(&charger->lock);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}
		power_supply_changed(psy);
		break;
	default:
		return -EPERM;
	}

	return 0;
}

/* 
Parse data of device-tree`s node
@retv:a bq24780s_platform pointer for bq24780s structure 
*/
static struct bq24780s_platform *bq24780s_parse_dt_data(struct i2c_client *client)
{
	struct bq24780s_platform *pdata;
	struct device_node *np = client->dev.of_node;
	u32 val;
	int ret;

	/* request a bq24780s_platform space */
	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev,
			"Memory alloc for bq24780s pdata failed\n");
		return NULL;
	}
	/* get configurations from device-tree */
	ret = of_property_read_u32(np, "charge-current", &val);
	if (!ret)
		pdata->charge_current = val;

	ret = of_property_read_u32(np, "charge-voltage", &val);
	if (!ret)
		pdata->charge_voltage = val;

	ret = of_property_read_u32(np, "input-current", &val);
	if (!ret)
		pdata->input_current = val;

	ret = of_property_read_u32(np, "discharge-current", &val);
	if (!ret)
		pdata->discharge_current = val;

	return pdata;
}

/*probe function*/
static int bq24780s_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret;
	struct bq24780s *charger;
	struct power_supply_desc *supply_desc;
	struct power_supply_config psy_cfg = {};
	char *name;

	/* request space */
	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	/* init mutex_lock */
	mutex_init(&charger->lock);
	charger->charging = true;	/* why??? */
	charger->pdata = client->dev.platform_data;  /*get platform data*/

	/* get device-tree data */
	if (IS_ENABLED(CONFIG_OF) && !charger->pdata && client->dev.of_node)
		charger->pdata = bq24780s_parse_dt_data(client);

	if (!charger->pdata) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}

	/*get devcie name */
	name = (char *)charger->pdata->name;
	if (!name) {
		name = devm_kasprintf(&client->dev, GFP_KERNEL,
				      "bq24780s@%s",
				      dev_name(&client->dev));
		if (!name) {
			dev_err(&client->dev, "Failed to alloc device name\n");
			return -ENOMEM;
		}
	}

	charger->client = client;

	/* config supply_desc structure */
	supply_desc = &charger->charger_desc;

	supply_desc->name = name;
	supply_desc->type = POWER_SUPPLY_TYPE_MAINS;
	supply_desc->properties = bq24780s_charger_properties;
	supply_desc->num_properties = ARRAY_SIZE(bq24780s_charger_properties);
	supply_desc->get_property = bq24780s_charger_get_property;
	supply_desc->set_property = bq24780s_charger_set_property;
	supply_desc->property_is_writeable =
				bq24780s_charger_property_is_writeable;

	/* config psy_cfg structure(power supply config) */
	psy_cfg.supplied_to = charger->pdata->supplied_to;
	psy_cfg.num_supplicants = charger->pdata->num_supplicants;
	psy_cfg.of_node = client->dev.of_node;
	psy_cfg.drv_data = charger;

	i2c_set_clientdata(client, charger);

	/* get gpio desc */
	charger->acok = devm_gpiod_get_optional(&client->dev,
						       "gpio-acok",
						       GPIOD_IN);
	if (IS_ERR(charger->acok)) {
		ret = PTR_ERR(charger->acok);
		dev_err(&client->dev, "Getting acok gpio failed: %d\n", ret);
		return ret;
	}

	charger->prochot = devm_gpiod_get_optional(&client->dev,
						       "gpio-prochot",
						       GPIOD_IN);
	if (IS_ERR(charger->prochot)) {
		ret = PTR_ERR(charger->prochot);
		dev_err(&client->dev, "Getting prochot gpio failed: %d\n", ret);
		return ret;
	}

	/* check if adapet  is present and get MID & DID */
	if (bq24780s_acok_is_present(charger)) {
		ret = bq24780s_read_word(client, BQ24780S_MANUFACTURER_ID);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to read manufacturer id : %d\n",
				ret);
			return ret;
		} else if (ret != 0x0040) {
			dev_err(&client->dev,
				"manufacturer id mismatch. 0x0040 != 0x%04x\n", ret);
			return -ENODEV;
		}
		pr_info("manufacturer id = 0x%04x\n", ret);

		ret = bq24780s_read_word(client, BQ24780S_DEVICE_ID);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to read device id : %d\n", ret);
			return ret;
		} else if (ret != 0x0030) {
			dev_err(&client->dev,
				"device id mismatch. 0x0030 != 0x%04x\n", ret);
			return -ENODEV;
		}
		pr_info("device id = 0x%04x\n", ret);

		/* enable charging */
		ret = bq24780s_enable_charging(charger);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to enable charging\n");
			return ret;
		}
	}

	
	charger->charger = devm_power_supply_register(&client->dev, supply_desc,
						      &psy_cfg);
	if (IS_ERR(charger->charger)) {
		ret = PTR_ERR(charger->charger);
		dev_err(&client->dev, "Failed to register power supply: %d\n",
			ret);
		return ret;
	}

	bq24780s_update_word(client, BQ24780S_PROCHOT_OPT1, 
						BQ24780S_PROCHOT_OPT1_EVENT, 0X7F);

	bq24780s_update_word(client, BQ24780S_CHG_OPT0, 
						BQ24780S_CHG_OPT0_WDTMR, 0x0);

	/*
	if client have a irq, define a irq,else,using a poll work
	*/
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, bq24780s_charger_acok_isr,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						supply_desc->name,
						charger->charger);
		if (ret) {
			dev_err(&client->dev,
				"Unable to register IRQ %d err %d\n",
				client->irq, ret);
			return ret;
		}
	}

#if 0
	ret = device_property_read_u32(&client->dev, "poll-interval", &charger->poll_interval);
	if (ret)
		return 0;
	if (!charger->poll_interval)
		return 0;

	INIT_DELAYED_WORK(&charger->poll, bq24780s_poll);
	schedule_delayed_work(&charger->poll, 
							msecs_to_jiffies(charger->poll_interval));
#endif 
	return 0;
}

/*remove function*/
static int bq24780s_charger_remove(struct i2c_client *client)
{
	struct bq24780s *charger = i2c_get_clientdata(client);

	bq24780s_disable_charging(charger);

	if (charger->poll_interval)
		cancel_delayed_work_sync(&charger->poll);

	return 0;
}


/* Traditional type match device id */
static const struct i2c_device_id bq24780s_charger_id[] = {
	{ "ti,bq24780s", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq24780s_charger_id);


/* of match table */
static const struct of_device_id bq24780s_match_ids[] = {
	{ .compatible = "ti,bq24780s", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, bq24780s_match_ids);


/* i2c platform driver */
static struct i2c_driver bq24780s_charger_driver = {
	.driver = {
		.name = "bq24780s",
		.of_match_table = bq24780s_match_ids,
	},
	.probe = bq24780s_charger_probe,
	.remove = bq24780s_charger_remove,
	.id_table = bq24780s_charger_id,
};

module_i2c_driver(bq24780s_charger_driver);

MODULE_DESCRIPTION("bq24780s battery charging driver");
MODULE_AUTHOR("Fenix Lee <leelinfae@163.com>");
MODULE_LICENSE("GPL");
