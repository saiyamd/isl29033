/*
 *  isl29033.c - Linux kernel module for
 * 		Intersil ISL29033 ambient light sensor
 *
 *  Based on code written by, (for isl29003.c)
 *  	Daniel Mack <daniel@caiaq.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/errno.h>

#define ISL29033_DRV_NAME	"isl29033"
#define DRIVER_VERSION		"1.0"

#define ISL29033_REG_CMD1		0x00
#define ISL29033_REG_CMD2		0x01
#define ISL29033_REG_DATA_LSB		0x02
#define ISL29033_REG_DATA_MSB		0x03
#define ISL29033_REG_INT_LT_LSB		0x04
#define ISL29033_REG_INT_LT_MSB		0x05
#define ISL29033_REG_INT_HT_LSB		0x06
#define ISL29033_REG_INT_HT_MSB		0x07


#define ISL29033_ADC_ENABLED	(1 << 7)
#define ISL29033_ADC_PD		(1 << 6)
#define ISL29033_INT_FLG	(1 << 5)

#define ISL29033_RES_SHIFT	(2)
#define ISL29033_RES_MASK	(0x3 << ISL29033_RES_SHIFT)
#define	ISL29033_MODE_SHIFT	(5)
#define	ISL29033_MODE_MASK	(0x7 << ISL29033_MODE_SHIFT)
#define ISL29033_RANGE_SHIFT	(0)
#define ISL29033_RANGE_MASK	(0x3 << ISL29033_RANGE_SHIFT)
#define ISL29033_PERSISTS_SHIFT	(0)
#define ISL29033_PERSISTS_MASK	(0x3 << ISL29033_PERSISTS_SHIFT)

#define ISL29033_NUM_CACHABLE_REGS	4
#define	TH_MAX_VAL	65535
#define	TH_MIN_VAL	0

struct isl29033_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[ISL29033_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
};

static int gain_range[] = {
	125, 500, 2000, 8000
};

/*
 * register access helpers
 */

static int __isl29033_read_reg(struct i2c_client *client,
		u32 reg, u8 mask, u8 shift)
{
	struct isl29033_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __isl29033_write_reg(struct i2c_client *client,
		u32 reg, u8 mask, u8 shift, u8 val)
{
	struct isl29033_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= ISL29033_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;
	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int isl29033_get_range(struct i2c_client *client)
{
	return __isl29033_read_reg(client, ISL29033_REG_CMD2,
			ISL29033_RANGE_MASK, ISL29033_RANGE_SHIFT);
}

static int isl29033_set_range(struct i2c_client *client, int range)
{
	return __isl29033_write_reg(client, ISL29033_REG_CMD2,
			ISL29033_RANGE_MASK, ISL29033_RANGE_SHIFT, range);
}

/* resolution */
static int isl29033_get_resolution(struct i2c_client *client)
{
	return __isl29033_read_reg(client, ISL29033_REG_CMD2,
			ISL29033_RES_MASK, ISL29033_RES_SHIFT);
}

static int isl29033_set_resolution(struct i2c_client *client, int res)
{
	return __isl29033_write_reg(client, ISL29033_REG_CMD2,
			ISL29033_RES_MASK, ISL29033_RES_SHIFT, res);
}

static int isl29033_set_persist(struct i2c_client *client, int res)
{
	return __isl29033_write_reg(client, ISL29033_REG_CMD1,
			ISL29033_PERSISTS_MASK, ISL29033_PERSISTS_SHIFT, res);
}

/* mode */
static int isl29033_get_mode(struct i2c_client *client)
{
	return __isl29033_read_reg(client, ISL29033_REG_CMD1,
			ISL29033_RES_MASK, ISL29033_RES_SHIFT);
}

static int isl29033_set_mode(struct i2c_client *client, int mode)
{
	return __isl29033_write_reg(client, ISL29033_REG_CMD1,
			ISL29033_MODE_MASK, ISL29033_MODE_SHIFT, mode);
}

/* power_state */
static int isl29033_set_power_state(struct i2c_client *client, int state)
{
	return __isl29033_write_reg(client, ISL29033_REG_CMD1,
			ISL29033_ADC_ENABLED | ISL29033_ADC_PD, 0,
			state ? ISL29033_ADC_ENABLED : ISL29033_ADC_PD);
}

static int isl29033_get_power_state(struct i2c_client *client)
{
	//fix
	return 0x01;
}

/* HT */
static int isl29033_set_ht(struct i2c_client *client, uint16_t val)
{
	int8_t msb = ((val >> 8) & 0xff);
	int8_t lsb = ((val >> 0) & 0xff);
	int ret;

	ret = i2c_smbus_write_byte_data(client, ISL29033_REG_INT_HT_LSB, lsb);
	if(ret != 0) {
		printk("Failed to set ht value\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29033_REG_INT_HT_MSB, msb);
	if(ret != 0) {
		printk("Failed to set ht value\n");
		return ret;
	}

	return ret;
}

static int isl29033_get_ht(struct i2c_client *client)
{
	int msb, lsb;
	struct isl29033_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->lock);
	msb = i2c_smbus_read_byte_data(client, ISL29033_REG_INT_HT_MSB);
	if(msb < 0) {
		mutex_unlock(&data->lock);
		return msb;
	}

	lsb = i2c_smbus_read_byte_data(client, ISL29033_REG_INT_HT_LSB);
	if(lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	mutex_unlock(&data->lock);

	return (msb << 8) | lsb;
}

/* LT */
static int isl29033_set_lt(struct i2c_client *client, uint16_t val)
{
	int8_t msb = ((val >> 8) & 0xff);
	int8_t lsb = ((val >> 0) & 0xff);
	int ret;

	ret = i2c_smbus_write_byte_data(client, ISL29033_REG_INT_LT_LSB, lsb);
	if(ret != 0) {
		printk("Failed to set ht value\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29033_REG_INT_LT_MSB, msb);
	if(ret != 0) {
		printk("Failed to set ht value\n");
		return ret;
	}

	return ret;
}

static int isl29033_get_lt(struct i2c_client *client)
{
	int msb, lsb;
	struct isl29033_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->lock);
	msb = i2c_smbus_read_byte_data(client, ISL29033_REG_INT_LT_MSB);
	if(msb < 0) {
		mutex_unlock(&data->lock);
		return msb;
	}

	lsb = i2c_smbus_read_byte_data(client, ISL29033_REG_INT_LT_LSB);
	if(lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	mutex_unlock(&data->lock);

	return (msb << 8) | lsb;
}

static int isl29033_get_data(struct i2c_client *client)
{
	int msb, lsb;
	struct isl29033_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->lock);
	msb = i2c_smbus_read_byte_data(client, ISL29033_REG_DATA_MSB);
	if(msb < 0) {
		mutex_unlock(&data->lock);
		return msb;
	}

	lsb = i2c_smbus_read_byte_data(client, ISL29033_REG_DATA_LSB);
	if(lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	mutex_unlock(&data->lock);

	return (msb << 8) | lsb;
}

static int isl29033_get_adc_value(struct i2c_client *client)
{
	struct isl29033_data *data = i2c_get_clientdata(client);
	int lsb, msb, range, bitdepth;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29033_REG_DATA_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29033_REG_DATA_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = isl29033_get_range(client);
	bitdepth = (4 - isl29033_get_resolution(client)) * 4;
	return (((msb << 8) | lsb) * gain_range[range]) >> bitdepth;
}

/*
 * sysfs layer
 */

/* range */
static ssize_t isl29033_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl29033_get_range(client));
}

static ssize_t isl29033_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

	ret = isl29033_set_range(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		isl29033_show_range, isl29033_store_range);


/* resolution */
static ssize_t isl29033_show_resolution(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_resolution(client));
}

static ssize_t isl29033_store_resolution(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

	ret = isl29033_set_resolution(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
		isl29033_show_resolution, isl29033_store_resolution);

/* mode */
static ssize_t isl29033_show_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_mode(client));
}

static ssize_t isl29033_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

	ret = isl29033_set_mode(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		isl29033_show_mode, isl29033_store_mode);


/* Higher Threshold */
static ssize_t isl29033_show_ht(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_ht(client));
}

static ssize_t isl29033_store_ht(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) ||
			(val > TH_MAX_VAL) ||
			(val < TH_MIN_VAL) )
		return -EINVAL;

	ret = isl29033_set_ht(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(ht, S_IWUSR | S_IRUGO,
		isl29033_show_ht, isl29033_store_ht);

/* Lower Threshold */
static ssize_t isl29033_show_lt(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_lt(client));
}

static ssize_t isl29033_store_lt(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) ||
			(val > TH_MAX_VAL) ||
			(val < TH_MIN_VAL) )
		return -EINVAL;

	ret = isl29033_set_lt(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(lt, S_IWUSR | S_IRUGO,
		isl29033_show_lt, isl29033_store_lt);

/* Data Register */
static ssize_t isl29033_show_data(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_data(client));
}

static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, isl29033_show_data, NULL);

/* power state */
static ssize_t isl29033_show_power_state(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29033_get_power_state(client));
}

static ssize_t isl29033_store_power_state(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

	ret = isl29033_set_power_state(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		isl29033_show_power_state, isl29033_store_power_state);


/* lux */
static ssize_t isl29033_show_lux(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (!isl29033_get_power_state(client))
		return -EBUSY;

	return sprintf(buf, "%d\n", isl29033_get_adc_value(client));
}

static DEVICE_ATTR(lux, S_IRUGO, isl29033_show_lux, NULL);

static struct attribute *isl29033_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_mode.attr,
	&dev_attr_power_state.attr,
	&dev_attr_lux.attr,
	&dev_attr_ht.attr,
	&dev_attr_lt.attr,
	&dev_attr_data.attr,
	NULL
};

static const struct attribute_group isl29033_attr_group = {
	.attrs = isl29033_attributes,
};

static int isl29033_init_client(struct i2c_client *client)
{
	struct isl29033_data *data = i2c_get_clientdata(client);
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, i);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	isl29033_set_range(client, 0x03);
	isl29033_set_resolution(client, 0x00);
	isl29033_set_persist(client, 0x03);
	isl29033_set_mode(client, 0x05);

	return 0;
}

/*
 * I2C layer
 */

static int __devinit isl29033_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct isl29033_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct isl29033_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* initialize the ISL29033 chip */
	err = isl29033_init_client(client);
	if (err)
		goto exit_kfree;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &isl29033_attr_group);
	if (err)
		goto exit_kfree;

	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);
	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int __devexit isl29033_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &isl29033_attr_group);
	isl29033_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));
	return 0;
}

#ifdef CONFIG_PM
static int isl29033_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29033_data *data = i2c_get_clientdata(client);

	data->power_state_before_suspend = isl29033_get_power_state(client);
	return isl29033_set_power_state(client, 0);
}

static int isl29033_resume(struct i2c_client *client)
{
	int i;
	struct isl29033_data *data = i2c_get_clientdata(client);

	/* restore registers from cache */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
		if (i2c_smbus_write_byte_data(client, i, data->reg_cache[i]))
			return -EIO;

	return isl29033_set_power_state(client,
			data->power_state_before_suspend);
}

#else
#define isl29033_suspend	NULL
#define isl29033_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id isl29033_id[] = {
	{ "isl29033", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, isl29033_id);

static struct i2c_driver isl29033_driver = {
	.driver = {
		.name	= ISL29033_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = isl29033_suspend,
	.resume	= isl29033_resume,
	.probe	= isl29033_probe,
	.remove	= __devexit_p(isl29033_remove),
	.id_table = isl29033_id,
};

module_i2c_driver(isl29033_driver);

MODULE_AUTHOR("Saiyam Doshi");
MODULE_DESCRIPTION("ISL29033 ambient light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
