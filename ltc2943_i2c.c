/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : ltc2943_i2c.c
 @brief  : the driver of ltc2943 I2C .
 @author : pengrui
 @history:
           2018-05-04    pengrui    Created file
           ...
******************************************************************************/
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>

#include "ltc2943_core.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold cdata->lock */
static int ltc2943_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	if (len > 1)
		addr |= I2C_AUTO_INCREMENT;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	return i2c_transfer(client->adapter, msg, 2);
}

/* XXX: caller must hold cdata->lock */
static int ltc2943_i2c_write(struct device *dev, u8 addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	if (len > 1)
		addr |= I2C_AUTO_INCREMENT;

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len + 1;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static const struct ltc2943_transfer_function ltc2943_i2c_tf = {
	.write = ltc2943_i2c_write,
	.read = ltc2943_i2c_read,
};

#ifdef CONFIG_PM
static int ltc2943_i2c_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct ltc2943_dev *dev = i2c_get_clientdata(client);

	return ltc2943_enable(dev);
}

static int ltc2943_i2c_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct ltc2943_dev *dev = i2c_get_clientdata(client);

	return ltc2943_disable(dev);
}

static const struct dev_pm_ops ltc2943_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltc2943_i2c_suspend,
				ltc2943_i2c_resume)
};
#endif /* CONFIG_PM */

static int ltc2943_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int err;
	struct ltc2943_dev *dev;

#ifdef ltc2943_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct ltc2943_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = client->name;
	dev->bus_type = BUS_I2C;
	dev->tf = &ltc2943_i2c_tf;
	dev->dev = &client->dev;

	i2c_set_clientdata(client, dev);

	mutex_init(&dev->lock);

	err = ltc2943_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

int ltc2943_i2c_remove(struct i2c_client *client)
{
	struct ltc2943_dev *dev = i2c_get_clientdata(client);

#ifdef LTC2943_DEBUG
	dev_info(&client->dev, "driver removing\n");
#endif

	ltc2943_remove(dev);
	kfree(dev);

	return 0;
}

static const struct i2c_device_id ltc2943_i2c_id[] = {
	{ "ltc2943", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ltc2943_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id ltc2943_i2c_id_table[] = {
	{ .compatible = "ltc,ltc2943" },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc2943_i2c_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver ltc2943_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ltc2943",
#ifdef CONFIG_PM
		.pm = &ltc2943_i2c_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = ltc2943_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = ltc2943_i2c_probe,
	.remove = ltc2943_i2c_remove,
	.id_table = ltc2943_i2c_id,
};

module_i2c_driver(ltc2943_i2c_driver);

MODULE_DESCRIPTION("ltc2943 i2c driver");
MODULE_AUTHOR("pengrui <pengrui_2009@163.com>");
MODULE_LICENSE("GPL v2");


