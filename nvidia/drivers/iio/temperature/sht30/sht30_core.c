// SPDX-License-Identifier: GPL-2.0-or-later
/* Sensirion SHT30 humidity and temperature sensor driver.
 *
 * Copyright (C) 2023 Direct Drive Tech
 * Author: Meowguru <jiali.gao@directdrivetech.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "sht30.h"

#define SHT30_DRV_NAME		    "sht30"

#define SEN_HUMI_IOCTL_MEASURE 		_IOWR('b', 0x1, void *)
#define SEN_HUMI_IOCTL_DEVID 		_IOWR('b', 0x2, void *)

static struct miscdevice sen_humi_miscdev;
static uint8_t * raw_data_buffer = NULL;

static uint8_t i2c_driver_added = 0;
static uint8_t misc_registered = 0;

struct sht30_comms_struct {
	__u16   len;
	__u8    *buf;
};

static const struct i2c_device_id sht30_i2c_id[] = {
	{SHT30_DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sht30_i2c_id);

static struct i2c_client *sht30_i2c_client;

static const struct of_device_id sen_humi_of_match[] = {
	{
		.compatible = "sensirion, sht30",
		.data = SHT30_DRV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, sen_humi_of_match);  // add to the kernel device tree table

static int sht30_open(struct inode *inode, struct file *file)
{
	pr_debug("sht30 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int sht30_release(struct inode *inode, struct file *file)
{
	pr_debug("sht30 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

int16_t sht3x_measure(void) {
    uint8_t cmd[2];

    cmd[0] = 0x24;
    cmd[1] = 0x00;

    return sht30_write_no_regaddr(sht30_i2c_client, cmd, 2);
}

void tick_to_temperature(uint16_t tick, int32_t* temperature) {
    *temperature = ((21875 * (int32_t)tick) >> 13) - 45000;
}

void tick_to_humidity(uint16_t tick, int32_t* humidity) {
    *humidity = ((12500 * (int32_t)tick) >> 13);
}

int16_t sht3x_read(int32_t* temperature,
                   int32_t* humidity) {
    uint8_t words[6];
    int16_t ret;
    uint16_t words_t;
    uint16_t words_h;

    ret = sht30_read_no_regaddr(sht30_i2c_client, words, 6);

    words_t = be16_to_cpup((__be16 *)words);
    words_h = be16_to_cpup((__be16 *)(words + 3));

    /**
     * formulas for conversion of the sensor signals, optimized for fixed point
     * algebra: Temperature = 175 * S_T / 2^16 - 45
     * Relative Humidity = * 100 * S_RH / 2^16
     */
    tick_to_temperature(words_t, temperature);
    tick_to_humidity(words_h, humidity);

    return ret;
}

int16_t sht3x_measure_blocking_read(int32_t* temperature,
                                    int32_t* humidity) {
    int16_t ret = sht3x_measure();
    if (ret == 0) {
        udelay(SHT3X_MEASUREMENT_DURATION_USEC);
        ret = sht3x_read(temperature, humidity);
    } else {
        pr_err("sht30 mesure failed: %d", ret);
    }
    return ret;
}

static long sht30_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int32_t temperature, humidity;
	uint8_t status[2];
	struct sht30_comms_struct comms_struct = {0};
	void __user *data_ptr = NULL;
	int32_t ret = 0;

	ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
	if (ret) {
		pr_err("Error at %s(%d)\n", __func__, __LINE__);
		return -EINVAL;
	}

	data_ptr = (u8 __user *)(comms_struct.buf);
	comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);

	switch (cmd) {
		case SEN_HUMI_IOCTL_MEASURE:
			ret = sht3x_measure_blocking_read(&temperature, &humidity);
			if (ret != 0) {
				pr_err("sht30 temperature: %d degreeCelsius, humidity: %d percentRH\n", 
					temperature / 1000, humidity / 1000);
			} else {
				pr_err("error reading measurement\n");
			}

			// copy to user buffer the read transfer
			ret = copy_to_user(data_ptr, &temperature, 4);

			if (ret) {
				pr_err("sht30: Error at %s(%d) when copy temperature to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			ret = copy_to_user(data_ptr + 4, &humidity, 4);

			if (ret) {
				pr_err("sht30: Error at %s(%d) when copy humidity to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			break;
		case SEN_HUMI_IOCTL_DEVID:
			ret = sht30_read_multi(sht30_i2c_client, raw_data_buffer, SHT3X_CMD_READ_STATUS_REG, status, 2);
			if(ret)
			{
				pr_err("sht30 get status failed\n");
			} else {
				pr_info("sht30 get status :%x %x", status[1], status[0]);
			}
			ret = copy_to_user(data_ptr, status, 2);
            if (ret) {
                pr_err("sht30: Error at %s(%d) when copy status to user.\n", __func__, __LINE__);
                return -EINVAL;
            }
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations sht30_fops = {
	.owner 			= THIS_MODULE,
	.unlocked_ioctl		= sht30_ioctl,
	.open 			= sht30_open,
	.release 		= sht30_release,
};

static int sht30_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	uint8_t status[2];
	sht30_i2c_client = client;
	

	raw_data_buffer = kzalloc(SHT30_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);

    ret = sht30_read_multi(sht30_i2c_client, raw_data_buffer, SHT3X_CMD_READ_STATUS_REG, status, 2);
	if(ret)
	{
		pr_err("sht30 get status failed\n");
	} else {
		pr_info("sht30 get status :%x %x", status[1], status[0]);
	}

	sen_humi_miscdev.minor = MISC_DYNAMIC_MINOR;
	sen_humi_miscdev.name = "sht30";
	sen_humi_miscdev.fops = &sht30_fops;
	sen_humi_miscdev.mode = 0444;

	ret = misc_register(&sen_humi_miscdev);
	if (ret) {
		pr_err("stmvl53l5cx : Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;
	
	return ret;
}

static int sht30_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver sht30_i2c_driver = {
	.driver = {
		.name = SHT30_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sen_humi_of_match),
	},
	.probe = sht30_probe,
	.remove = sht30_remove,
	.id_table = sht30_i2c_id,
};

static int __init sht30_module_init(void)
{
	int ret = 0;

	printk("sht30: module init\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&sht30_i2c_driver);

	if (ret) {
		i2c_del_driver(&sht30_i2c_driver);
		printk("sht30: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

	return ret;
}

static void __exit sht30_module_exit(void)
{

	pr_debug("sht30 : module exit\n");

	if (misc_registered) {
		misc_deregister(&sen_humi_miscdev);
		misc_registered = 0;
	}

	if (i2c_driver_added) {
		i2c_del_driver(&sht30_i2c_driver);
		i2c_driver_added = 0;
	}

}

module_init(sht30_module_init);
module_exit(sht30_module_exit);

MODULE_AUTHOR("Meowguru <jiali.gao@directdrivetech.com>");
MODULE_LICENSE("GPL");