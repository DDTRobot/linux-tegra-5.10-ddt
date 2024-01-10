// Copyright (c) Direct Drive Technology Co., Ltd. All rights reserved.
// Author: Zhibin Wu <zhibin.wu@directdrivetech.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <linux/device.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include "bmi08x.h"

#define BMI_NAME			"bmi088"
#define BMI_PART_BMI088			(0)

#define BMI_HW_DELAY_DEV_ON_US		(2)

#define M_PI        3.14159265358979323846

#define BMI088_GYRO_RANGE_2000_DPS               UINT8_C(0x00)

#define BMI_HW_DELAY_DEV_OFF_US		(1000)

/*! For dps to rps */
#define DPS_TO_RPS		     0.017453292519943295769236907684

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! bit width for LSM to m/s^2*/
#define BMI088_BIT_WIDTH                         UINT8_C(16)

#define BMI088_ACCEL_RANGE_3                     3
#define BMI088_ACCEL_RANGE_6                     6
#define BMI088_ACCEL_RANGE_12                    12
#define BMI088_ACCEL_RANGE_24                    24

#define BMI088_COMMS_CHUNK_SIZE  		1024

#define BMI_IMU_IOCTL_TRANSFER 		_IOWR('d',0x1, void*)
#define BMI_IMU_IOCTL_ADEVID 		_IOWR('d',0x2, void*)
#define BMI_IMU_IOCTL_GDEVID 		_IOWR('d',0x3, void*)
#define BMI_IMU_IOCTL_ASELFTEST 	_IOWR('d',0x4, void*)
#define BMI_IMU_IOCTL_GSELFTEST 	_IOWR('d',0x5, void*)
#define BMI_IMU_IOCTL_INIT 		    _IOWR('d',0x6, void*)

struct bmi088_comms_struct {
	__u16   len;
	__u8    *buf;
};

static struct miscdevice bmi_imu_miscdev;

struct bmi08x_sensor_data bmi08x_accel;
struct bmi08x_sensor_data bmi08x_gyro;

static u8 bmi08x_accel_spec;
static u8 bmi08x_gyro_spec;

struct bmi_state {
	struct i2c_client *i2c;
    struct bmi08x_dev dev;
};

struct bmi_state *st;
static uint8_t * raw_data_buffer = NULL;

static uint8_t misc_registered = 0;

BMI08X_INTF_RET_TYPE bmi08x_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct i2c_msg msg;
    int ret = 0;

    uint8_t addr = *((uint8_t *)intf_ptr);

    msg.addr = addr;
    msg.flags = 0;
    msg.len = 1;
    msg.buf = &reg_addr;

    ret = i2c_transfer(st->i2c->adapter, &msg, 1);
    if (ret != 1)
    {
        dev_err(&st->i2c->dev, "bmi08x_read w err");
        return -EIO;
    }
    msg.flags = 1;
    msg.len = len;
    msg.buf = reg_data;

    ret = i2c_transfer(st->i2c->adapter, &msg, 1);
    if (ret != 1)
    {
        dev_err(&st->i2c->dev, "bmi08x_read r err");
        return -EIO;
    }

    return BMI08X_INTF_RET_SUCCESS;
}


BMI08X_INTF_RET_TYPE bmi08x_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct i2c_msg msg;
	int ret;

    uint8_t addr = *((uint8_t *)intf_ptr);

    raw_data_buffer[0] = reg_addr;

    memcpy(&raw_data_buffer[1], reg_data, len);

    msg.addr = addr;
    msg.flags = 0;
    msg.len = len + 1;
    msg.buf = raw_data_buffer;

    ret = i2c_transfer(st->i2c->adapter, &msg, 1);
    if (ret != 1)
	{
        dev_err(&st->i2c->dev, "bmi08x_write w err");
        return -EIO;
    }

    return BMI08X_INTF_RET_SUCCESS;
}

void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
    udelay(period);
}

static struct bmi08x_dev bmi08x_dev = {

    .intf_ptr_accel = (void *)&bmi08x_accel_spec,
    .intf_ptr_gyro = (void *)&bmi08x_gyro_spec,
    .intf = BMI08X_I2C_INTF,
    .variant = BMI088_VARIANT,
    .read_write_len = 64,
    .read = bmi08x_read,
    .write = bmi08x_write,
    .delay_us = bmi08x_delay_us,
};

int bmi088_get_data(void)
{
    int8_t rslt;
    rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08x_dev);
    if (rslt != BMI08X_OK) {
        dev_err(&st->i2c->dev, "Gyro error: %d", rslt);
    }

    rslt = bmi08a_get_data(&bmi08x_accel, &bmi08x_dev);
    if (rslt != BMI08X_OK) {
        dev_err(&st->i2c->dev, "Accel error: %d", rslt);
    }

    return 0;
}

static int bmi088_open(struct inode *inode, struct file *file)
{
    dev_info(&st->i2c->dev, "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int bmi088_release(struct inode *inode, struct file *file)
{
    dev_info(&st->i2c->dev, "%s(%d)\n", __func__, __LINE__);
	return 0;
}

int bmi088_init(void)
{
    int8_t rslt;

    rslt = bmi08a_init(&bmi08x_dev);
    if (rslt != BMI08X_OK) {
        dev_err(&st->i2c->dev, "bmi08a_init failed. %d",rslt);
    }

    if (rslt == BMI08X_OK) {
        rslt = bmi08g_init(&bmi08x_dev);
    } else {
        dev_err(&st->i2c->dev, "bmi08g_init failed. %d",rslt);
    }

    if (rslt == BMI08X_OK) {
        rslt = bmi08a_soft_reset(&bmi08x_dev);
        if (rslt != BMI08X_OK) {
            dev_err(&st->i2c->dev, "bmi08a_soft_reset failed.");
        }

        rslt = bmi08g_soft_reset(&bmi08x_dev);
        if (rslt != BMI08X_OK) {
            dev_err(&st->i2c->dev, "bmi08g_soft_reset failed.");
        }
    }

    if (rslt == BMI08X_OK) {
        rslt = bmi08a_load_config_file(&bmi08x_dev);
        if (rslt != BMI08X_OK) {
            dev_err(&st->i2c->dev, "Uploading config file failed.");
        }
    }

    rslt = BMI08X_OK;

    if (rslt == BMI08X_OK) {
        bmi08x_dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
        bmi08x_dev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;
        bmi08x_dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        bmi08x_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;

        rslt = bmi08a_set_power_mode(&bmi08x_dev);
        if (rslt == BMI08X_OK) {
            rslt = bmi08a_set_meas_conf(&bmi08x_dev);
        } else {
            dev_err(&st->i2c->dev, "Accel power on failed.");
        }

        bmi08x_dev.gyro_cfg.bw = BMI08X_GYRO_BW_64_ODR_200_HZ;
        bmi08x_dev.gyro_cfg.odr = BMI08X_GYRO_BW_64_ODR_200_HZ;
        bmi08x_dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
        bmi08x_dev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;

        rslt = bmi08g_set_power_mode(&bmi08x_dev);
        if (rslt == BMI08X_OK) {
            dev_err(&st->i2c->dev, "bmi088 set_power_mode ok.");
            rslt = bmi08g_set_meas_conf(&bmi08x_dev);
        } else {
            dev_err(&st->i2c->dev, "bmi088 Gyro power on failed.");
        }

    }

    if (rslt == BMI08X_OK) {
        dev_err(&st->i2c->dev, "Accel ID: 0x%02X", bmi08x_dev.accel_chip_id);
        dev_err(&st->i2c->dev, "Gyro ID: 0x%02X", bmi08x_dev.gyro_chip_id);
    } else {
        dev_err(&st->i2c->dev, "BMI08x initial failed");
    }

    return rslt;
}

static void bmi_remove(void *data)
{
	struct i2c_client *client = data;

	dev_info(&client->dev, "removed\n");
}

static long bmi088_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
    uint8_t self_rslt;
    int init_status;
	struct bmi088_comms_struct comms_struct = {0};
	int ret = 0;
	void __user *data_ptr = NULL;

    ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
    if (ret) {
        dev_err(&st->i2c->dev, "Error at %s(%d)\n", __func__, __LINE__);
        return -EINVAL; 
    }

    data_ptr = (u8 __user *)(comms_struct.buf);
    comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);

    switch (cmd) {
        case BMI_IMU_IOCTL_INIT:
            ret = bmi088_init();
            if (ret) {
                bmi_remove(st->i2c);
            }
            init_status = ret;
            ret = copy_to_user(data_ptr, &init_status, sizeof(int));
            if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy init_status to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

            break;
		case BMI_IMU_IOCTL_TRANSFER:
			ret = bmi088_get_data();
			if (ret) {
				dev_err(&st->i2c->dev, "Error at bmi088_get_data %s(%d)\n", __func__, __LINE__);
			}

			ret = copy_to_user(data_ptr, &bmi08x_accel, 2);
            ret = copy_to_user(data_ptr + 2, &bmi08x_accel.y, 2);
            ret = copy_to_user(data_ptr + 4, &bmi08x_accel.z, 2);

			if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy accel_data to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			ret = copy_to_user(data_ptr + 6, &bmi08x_gyro, 2);
            ret = copy_to_user(data_ptr + 8, &bmi08x_gyro.y, 2);
            ret = copy_to_user(data_ptr + 10, &bmi08x_gyro.z, 2);

			if (ret) {
				dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy gyro to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			break;
        case BMI_IMU_IOCTL_ADEVID:
            ret = copy_to_user(data_ptr, &bmi08x_dev.accel_chip_id, 1);
            if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy adev id to user.\n", __func__, __LINE__);
                return -EINVAL;
            }

            break;
        case BMI_IMU_IOCTL_GDEVID:
            ret = copy_to_user(data_ptr + 1, &bmi08x_dev.gyro_chip_id, 1);
            if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy gdev id to user.\n", __func__, __LINE__);
                return -EINVAL;
            }

            break;
        case BMI_IMU_IOCTL_ASELFTEST:
            self_rslt = bmi08a_perform_selftest(&bmi08x_dev);

            if(!self_rslt) {
                dev_err(&st->i2c->dev, "bmi088: aself_rslt at %s(%d): 0x%x.\n", __func__, __LINE__, self_rslt);
                self_rslt = 0;
            } else {
                dev_err(&st->i2c->dev, "bmi088: error aself_rslt at %s(%d): 0x%x.\n", __func__, __LINE__, self_rslt);
                self_rslt = 1;
            }

            ret = copy_to_user(data_ptr + 2, &self_rslt, 1);
            if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy gdev id to user.\n", __func__, __LINE__);
                return -EINVAL;
            }
            break;

        case BMI_IMU_IOCTL_GSELFTEST:
            self_rslt = bmi08g_perform_selftest(&bmi08x_dev);

            if(!self_rslt) {
                dev_err(&st->i2c->dev, "bmi088: gself_rslt at %s(%d): 0x%x.\n", __func__, __LINE__, self_rslt);
                self_rslt = 0;
            } else {
                dev_err(&st->i2c->dev, "bmi088: error gself_rslt at %s(%d): 0x%x.\n", __func__, __LINE__, self_rslt);
                self_rslt = 1;
            }

            ret = copy_to_user(data_ptr + 3, &self_rslt, 1);
            if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy gdev id to user.\n", __func__, __LINE__);
                return -EINVAL;
            }
            break;

		default:
			return -EINVAL;
	}

    return 0;
}

static const struct file_operations bmi088_ranging_fops = {
	.owner 			= THIS_MODULE,
	.unlocked_ioctl		= bmi088_ioctl,
	.open 			= bmi088_open,
	.release 		= bmi088_release,
};

static int bmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	// struct i2c_msg msg[2];
	// u8 reg = 0x00;
	// u8 chip_id;
    u32 val32 = 0;

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

    raw_data_buffer = kzalloc(BMI088_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL)
		 return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
    st->dev = bmi08x_dev;
#if 0
	{
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &reg;
		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = 1;
		msg[1].buf = &chip_id;

		ret = i2c_transfer(client->adapter, msg, 2);
		if(ret != 2 || chip_id != 0x0f) {
            dev_err(&st->i2c->dev, "ERR: read chip id failed\n");
			return ret;
		} else {
            dev_info(&st->i2c->dev, "read chip id success\n");
            ret  = 0;
        }
			
	}
#endif
    bmi08x_gyro_spec = st->i2c->addr;
    
    if (!of_property_read_u32(st->i2c->dev.of_node, "accel_i2c_addr", &val32))
        bmi08x_accel_spec = val32;
    else
        return -ENODEV;
    dev_info(&st->i2c->dev, "bmi08x_gyro_spec: %x\n", bmi08x_gyro_spec);
    dev_info(&st->i2c->dev, "bmi08x_accel_spec: %x\n", bmi08x_accel_spec);

	ret = devm_add_action_or_reset(&client->dev, bmi_remove, client);
	if (ret)
		return ret;

    bmi_imu_miscdev.minor = MISC_DYNAMIC_MINOR;
	bmi_imu_miscdev.name = "bmi088";
	bmi_imu_miscdev.fops = &bmi088_ranging_fops;
	bmi_imu_miscdev.mode = 0444;

    ret = misc_register(&bmi_imu_miscdev);
	if (ret) {
        dev_err(&st->i2c->dev, "Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;

	dev_info(&client->dev, "done\n");

	return ret;
}

static const struct i2c_device_id bmi_i2c_device_ids[] = {
	{ BMI_NAME, BMI_PART_BMI088 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bmi_i2c_device_ids);

static const struct of_device_id bmi_of_match[] = {
	{ .compatible = "bmi,bmi088", },
	{}
};

MODULE_DEVICE_TABLE(of, bmi_of_match);

static struct i2c_driver bmi_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= bmi_probe,
	.driver				= {
		.name			= BMI_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(bmi_of_match),
        .probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table			= bmi_i2c_device_ids,
};

module_i2c_driver(bmi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BMI088 I2C driver");
MODULE_AUTHOR("NVIDIA Corporation");
