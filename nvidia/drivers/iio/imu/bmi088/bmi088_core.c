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
// #include "bmi088.h"

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

struct bmi088_comms_struct {
	__u16   len;
	__u8    *buf;
};

static struct miscdevice bmi_imu_miscdev;

struct bmi08x_sensor_data bmi08x_accel;
struct bmi08x_sensor_data bmi08x_gyro;

// LOG_MODULE_REGISTER(bmi08x, CONFIG_LOG_DEFAULT_LEVEL);

// #define BMI08x_SPI_OPS (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

// static struct spi_dt_spec bmi08x_accel_spec = SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel), BMI08x_SPI_OPS, 0);
// static struct spi_dt_spec bmi08x_gyro_spec = SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro), BMI08x_SPI_OPS, 0);

static u8 bmi08x_accel_spec;
static u8 bmi08x_gyro_spec;

struct bmi_state {
	struct i2c_client *i2c;
    struct bmi08x_dev dev;
};

struct bmi_state *st;
static uint8_t * raw_data_buffer = NULL;

static int accel_range[] = {3, 6, 12, 24};
static int gyro_range[] = {2000, 1000, 500, 250, 125};

static float accl_psc = 0; // G/Gravitational
static float gyro_psc = 0; // Angle， -180, + 180
static float stamp_psc = 0;
static uint8_t misc_registered = 0;

// static struct bmi08x_accel_int_channel_cfg accel_int_config;
// static struct bmi08x_gyro_int_channel_cfg gyro_int_config;

// static void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt)
// {
//     if (rslt != BMI08X_OK)
//     {
//         if (rslt == BMI08X_E_NULL_PTR) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Null pointer", api_name, rslt);
//         } else if (rslt == BMI08X_E_COM_FAIL) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Communication failure", api_name, rslt);
//         } else if (rslt == BMI08X_E_DEV_NOT_FOUND) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Device not found", api_name, rslt);
//         } else if (rslt == BMI08X_E_OUT_OF_RANGE) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Out of Range", api_name, rslt);
//         } else if (rslt == BMI08X_E_INVALID_INPUT) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Invalid input", api_name, rslt);
//         } else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Config stream error", api_name, rslt);
//         } else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Invalid Read write length", api_name, rslt);
//         } else if (rslt == BMI08X_E_INVALID_CONFIG) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Invalid config", api_name, rslt);
//         } else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED) {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Feature not supported", api_name, rslt);
//         } else if (rslt == BMI08X_W_FIFO_EMPTY) {
//             dev_err(&st->i2c->dev, "%s\tWarning [%d] : FIFO empty", api_name, rslt);
//         } else {
//             dev_err(&st->i2c->dev, "%s\tError [%d] : Unknown error code", api_name, rslt);
//         }
//     }
// }

BMI08X_INTF_RET_TYPE bmi08x_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct i2c_msg msg;
    int ret = 0;

    uint8_t addr = *((uint8_t *)intf_ptr);
    // dev_err(&st->i2c->dev, "bmi08x_read addr: %x", addr);

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
    //uint8_t buf[66];

    uint8_t addr = *((uint8_t *)intf_ptr);

    raw_data_buffer[0] = reg_addr;
    //buf[0] = reg_addr;

    memcpy(&raw_data_buffer[1], reg_data, len);
    //memcpy(&buf[1], reg_data, len);

    //printk("bmi088 len: %d\n", len);

    msg.addr = addr;
    msg.flags = 0;
    msg.len = len + 1;
    msg.buf = raw_data_buffer;
    //msg.buf = buf;

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

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
// static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
// {
//     float gravity;

//     float half_scale = ((1 << bit_width) / 2.0f);

//     gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

//     return gravity;
// }

// /*!
//  * @brief This function converts lsb to degree per second for 16 bit gyro at
//  * range 125, 250, 500, 1000 or 2000dps.
//  */
// static float lsb_to_dps(int16_t val, int8_t dps, uint8_t bit_width)
// {
//     float half_scale;

//     half_scale = ((float)(1 << bit_width) / 2.0f);

//     return (dps / ((half_scale) + (float)BMI088_GYRO_RANGE_2000_DPS)) * (float)(val) * DPS_TO_RPS;
// }

int bmi088_get_data(void)
{
    int8_t rslt;
    // struct bmi08x_sensor_data bmi08x_accel;
    // struct bmi08x_sensor_data bmi08x_gyro;
    float accel_data[3];
    float gyro_data[3];

    rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08x_dev);
    if (rslt != BMI08X_OK) {
        dev_err(&st->i2c->dev, "Gyro error: %d", rslt);
    }

    rslt = bmi08a_get_data(&bmi08x_accel, &bmi08x_dev);
    if (rslt != BMI08X_OK) {
        dev_err(&st->i2c->dev, "Accel error: %d", rslt);
    }

    dev_err(&st->i2c->dev, "Accel data: %d, %d, %d", bmi08x_accel.x, bmi08x_accel.y, bmi08x_accel.z);
    //dev_err(&st->i2c->dev, "Gyro data: %d, %d, %d", bmi08x_gyro.x, bmi08x_gyro.y, bmi08x_gyro.z);

    // accel_data[0] = lsb_to_mps2(bmi08x_accel.x, BMI088_ACCEL_RANGE_24, BMI088_BIT_WIDTH);
    // accel_data[1] = lsb_to_mps2(bmi08x_accel.y, BMI088_ACCEL_RANGE_24, BMI088_BIT_WIDTH);
    // accel_data[2] = lsb_to_mps2(bmi08x_accel.z, BMI088_ACCEL_RANGE_24, BMI088_BIT_WIDTH);
    accel_data[0] = (GRAVITY_EARTH * bmi08x_accel.x * BMI088_ACCEL_RANGE_24) / ((1 << BMI088_BIT_WIDTH) / 2.0f);
    accel_data[1] = (GRAVITY_EARTH * bmi08x_accel.y * BMI088_ACCEL_RANGE_24) / ((1 << BMI088_BIT_WIDTH) / 2.0f);
    accel_data[2] = (GRAVITY_EARTH * bmi08x_accel.z * BMI088_ACCEL_RANGE_24) / ((1 << BMI088_BIT_WIDTH) / 2.0f);
    //dev_err(&st->i2c->dev, "Accel data: %d, %d, %d", accel_data[0], accel_data[1], accel_data[2]);

    // gyro_data[0] = lsb_to_dps(bmi08x_gyro.x, 250, BMI088_BIT_WIDTH);
    // gyro_data[1] = lsb_to_dps(bmi08x_gyro.y, 250, BMI088_BIT_WIDTH);
    // gyro_data[2] = lsb_to_dps(bmi08x_gyro.z, 250, BMI088_BIT_WIDTH);
    gyro_data[0] = (250 / ((1 << BMI088_BIT_WIDTH) / 2.0f) + (float)BMI088_GYRO_RANGE_2000_DPS) * (float)(bmi08x_gyro.x) * DPS_TO_RPS;
    gyro_data[1] = (250 / ((1 << BMI088_BIT_WIDTH) / 2.0f) + (float)BMI088_GYRO_RANGE_2000_DPS) * (float)(bmi08x_gyro.y) * DPS_TO_RPS;
    gyro_data[2] = (250 / ((1 << BMI088_BIT_WIDTH) / 2.0f) + (float)BMI088_GYRO_RANGE_2000_DPS) * (float)(bmi08x_gyro.z) * DPS_TO_RPS;
    //dev_err(&st->i2c->dev, "Gyro data: %d, %d, %d", gyro_data[0], gyro_data[1], gyro_data[2]);

    return 0;
}

static int bmi088_open(struct inode *inode, struct file *file)
{
	pr_debug("bmi088 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int bmi088_release(struct inode *inode, struct file *file)
{
	pr_debug("bmi088 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static long bmi088_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
    // float accel_data[3];
    // float gyro_data[3];
    // int16_t acc_x;
    // int16_t gyr_x;

	struct bmi088_comms_struct comms_struct = {0};
	int32_t ret = 0;
	void __user *data_ptr = NULL;

    switch (cmd) {
		case BMI_IMU_IOCTL_TRANSFER:

            dev_err(&st->i2c->dev, "bmi088_ioctl : cmd = %u\n", cmd);

            ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
			if (ret) {
                dev_err(&st->i2c->dev, "Error at %s(%d)\n", __func__, __LINE__);
				return -EINVAL;
			}

			data_ptr = (u8 __user *)(comms_struct.buf);
			comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);

			ret = bmi088_get_data();
			if (ret) {
				dev_err(&st->i2c->dev, "Error at bmi088_get_data %s(%d)\n", __func__, __LINE__);
			}

			// copy to user buffer the read transfer
            // acc_x = bmi08x_accel.x;
            // dev_err(&st->i2c->dev, "bmi088: acc_x: %d.\n", acc_x);
            // ret = copy_to_user(data_ptr, &acc_x, 2);
			ret = copy_to_user(data_ptr, &bmi08x_accel, 2);
            ret = copy_to_user(data_ptr + 2, &bmi08x_accel.y, 2);
            ret = copy_to_user(data_ptr + 4, &bmi08x_accel.z, 2);

			if (ret) {
                dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy accel_data to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

            // gyr_x = bmi08x_gyro.x;
            // ret = copy_to_user(data_ptr + 6, &gyr_x, 2);
			ret = copy_to_user(data_ptr + 6, &bmi08x_gyro, 2);
            ret = copy_to_user(data_ptr + 8, &bmi08x_gyro.y, 2);
            ret = copy_to_user(data_ptr + 10, &bmi08x_gyro.z, 2);

			if (ret) {
				dev_err(&st->i2c->dev, "bmi088: Error at %s(%d) when copy gyro to user.\n", __func__, __LINE__);
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

static void bmi_remove(void *data)
{
	struct i2c_client *client = data;

	dev_info(&client->dev, "removed\n");
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
        bmi08x_dev.accel_cfg.odr = BMI08X_ACCEL_ODR_100_HZ;
        bmi08x_dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        bmi08x_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;

        rslt = bmi08a_set_power_mode(&bmi08x_dev);
        if (rslt == BMI08X_OK) {
            rslt = bmi08a_set_meas_conf(&bmi08x_dev);
        } else {
            dev_err(&st->i2c->dev, "Accel power on failed.");
        }

        bmi08x_dev.gyro_cfg.bw = BMI08X_GYRO_BW_32_ODR_100_HZ;
        bmi08x_dev.gyro_cfg.odr = BMI08X_GYRO_BW_32_ODR_100_HZ;
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
        accl_psc = accel_range[bmi08x_dev.accel_cfg.range] / 32768.0f;
        gyro_psc = gyro_range[bmi08x_dev.gyro_cfg.range] / 32768.0f * M_PI / 180;
        stamp_psc = 0;
        // imu->accl_scale[0] = imu->accl_scale[1] = imu->accl_scale[2] = 1.0f;
        // imu->accl_bias[0] = imu->accl_bias[1] = imu->accl_bias[2] = 0.0f;
        // imu->gyro_bias[0] = imu->gyro_bias[1] = imu->gyro_bias[2] = 0.0f;
    } else {
        dev_err(&st->i2c->dev, "BMI08x initial failed");
    }

    return rslt;
}

static int bmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct i2c_msg msg[2];
    // uint8_t drdy_data;
	u8 reg = 0x00;
	u8 chip_id;
    u32 val32 = 0;
    // int count = 10;
    // uint8_t drdy_addr = 0x03;

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

    raw_data_buffer = kzalloc(BMI088_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL)
		 return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
    st->dev = bmi08x_dev;

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
            dev_err(&st->i2c->dev, "read chip id success\n");
            ret  = 0;
        }
			
	}

    bmi08x_gyro_spec = st->i2c->addr;
    
    if (!of_property_read_u32(st->i2c->dev.of_node, "accel_i2c_addr", &val32))
        bmi08x_accel_spec = val32;
    else
        return -ENODEV;
    dev_err(&st->i2c->dev, "bmi08x_gyro_spec: %x\n", bmi08x_gyro_spec);
    dev_err(&st->i2c->dev, "bmi08x_accel_spec: %x\n", bmi08x_accel_spec);

    ret = bmi088_init();
    if (ret) {
		bmi_remove(client);
		return ret;
	}

	ret = devm_add_action_or_reset(&client->dev, bmi_remove, client);
	if (ret)
		return ret;

    bmi_imu_miscdev.minor = MISC_DYNAMIC_MINOR;
	bmi_imu_miscdev.name = "bmi088";
	bmi_imu_miscdev.fops = &bmi088_ranging_fops;
	bmi_imu_miscdev.mode = 0444;

    ret = misc_register(&bmi_imu_miscdev);
	if (ret) {
		pr_err("stmvl53l5cx : Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;

	dev_err(&client->dev, "done\n");

//     while(count > 0)
//     {
//         count --;
// // check_drdy:
//         //bmi08x_read(drdy_addr, &drdy_data, 1, bmi08x_dev.intf_ptr_accel);
//         // if(drdy_data & 0x80)
//         // {
//         //    bmi088_get_data(); 
//         // } else {
//         //     udelay(500);
//         //     goto check_drdy;
//         // }
//         bmi088_get_data(); 
//         mdelay(10);
//     }

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
		//.pm			= &bmi_pm_ops,
	},
	.id_table			= bmi_i2c_device_ids,
};

module_i2c_driver(bmi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BMI088 I2C driver");
MODULE_AUTHOR("NVIDIA Corporation");
