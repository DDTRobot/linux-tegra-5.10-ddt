/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
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

#include "stmvl53l5cx_i2c.h"
#include "stmvl53l5cx_adapt.h"

/*
 * GPIO number connected to L5 interrupt signal GPIO1
 */

struct vl53_gpio_struct {
	int intr_gpio_nb;
	int rst_gpio;
	int power_gpio;
	int lpn_gpio;
};

struct vl53_gpio_struct vl53_gpios[2];

#define STMVL53L5_DRV_NAME "stmvl53l5cx"
#define STMVL53L5_SLAVE_ADDR 0x29

#define ST_TOF_IOCTL_TRANSFER _IOWR('a', 0x1, void *)
#define ST_TOF_IOCTL_WAIT_FOR_INTERRUPT _IO('a', 0x2)
#define ST_TOF_IOCTL_ENABLE_INTERRUPT _IO('a', 0x3)
#define ST_TOF_IOCTL_DISABLE_INTERRUPT _IO('a', 0x4)

#define ST_TOF_IOCTL_TRANSFER_1 _IOWR('e', 0x1, void *)
#define ST_TOF_IOCTL_WAIT_FOR_INTERRUPT_1 _IO('e', 0x2)
#define ST_TOF_IOCTL_ENABLE_INTERRUPT_1 _IO('e', 0x3)
#define ST_TOF_IOCTL_DISABLE_INTERRUPT_1 _IO('e', 0x4)

struct stmvl53l5cx_comms_struct {
	__u16 len;
	__u16 reg_index;
	__u8 *buf;
	__u8 write_not_read;
};

struct gpio_own_flags_t {
	unsigned intr_gpio_owned : 1; /*!< set if intr gpio is owned*/
};

struct gpio_own_flags_t gpio_own_flags[2];

static struct miscdevice st_tof_miscdev_0;
static struct miscdevice st_tof_miscdev_1;
static uint8_t *raw_data_buffer_0 = NULL;
static uint8_t *raw_data_buffer_1 = NULL;

static uint8_t i2c_driver_added = 0;
static uint8_t misc_registered_0 = 0;
static uint8_t misc_registered_1 = 0;

static uint8_t irq_handler_registered_0 = 0;
static uint8_t irq_handler_registered_1 = 0;

static wait_queue_head_t wq;

static atomic_t intr_ready_flag[2];

static atomic_t force_wakeup[2];

static unsigned int st_tof_irq_num[2];

static const struct of_device_id st_tof_of_match[] = {
	{
		.compatible = "st,stmvl53l5cx_0",
		.data = STMVL53L5_DRV_NAME,
	},
	{
		.compatible = "st,stmvl53l5cx_1",
		.data = STMVL53L5_DRV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st_tof_of_match);

static struct i2c_client *stmvl53l5cx_i2c_client_0 = NULL;
static struct i2c_client *stmvl53l5cx_i2c_client_1 = NULL;

static int get_gpio(int gpio_number, const char *name, int direction)
{
	int rc = 0;

	if (gpio_number == -1) {
		pr_info("%s(%d) : gpio is required\n", __func__, __LINE__);
		rc = -ENODEV;
		goto no_gpio;
	}

	pr_debug("request gpio_number %d", gpio_number);
	rc = gpio_request(gpio_number, name);
	if (rc) {
		pr_info("fail to acquire gpio(%d), error = %d", gpio_number,
			rc);
		goto request_failed;
	}

	if (direction == 1) {
		rc = gpio_direction_output(gpio_number, 0);
		if (rc) {
			pr_info("fail to configure gpio_number(%d) as output %d",
				gpio_number, rc);
			goto direction_failed;
		}
	} else {
		rc = gpio_direction_input(gpio_number);
		if (rc) {
			pr_info("fail to configure gpio_number(%d) as input %d",
				gpio_number, rc);
			goto direction_failed;
		}
	}

	return rc;

direction_failed:
	gpio_free(gpio_number);

request_failed:
no_gpio:
	return rc;
}

static void put_gpio(int gpio_number)
{
	pr_debug("release gpio_number %d", gpio_number);
	gpio_free(gpio_number);
}

static int stmvl53l5cx_open_0(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5cx_release_0(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5cx_open_1(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5cx_release_1(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

/* Interrupt handler */
static irqreturn_t st_tof_intr_handler_0(int st_tof_irq_num, void *dev_id)
{
	// pr_info("enter interrupt handler\n");
	/* Update interrupt flag */
	atomic_set(&intr_ready_flag[0], 1);

	/* Unblock the IOCTL call to return to user space */
	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

static irqreturn_t st_tof_intr_handler_1(int st_tof_irq_num, void *dev_id)
{
	// pr_info("enter interrupt handler\n");
	/* Update interrupt flag */
	atomic_set(&intr_ready_flag[1], 1);

	/* Unblock the IOCTL call to return to user space */
	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

static long stmvl53l5cx_ioctl_0(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct i2c_msg st_i2c_message;
	struct stmvl53l5cx_comms_struct comms_struct = { 0 };
	int32_t ret = 0;
	uint16_t index, transfer_size, chunk_size;
	void __user *data_ptr = NULL;

	pr_debug("stmvl53l5cx_ioctl : cmd = %u\n", cmd);
	switch (cmd) {
	case ST_TOF_IOCTL_ENABLE_INTERRUPT:
		enable_irq(st_tof_irq_num[0]);
		pr_debug("stmvl53l5cx enable interrupt\n");
		break;
	case ST_TOF_IOCTL_DISABLE_INTERRUPT:
		disable_irq(st_tof_irq_num[0]);
		pr_debug("stmvl53l5cx disable interrupt\n");
		break;
	case ST_TOF_IOCTL_WAIT_FOR_INTERRUPT:
		pr_debug("%s(%d)\n", __func__, __LINE__);
		atomic_set(&force_wakeup[0], 0);
		ret = wait_event_interruptible(
			wq, atomic_read(&intr_ready_flag[0]) != 0);
		if (ret || atomic_read(&force_wakeup[0])) {
			pr_debug(
				"stmvl53l5cx wait_event_interruptible ret = %d, force_wakeup flag = %d\n",
				ret, atomic_read(&force_wakeup[0]));
			atomic_set(&intr_ready_flag[0], 0);
			atomic_set(&force_wakeup[0], 0);
			return -EINTR;
		}
		atomic_set(&intr_ready_flag[0], 0);
		break;
	case ST_TOF_IOCTL_TRANSFER:

		ret = copy_from_user(&comms_struct, (void __user *)arg,
				     sizeof(comms_struct));
		if (ret) {
			pr_info("Error at %s(%d)\n", __func__, __LINE__);
			return -EINVAL;
		}

		pr_debug(
			"Transfer. write_not_read = %d, reg_index = 0x%x size = %d\n",
			comms_struct.write_not_read, comms_struct.reg_index,
			comms_struct.len);
		// address and buis the same whatever the transfers to be done !
		st_i2c_message.addr = 0x27;
		// st_i2c_message.buf is the same whatever the transfers to be done
		st_i2c_message.buf = raw_data_buffer_0;

		if (!comms_struct.write_not_read) {
			data_ptr = (u8 __user *)(comms_struct.buf);
			comms_struct.buf =
				memdup_user(data_ptr, comms_struct.len);
		}

		// in case of i2c write, it is a single transfer with read index set in the 2 first bytes
		// the other case use fully the raw data buffer for raw data transfers
		if (comms_struct.write_not_read)
			chunk_size = VL53L5CX_COMMS_CHUNK_SIZE - 2;
		else
			chunk_size = VL53L5CX_COMMS_CHUNK_SIZE;

		// index is the number of bytes already transfered
		index = 0;

		do {
			// take the max number of bytes that can be transfered
			transfer_size =
				(comms_struct.len - index) > chunk_size ?
					chunk_size :
					(comms_struct.len - index);

			// ----- WRITE
			if (comms_struct.write_not_read) {
				// put red index at the beginning of the buffer
				raw_data_buffer_0[0] =
					(uint8_t)(((comms_struct.reg_index +
						    index) &
						   0xFF00) >>
						  8);
				raw_data_buffer_0[1] =
					(uint8_t)((comms_struct.reg_index +
						   index) &
						  0x00FF);

				ret = copy_from_user(&raw_data_buffer_0[2],
						     comms_struct.buf + index,
						     transfer_size);
				if (ret) {
					pr_info("Error at %s(%d)\n", __func__,
						__LINE__);
					return -EINVAL;
				}

				st_i2c_message.len = transfer_size + 2;
				st_i2c_message.flags = 0;
				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_0->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_info("Error %d at %s(%d)\n", ret,
						__func__, __LINE__);
					return -EIO;
				}
			}
			// ----- READ
			else {
				// write reg_index
				st_i2c_message.len = 2;
				st_i2c_message.flags = 0;
				raw_data_buffer_0[0] =
					(uint8_t)(((comms_struct.reg_index +
						    index) &
						   0xFF00) >>
						  8);
				raw_data_buffer_0[1] =
					(uint8_t)((comms_struct.reg_index +
						   index) &
						  0x00FF);

				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_0->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_info("Error at %s(%d)\n", __func__,
						__LINE__);
					return -EIO;
				}

				st_i2c_message.len = transfer_size;
				st_i2c_message.flags = 1;

				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_0->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_info("Error at %s(%d)\n", __func__,
						__LINE__);
					return -EIO;
				}

				// copy to user buffer the read transfer
				ret = copy_to_user(data_ptr + index,
						   raw_data_buffer_0,
						   transfer_size);

				if (ret) {
					pr_info("Error at %s(%d)\n", __func__,
						__LINE__);
					return -EINVAL;
				}

			} // ----- READ

			index += transfer_size;

		} while (index < comms_struct.len);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static long stmvl53l5cx_ioctl_1(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct i2c_msg st_i2c_message;
	struct stmvl53l5cx_comms_struct comms_struct = { 0 };
	int32_t ret = 0;
	uint16_t index, transfer_size, chunk_size;
	void __user *data_ptr = NULL;

	pr_debug("stmvl53l5cx_ioctl : cmd = %u\n", cmd);
	switch (cmd) {
	case ST_TOF_IOCTL_ENABLE_INTERRUPT_1:
		enable_irq(st_tof_irq_num[1]);
		pr_debug("stmvl53l5cx enable interrupt\n");
		break;
	case ST_TOF_IOCTL_DISABLE_INTERRUPT_1:
		disable_irq(st_tof_irq_num[1]);
		pr_debug("stmvl53l5cx disable interrupt\n");
		break;
	case ST_TOF_IOCTL_WAIT_FOR_INTERRUPT_1:
		pr_debug("%s(%d)\n", __func__, __LINE__);
		atomic_set(&force_wakeup[1], 0);
		ret = wait_event_interruptible(
			wq, atomic_read(&intr_ready_flag[1]) != 0);
		if (ret || atomic_read(&force_wakeup[1])) {
			pr_debug(
				"stmvl53l5cx wait_event_interruptible ret = %d, force_wakeup flag = %d\n",
				ret, atomic_read(&force_wakeup[1]));
			atomic_set(&intr_ready_flag[1], 0);
			atomic_set(&force_wakeup[1], 0);
			return -EINTR;
		}
		atomic_set(&intr_ready_flag[1], 0);
		break;
	case ST_TOF_IOCTL_TRANSFER_1:

		ret = copy_from_user(&comms_struct, (void __user *)arg,
				     sizeof(comms_struct));
		if (ret) {
			pr_info("Error at %s(%d)\n", __func__, __LINE__);
			return -EINVAL;
		}

		pr_debug("Transfer. write_not_read = %d, reg_index = 0x%x size = %d\n",
		       comms_struct.write_not_read, comms_struct.reg_index,
		       comms_struct.len);
		// address and buis the same whatever the transfers to be done !
		st_i2c_message.addr = 0x29;
		// st_i2c_message.buf is the same whatever the transfers to be done
		st_i2c_message.buf = raw_data_buffer_1;

		if (!comms_struct.write_not_read) {
			data_ptr = (u8 __user *)(comms_struct.buf);
			comms_struct.buf =
				memdup_user(data_ptr, comms_struct.len);
		}

		// in case of i2c write, it is a single transfer with read index set in the 2 first bytes
		// the other case use fully the raw data buffer for raw data transfers
		if (comms_struct.write_not_read)
			chunk_size = VL53L5CX_COMMS_CHUNK_SIZE - 2;
		else
			chunk_size = VL53L5CX_COMMS_CHUNK_SIZE;

		// index is the number of bytes already transfered
		index = 0;

		do {
			// take the max number of bytes that can be transfered
			transfer_size =
				(comms_struct.len - index) > chunk_size ?
					chunk_size :
					(comms_struct.len - index);

			// ----- WRITE
			if (comms_struct.write_not_read) {
				// put red index at the beginning of the buffer
				raw_data_buffer_1[0] =
					(uint8_t)(((comms_struct.reg_index +
						    index) &
						   0xFF00) >>
						  8);
				raw_data_buffer_1[1] =
					(uint8_t)((comms_struct.reg_index +
						   index) &
						  0x00FF);

				ret = copy_from_user(&raw_data_buffer_1[2],
						     comms_struct.buf + index,
						     transfer_size);
				if (ret) {
					pr_info("Error at %s(%d)\n", __func__,
						__LINE__);
					return -EINVAL;
				}

				st_i2c_message.len = transfer_size + 2;
				st_i2c_message.flags = 0;
				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_1->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_err("Error %d at %s(%d)\n", ret,
					       __func__, __LINE__);
					return -EIO;
				}
			}
			// ----- READ
			else {
				// write reg_index
				st_i2c_message.len = 2;
				st_i2c_message.flags = 0;
				raw_data_buffer_1[0] =
					(uint8_t)(((comms_struct.reg_index +
						    index) &
						   0xFF00) >>
						  8);
				raw_data_buffer_1[1] =
					(uint8_t)((comms_struct.reg_index +
						   index) &
						  0x00FF);

				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_1->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_err("Error at %s(%d)\n", __func__,
					       __LINE__);
					return -EIO;
				}

				st_i2c_message.len = transfer_size;
				st_i2c_message.flags = 1;

				ret = i2c_transfer(
					stmvl53l5cx_i2c_client_1->adapter,
					&st_i2c_message, 1);
				if (ret != 1) {
					pr_err("Error at %s(%d)\n", __func__,
					       __LINE__);
					return -EIO;
				}

				// copy to user buffer the read transfer
				ret = copy_to_user(data_ptr + index,
						   raw_data_buffer_1,
						   transfer_size);

				if (ret) {
					pr_err("Error at %s(%d)\n", __func__,
					       __LINE__);
					return -EINVAL;
				}

			} // ----- READ

			index += transfer_size;

		} while (index < comms_struct.len);
		break;

	default:
		pr_err("stmvl53l5cx_ioctl : cmd err\n");
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations stmvl53l5cx_ranging_fops_0 = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = stmvl53l5cx_ioctl_0,
	.open = stmvl53l5cx_open_0,
	.release = stmvl53l5cx_release_0,
};

static const struct file_operations stmvl53l5cx_ranging_fops_1 = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = stmvl53l5cx_ioctl_1,
	.open = stmvl53l5cx_open_1,
	.release = stmvl53l5cx_release_1,
};
static int stmvl53l5cx_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret;
	const char *compatible;
	uint8_t page = 0, revision_id = 0, device_id = 0;
	// int dev1_lpn_id = 482;

	ret = device_property_read_string(&client->dev, "compatible",
					  &compatible);
	init_waitqueue_head(&wq);

	if (!strcmp("st,stmvl53l5cx_0", compatible)) {
		stmvl53l5cx_i2c_client_0 = client;
		adapt_client = client;
		printk("vl53: addr 0x%x\n", client->addr);
		raw_data_buffer_0 = kzalloc(VL53L5CX_COMMS_CHUNK_SIZE,
					    GFP_DMA | GFP_KERNEL);
		if (raw_data_buffer_0 == NULL)
			return -ENOMEM;
		vl53_gpios[0].intr_gpio_nb = -1;
		vl53_gpios[0].lpn_gpio = -1;
		vl53_gpios[0].power_gpio = -1;
		vl53_gpios[0].rst_gpio = -1;

		vl53_gpios[0].intr_gpio_nb =
			of_get_named_gpio(stmvl53l5cx_i2c_client_0->dev.of_node,
					  "intr_gpio_nb", 0);
		vl53_gpios[0].lpn_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_0->dev.of_node, "lpn_gpio", 0);
		vl53_gpios[0].power_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_0->dev.of_node, "power_gpio", 0);
		vl53_gpios[0].rst_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_0->dev.of_node, "rst_gpio", 0);

		if (vl53_gpios[0].intr_gpio_nb >= 0) {
			ret = get_gpio(vl53_gpios[0].intr_gpio_nb,
				       "intr_gpio_nb", 0);
			if (ret != 0) {
				pr_info("Failed to acquire INTR GPIO(%d)\n",
					vl53_gpios[0].intr_gpio_nb);
			} else {
				gpio_own_flags[0].intr_gpio_owned = 1;

				st_tof_irq_num[0] =
					gpio_to_irq(vl53_gpios[0].intr_gpio_nb);

				// init_waitqueue_head(&wq);

				ret = request_threaded_irq(
					st_tof_irq_num[0], NULL,
					st_tof_intr_handler_0,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"st_tof_sensor", NULL);

				if (ret) {
					dev_err(&stmvl53l5cx_i2c_client_0->dev,
						"stmvl53l5cx: Failed to Register IRQ handler,"
						" GPIO = %d, st_tof_irq_num = %d\n",
						vl53_gpios[0].intr_gpio_nb,
						st_tof_irq_num[0]);
					kfree(raw_data_buffer_0);
					return -EPERM;
				} else {
					dev_err(&stmvl53l5cx_i2c_client_0->dev,
						"Registered IRQ handler, GPIO = %d, "
						"st_tof_irq_num = %d\n",
						vl53_gpios[0].intr_gpio_nb,
						st_tof_irq_num[0]);
					irq_handler_registered_0 = 1;
				}
			}
		}

		if (vl53_gpios[0].power_gpio >= 0) {
			ret = get_gpio(vl53_gpios[0].power_gpio, "power_gpio",
				       1);
			if (ret != 0) {
				pr_info("Failed to acquire power GPIO(%d)\n",
					vl53_gpios[0].power_gpio);
			} else {
				gpio_set_value(vl53_gpios[0].power_gpio, 1);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_0->dev,
				"can't get named gpio: power_gpio");
		}

		if (vl53_gpios[0].rst_gpio >= 0) {
			ret = get_gpio(vl53_gpios[0].rst_gpio, "rst_gpio", 1);
			if (ret != 0) {
				pr_info("Failed to acquire rst GPIO(%d)\n",
					vl53_gpios[0].rst_gpio);
			} else {
				gpio_set_value(vl53_gpios[0].rst_gpio, 0);
				// gpio_direction_output(vl53_gpios[0].rst_gpio,
				// 		      0);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_0->dev,
				"can't get named gpio: rst_gpio");
		}

		if (vl53_gpios[0].lpn_gpio >= 0) {
			ret = get_gpio(vl53_gpios[0].lpn_gpio, "lpn_gpio", 1);
			pr_err("vl53 device 0 lpn gpio_id: %d\n", vl53_gpios[0].lpn_gpio);
			if (ret != 0) {
				pr_info("Failed to acquire lpn GPIO(%d)\n",
					vl53_gpios[0].lpn_gpio);
			} else {
				// gpio_set_value(vl53_gpios[0].lpn_gpio, 1);
				gpio_direction_output(vl53_gpios[0].lpn_gpio,
						      1);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_0->dev,
				"can't get named gpio: lpn_gpio");
		}

		// vl53l5cx_set_i2c_address(0x27 << 1);
		// udelay(10000);

		// ret = stmvl53l5cx_write_multi(stmvl53l5cx_i2c_client_0, raw_data_buffer_0, 0x7FFF, &page, 1);
		// ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0, raw_data_buffer_0, 0x00, &device_id, 1);
		// ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0, raw_data_buffer_0, 0x01, &revision_id, 1);

		// if ((device_id != 0xF0) || (revision_id != 0x02)) {
		//     pr_info("stmvl53l5cx: Error. Could not read device and revision id registers\n");
		//     // return ret;
		// }
		// printk("stmvl53l5cx: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);

		// st_tof_miscdev_0.minor = MISC_DYNAMIC_MINOR;
		// st_tof_miscdev_0.name = "stmvl53l5cx";
		// st_tof_miscdev_0.fops = &stmvl53l5cx_ranging_fops_0;
		// st_tof_miscdev_0.mode = 0444;

		// ret = misc_register(&st_tof_miscdev_0);
		// if (ret) {
		// 	misc_registered_0 = 0;
		// 	pr_err("stmvl53l5cx_0 : Failed to create misc device, err = %d\n",
		// 		ret);
		// 	// return -1;
		// } else {
		// 	misc_registered_0 = 1;
		// 	pr_info("stmvl53l5cx_0 : misc device created\n");
		// }

		return ret;

	} else if (!strcmp("st,stmvl53l5cx_1", compatible)) {
		stmvl53l5cx_i2c_client_1 = client;
		printk("vl53: addr 0x%x\n", client->addr);
		raw_data_buffer_1 = kzalloc(VL53L5CX_COMMS_CHUNK_SIZE,
					    GFP_DMA | GFP_KERNEL);
		if (raw_data_buffer_1 == NULL)
			return -ENOMEM;
		vl53_gpios[1].intr_gpio_nb = -1;
		vl53_gpios[1].lpn_gpio = -1;
		vl53_gpios[1].power_gpio = -1;
		vl53_gpios[1].rst_gpio = -1;

		vl53_gpios[1].intr_gpio_nb =
			of_get_named_gpio(stmvl53l5cx_i2c_client_1->dev.of_node,
					  "intr_gpio_nb", 0);
		vl53_gpios[1].lpn_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_1->dev.of_node, "lpn_gpio", 0);
		vl53_gpios[1].power_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_1->dev.of_node, "power_gpio", 0);
		vl53_gpios[1].rst_gpio = of_get_named_gpio(
			stmvl53l5cx_i2c_client_1->dev.of_node, "rst_gpio", 0);

		if (vl53_gpios[1].intr_gpio_nb >= 0) {
			ret = get_gpio(vl53_gpios[1].intr_gpio_nb,
				       "intr_gpio_nb", 0);
			if (ret != 0) {
				pr_info("Failed to acquire INTR GPIO(%d)\n",
					vl53_gpios[1].intr_gpio_nb);
			} else {
				gpio_own_flags[1].intr_gpio_owned = 1;

				st_tof_irq_num[1] =
					gpio_to_irq(vl53_gpios[1].intr_gpio_nb);

				// init_waitqueue_head(&wq);

				ret = request_threaded_irq(
					st_tof_irq_num[1], NULL,
					st_tof_intr_handler_1,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"st_tof_sensor", NULL);

				if (ret) {
					dev_err(&stmvl53l5cx_i2c_client_1->dev,
						"stmvl53l5cx: Failed to Register IRQ handler,"
						" GPIO = %d, st_tof_irq_num = %d\n",
						vl53_gpios[1].intr_gpio_nb,
						st_tof_irq_num[1]);
					kfree(raw_data_buffer_1);
					return -EPERM;
				} else {
					dev_err(&stmvl53l5cx_i2c_client_1->dev,
						"Registered IRQ handler, GPIO = %d, "
						"st_tof_irq_num = %d\n",
						vl53_gpios[1].intr_gpio_nb,
						st_tof_irq_num[1]);
					irq_handler_registered_1 = 1;
				}
			}
		}

		if (vl53_gpios[1].power_gpio >= 0) {
			ret = get_gpio(vl53_gpios[1].power_gpio, "power_gpio",
				       1);
			if (ret != 0) {
				pr_info("Failed to acquire power GPIO(%d)\n",
					vl53_gpios[1].power_gpio);
			} else {
				gpio_set_value(vl53_gpios[1].power_gpio, 1);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_1->dev,
				"can't get named gpio: power_gpio");
		}

		if (vl53_gpios[1].rst_gpio >= 0) {
			ret = get_gpio(vl53_gpios[1].rst_gpio, "rst_gpio", 1);
			if (ret != 0) {
				pr_info("Failed to acquire rst GPIO(%d)\n",
					vl53_gpios[1].rst_gpio);
			} else {
				gpio_set_value(vl53_gpios[1].rst_gpio, 0);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_1->dev,
				"can't get named gpio: rst_gpio");
		}

		if (vl53_gpios[1].lpn_gpio >= 0) {
			ret = get_gpio(vl53_gpios[1].lpn_gpio, "lpn_gpio", 1);
			pr_err("vl53 device 1 lpn gpio_id: %d\n", vl53_gpios[1].lpn_gpio);
			if (ret != 0) {
				pr_info("Failed to acquire lpn GPIO(%d)\n",
					vl53_gpios[1].lpn_gpio);
			} else {
				gpio_direction_output(vl53_gpios[1].lpn_gpio,
						      0);
				gpio_set_value(vl53_gpios[1].lpn_gpio, 0);
			}
		} else {
			dev_err(&stmvl53l5cx_i2c_client_1->dev,
				"can't get named gpio: lpn_gpio");
		}

		// ret = gpio_request(dev1_lpn_id, "dev1_lpn_id");
		// if(ret) {
		// 	pr_err("vl53 failed to request dev0 lpn pin %d\n", ret);
		// }

		// gpio_direction_output(dev1_lpn_id, 0);
		// gpio_set_value(dev1_lpn_id, 0);

		ret = stmvl53l5cx_write_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x7FFF, &page,
					      1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x00,
					      &device_id, 1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x01,
					      &revision_id, 1);

		if ((device_id != 0xF0) || (revision_id != 0x02)) {
			vl53l5cx_set_i2c_address(0x27 << 1);
			udelay(1000);
			// gpio_direction_output(vl53_gpios[1].lpn_gpio, 1);
			// pr_err("stmvl53l5cx: Error. Could'nt read 0 device and revision id registers\n");
			// return ret;
		} else {
			// printk("stmvl53l5cx: 0 device_id : 0x%x. revision_id : 0x%x\n",
			//        device_id, revision_id);
		}

		gpio_direction_output(vl53_gpios[1].lpn_gpio, 1);

		page = 0;
		revision_id = 0;
		device_id = 0;

		ret = stmvl53l5cx_write_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x7FFF, &page,
					      1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x00,
					      &device_id, 1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_0,
					      raw_data_buffer_0, 0x01,
					      &revision_id, 1);

		if ((device_id != 0xF0) || (revision_id != 0x02)) {
			pr_err("stmvl53l5cx: Error. Could'nt read 0 device and revision id registers\n");
			// return ret;
		} else {
			printk("stmvl53l5cx: 0 device_id : 0x%x. revision_id : 0x%x\n",
			       device_id, revision_id);

			st_tof_miscdev_0.minor = MISC_DYNAMIC_MINOR;
			st_tof_miscdev_0.name = "stmvl53l5cx";
			st_tof_miscdev_0.fops = &stmvl53l5cx_ranging_fops_0;
			st_tof_miscdev_0.mode = 0444;

			ret = misc_register(&st_tof_miscdev_0);
			if (ret) {
				misc_registered_0 = 0;
				pr_err("stmvl53l5cx_0 : Failed to create misc device, err = %d\n",
				       ret);
				// return -1;
			} else {
				misc_registered_0 = 1;
				pr_info("stmvl53l5cx_0 : misc device created\n");
			}
		}

		page = 0;
		revision_id = 0;
		device_id = 0;

		ret = stmvl53l5cx_write_multi(stmvl53l5cx_i2c_client_1,
					      raw_data_buffer_1, 0x7FFF, &page,
					      1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_1,
					      raw_data_buffer_1, 0x00,
					      &device_id, 1);
		ret |= stmvl53l5cx_read_multi(stmvl53l5cx_i2c_client_1,
					      raw_data_buffer_1, 0x01,
					      &revision_id, 1);

		if ((device_id != 0xF0) || (revision_id != 0x02)) {
			pr_err("stmvl53l5cx: Error. Could'nt read 1 device and revision id registers\n");
			// return ret;
		} else {
			printk("stmvl53l5cx: 1 device_id : 0x%x. revision_id : 0x%x\n",
			       device_id, revision_id);
			st_tof_miscdev_1.minor = MISC_DYNAMIC_MINOR;
			st_tof_miscdev_1.name = "stmvl53l5cx_1";
			st_tof_miscdev_1.fops = &stmvl53l5cx_ranging_fops_1;
			st_tof_miscdev_1.mode = 0444;

			ret = misc_register(&st_tof_miscdev_1);
			if (ret) {
				misc_registered_1 = 0;
				pr_err("stmvl53l5cx_1 : Failed to create misc device, err = %d\n",
					ret);
				// return -1;
			} else {
				misc_registered_1 = 1;
				pr_info("stmvl53l5cx_1 : misc device created\n");
			}

			return ret;
		}

	}

	return 0;
}

static int stmvl53l5cx_remove(struct i2c_client *client)
{
	if (raw_data_buffer_0)
		kfree(raw_data_buffer_0);
	return 0;
}

static struct i2c_driver stmvl53l5cx_i2c_driver = {
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(st_tof_of_match), // for platform register to pick up the dts info
	},
	.probe = stmvl53l5cx_probe,
	.remove = stmvl53l5cx_remove,
};

static int __init st_tof_module_init(void)
{
	int ret = 0;

	printk("stmvl53l5cx: module init\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l5cx_i2c_driver);

	if (ret) {
		i2c_del_driver(&stmvl53l5cx_i2c_driver);
		printk("stmvl53l5cx: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

	return ret;
}

static void __exit st_tof_module_exit(void)
{
	pr_debug("stmvl53l5cx : module exit\n");

	if (misc_registered_0) {
		misc_deregister(&st_tof_miscdev_0);
		misc_registered_0 = 0;
	}
	if (misc_registered_1) {
		misc_deregister(&st_tof_miscdev_1);
		misc_registered_1 = 0;
	}

	if (i2c_driver_added) {
		i2c_del_driver(&stmvl53l5cx_i2c_driver);
		i2c_driver_added = 0;
	}

	if (irq_handler_registered_0) {
		free_irq(st_tof_irq_num[0], NULL);
	}

	if (irq_handler_registered_1) {
		free_irq(st_tof_irq_num[1], NULL);
	}

	if (gpio_own_flags[0].intr_gpio_owned == 1) {
		gpio_set_value(vl53_gpios[0].lpn_gpio, 0);
		put_gpio(vl53_gpios[0].intr_gpio_nb);
		put_gpio(vl53_gpios[0].rst_gpio);
		put_gpio(vl53_gpios[0].power_gpio);
		put_gpio(vl53_gpios[0].lpn_gpio);
	}

	if (gpio_own_flags[1].intr_gpio_owned == 1) {
		gpio_set_value(vl53_gpios[1].lpn_gpio, 0);
		put_gpio(vl53_gpios[1].intr_gpio_nb);
		put_gpio(vl53_gpios[1].rst_gpio);
		put_gpio(vl53_gpios[1].power_gpio);
		put_gpio(vl53_gpios[1].lpn_gpio);
	}
}

module_init(st_tof_module_init);
module_exit(st_tof_module_exit);
MODULE_LICENSE("GPL");
