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

#ifndef STMVL53L5CX_ADAPT_H
#define STMVL53L5CX_ADAPT_H

#define VL53L5CX_STATUS_OK ((uint8_t)0U)
#define VL53L5CX_COMMS_ERROR -2

#define VL53L5CX_OFFSET_BUFFER_SIZE ((uint16_t)488U)
#define VL53L5CX_XTALK_BUFFER_SIZE ((uint16_t)776U)
#define VL53L5CX_TEMPORARY_BUFFER_SIZE ((uint32_t)1024U)

#define VL53L5CX_COMMS_CHUNK_SIZE 1024

static uint8_t i2c_buffer[VL53L5CX_COMMS_CHUNK_SIZE];

struct i2c_client *adapt_client;

struct comms_struct {
	uint16_t len;
	uint16_t reg_address;
	uint8_t *buf;
	uint8_t write_not_read;
};

/**
 * @brief Structure VL53L5CX_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L5CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct {
	/* To be filled with customer's platform. At least an I2C address/descriptor
	 * needs to be added */
	/* Example for most standard platform : I2C address of sensor */
	uint16_t address;

	/* For Linux implementation, file descriptor */
	int fd;

} VL53L5CX_Platform;

/**
 * @brief Structure VL53L5CX_Configuration contains the sensor configuration.
 * User MUST not manually change these field, except for the sensor address.
 */

typedef struct {
	/* Platform, filled by customer into the 'platform.h' file */
	VL53L5CX_Platform platform;
	/* Results streamcount, value auto-incremented at each range */
	uint8_t streamcount;
	/* Size of data read though I2C */
	uint32_t data_read_size;
	/* Address of default configuration buffer */
	uint8_t *default_configuration;
	/* Address of default Xtalk buffer */
	uint8_t *default_xtalk;
	/* Offset buffer */
	uint8_t offset_data[VL53L5CX_OFFSET_BUFFER_SIZE];
	/* Xtalk buffer */
	uint8_t xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
	/* Temporary buffer used for internal driver processing */
	uint8_t temp_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
	/* Auto-stop flag for stopping the sensor */
	uint8_t is_auto_stop_enabled;
} VL53L5CX_Configuration;

int32_t adapt_write_multi(uint16_t reg_addr, uint16_t reg_index, uint8_t *pdata,
			  uint32_t count)
{
	int32_t status = 0;
	uint32_t position = 0;
	int32_t data_size = 0;
	struct i2c_msg message;

	message.addr = reg_addr;
	adapt_client->addr = reg_addr;

	do {
		data_size =
			(count - position) > (VL53L5CX_COMMS_CHUNK_SIZE - 2) ?
				(VL53L5CX_COMMS_CHUNK_SIZE - 2) :
				(count - position);

		memcpy(&i2c_buffer[2], &pdata[position], data_size);

		i2c_buffer[0] = (reg_index + position) >> 8;
		i2c_buffer[1] = (reg_index + position) & 0xFF;

		message.flags = 0;
		message.len = data_size + 2;
		message.buf = i2c_buffer;

		status = i2c_transfer(adapt_client->adapter, &message, 1);
		if (status != 1)
			return -EIO;

		position += data_size;

	} while (position < count);

	return 0;
}

uint8_t vl53l5cx_set_i2c_address(uint16_t i2c_address)
{
	uint8_t reg_data;
	uint8_t status = VL53L5CX_STATUS_OK;

	reg_data = 0x00;
	status = adapt_write_multi(0x29, 0x7fff, &reg_data, 1);
	reg_data = (uint8_t)(i2c_address >> 1);
	status = adapt_write_multi(0x29, 0x4, &reg_data, 1);
	reg_data = 0x02;
	status = adapt_write_multi(0x27, 0x7fff, &reg_data, 1);

	return status;
}
#endif
