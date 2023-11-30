#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
// #include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include "sht30.h"

int32_t sht30_read_multi(struct i2c_client *client,
	uint8_t * i2c_buffer,
	uint16_t reg_index,
	uint8_t *pdata,
	uint32_t count)
{
	int32_t status = 0;
	uint32_t position = 0;
	uint32_t data_size = 0;
	struct i2c_msg message;

	message.addr  = 0x44;

	do {
		data_size = (count - position) > SHT30_COMMS_CHUNK_SIZE ? SHT30_COMMS_CHUNK_SIZE : (count - position);

		i2c_buffer[0] = (reg_index + position) >> 8;
		i2c_buffer[1] = (reg_index + position) & 0xFF;

		message.flags = 0;
		message.buf   = i2c_buffer;
		message.len   = 2;

		status = i2c_transfer(client->adapter, &message, 1);
		if (status != 1)
			return -EIO;

		message.flags = 1;
		message.buf   = pdata + position;
		message.len   = data_size;

		status = i2c_transfer(client->adapter, &message, 1);
		if (status != 1)
			return -EIO;

		position += data_size;

        udelay(1000);

	} while (position < count);
    
    return 0;
}


int32_t sht30_write_multi(struct i2c_client *client,
		uint8_t *i2c_buffer,
		uint16_t reg_index,
		uint8_t *pdata,
		uint32_t count)
{
	int32_t status = 0;
	uint32_t position = 0;
	int32_t data_size = 0;
	struct i2c_msg message;

	message.addr  = 0x44;

	do {
		data_size = (count - position) > (SHT30_COMMS_CHUNK_SIZE-2) ? (SHT30_COMMS_CHUNK_SIZE-2) : (count - position);

		memcpy(&i2c_buffer[2], &pdata[position], data_size);

		i2c_buffer[0] = (reg_index + position) >> 8;
		i2c_buffer[1] = (reg_index + position) & 0xFF;

		message.flags = 0;
		message.len   = data_size + 2;
		message.buf   = i2c_buffer;

		status = i2c_transfer(client->adapter, &message, 1);
		if (status != 1)
			return -EIO;

		position +=  data_size;

        udelay(1000);

	} while (position < count);

	return 0;
}

int32_t sht30_write_byte(
	struct i2c_client *client, uint8_t * i2c_buffer, uint16_t address, uint8_t value)
{
	return sht30_write_multi(client, i2c_buffer, address, &value, 1);
}

int32_t sht30_read_byte(
	struct i2c_client *client, uint8_t * i2c_buffer, uint16_t address, uint8_t *p_value)
{
	return sht30_read_multi(client, i2c_buffer, address, p_value, 1);
}

int32_t sht30_read_no_regaddr(
	struct i2c_client *client, 
    uint8_t *p_value, 
    uint32_t count)
{
    int32_t status = 0;
    struct i2c_msg message;

    message.addr  = 0x44;
    message.flags = 1;
		message.buf   = p_value;
		message	.len   = count;

    status = i2c_transfer(client->adapter, &message, 1);
    //printk("read no addr status: %d\n", status);
    if (status != 1)
		return status;
    udelay(1000);
		return 0;
}

int32_t sht30_write_no_regaddr(
	struct i2c_client *client, 
    uint8_t *p_value, 
    uint32_t count)
{
    int32_t status = 0;

    struct i2c_msg message;

    message.addr  = 0x44;
    message.flags = 0;
    message.buf   = p_value;
    message.len   = count;

    status = i2c_transfer(client->adapter, &message, 1);
    //printk("write no addr status: %d\n", status);
    if (status != 1)
		return status;

    udelay(1000);

	return 0;
}
