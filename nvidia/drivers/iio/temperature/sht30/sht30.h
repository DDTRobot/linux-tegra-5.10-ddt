#include "types.h"

#ifndef SHT30_I2C_H
#define SHT30_I2C_H

#define SHT30_COMMS_CHUNK_SIZE  1024

static const uint16_t SHT3X_CMD_READ_STATUS_REG = 0xF32D;

#define SHT3X_CMD_MEASURE_HPM 0x2C06

#define SHT3X_MEASUREMENT_DURATION_USEC 15000

int32_t sht30_write_byte(
	struct i2c_client *client,
	uint8_t *i2c_buffer,
	uint16_t address,
	uint8_t value);

int32_t sht30_read_byte(
	struct i2c_client *client,
	uint8_t *i2c_buffer,
	uint16_t address,
	uint8_t *p_value);

int32_t sht30_read_multi(
	struct i2c_client *client,
	uint8_t *i2c_buffer,
	uint16_t reg_index,
	uint8_t *pdata,
	uint32_t count);

int32_t sht30_write_multi(
	struct i2c_client *client,
	uint8_t *i2c_buffer,
	uint16_t reg_index,
	uint8_t *pdata,
	uint32_t count);

int32_t sht30_read_no_regaddr(
	struct i2c_client *client, 
	uint8_t *p_value, 
	uint32_t count);
int32_t sht30_write_no_regaddr(
	struct i2c_client *client, 
    uint8_t *p_value, 
    uint32_t count);

#endif