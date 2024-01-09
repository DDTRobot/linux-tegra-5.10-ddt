/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#ifndef __CHIRP_DVB_H
#define __CHIRP_DVB_H

#include <linux/device.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/tegra-gte.h>
#include <linux/bitops.h>

#include "inc/ch201_gprmt.h"

#define  CH201_COMMS_CHUNK_SIZE  1024

#define CHIRP_I2C_ADDRS		    0x45
#define CHIRP_APP_I2C_ADDRS		0x29
#define CHIRP_I2C_BUSES		    0

#define CHBSP_RTC_CAL_PULSE_MS	100     // length of pulse applied to sensor INT line during clock cal, in ms

#ifdef CHIRP_MAX_NUM_SENSORS
#define CHBSP_MAX_DEVICES 		CHIRP_MAX_NUM_SENSORS
#else
#define CHBSP_MAX_DEVICES 		1
#endif

#ifdef CHIRP_NUM_I2C_BUSES
#define CHBSP_NUM_I2C_BUSES		CHIRP_NUM_I2C_BUSES
#else
#define CHBSP_NUM_I2C_BUSES 	1
#endif

#define I2C_DRV_FLAGS	(I2C_DRV_FLAG_RESET_AFTER_NB | I2C_DRV_FLAG_USE_PROG_NB)


#if 0

#define  CH201_COMMS_CHUNK_SIZE  1024

#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprmt_init        /* CH201 GPR Multi-Threshold firmware */

/* Sensor and I2C bus counts - normally taken from chirp_config.h */
#ifdef CHIRP_MAX_NUM_SENSORS
#define CHBSP_MAX_DEVICES 		CHIRP_MAX_NUM_SENSORS
#else
#define CHBSP_MAX_DEVICES 		1
#endif

#ifdef CHIRP_NUM_I2C_BUSES
#define CHBSP_NUM_I2C_BUSES		CHIRP_NUM_I2C_BUSES
#else
#define CHBSP_NUM_I2C_BUSES 	1
#endif

/* RTC calibration pulse length */
#define CHBSP_RTC_CAL_PULSE_MS	100     // length of pulse applied to sensor INT line during clock cal, in ms

/* I2C Addresses and bus index for each possible device, indexed by device number */
#define CHIRP_I2C_ADDRS		    0x45
#define CHIRP_APP_I2C_ADDRS		0x29
#define CHIRP_I2C_BUSES		    0

/* I2C bus speed */
#define I2C_BUS_SPEED		(400000)
#define I2C_TIMEOUT	2000    // XXX move

/* Flags for special I2C handling by Chirp driver */
#define I2C_DRV_FLAGS	(I2C_DRV_FLAG_RESET_AFTER_NB | I2C_DRV_FLAG_USE_PROG_NB)        // reset i2c interface after non-blocking

// #define	MEASUREMENT_INTERVAL_MS		100     // 100ms interval = 10Hz sampling
#define	MEASUREMENT_INTERVAL_MS		20     // 20ms interval = 50Hz sampling

/*============================ Sensor Configuration =============================*/

/* Define configuration settings for the Chirp sensors 
 *   The following symbols define configuration values that are used to 
 *   initialize the ch_config_t structure passed during the ch_set_config() 
 *   call.  
 */
// #define	CHIRP_SENSOR_MAX_RANGE_MM		2000    /* maximum range, in mm */
#define	CHIRP_SENSOR_MAX_RANGE_MM		1000    /* maximum range, in mm */

#define	CHIRP_SENSOR_STATIC_RANGE		0       /* static target rejection sample 
                                                           range, in samples (0=disabled) */
#define CHIRP_SENSOR_SAMPLE_INTERVAL	0       /* internal sample interval - 
                                                   NOT USED IF TRIGGERED */

#endif

#endif /* __CHIRP_DVB_H */