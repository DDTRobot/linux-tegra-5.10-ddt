#define DT_DRV_COMPAT st_ch201

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
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

#include "app_config.h"
#include "chirp_dvb.h"
#include "types.h"

#define CHIRP_NAME			"ch201"
#define CHIRP_PART_CH201			(0)

#define SEN_ULTRASONIC_IOCTL_MEASURE 		_IOWR('c',0x1, void*)
#define SEN_ULTRASONIC_IOCTL_DEVID 			_IOWR('c',0x2, void*)

static struct miscdevice sen_ulsonic_miscdev;

/* Bit flags used in main loop to check for completion of I/O or timer operations.  */
#define DATA_READY_FLAG		(1 << 0)		// data ready from sensor
#define IQ_READY_FLAG		(1 << 1)		// non-blocking I/Q read has completed
#define TIMER_FLAG			(1 << 2)		// period timer has interrupted

static unsigned int tdk_ultra_irq_num;


uint8_t interrupt_flag = 0;

static uint8_t irq_handler_registered = 0;

static uint32_t active_devices;

/* Number of connected sensors */
static uint8_t	num_connected_sensors = 0;

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags
 *   that are set in I/O processing routines.  The flags are checked in the
 *   main() loop and, if set, will cause an appropriate handler function to
 *   be called to process sensor data.
 */
volatile uint32_t taskflags = 0;

static uint8_t misc_registered = 0;

static uint8_t i2c_driver_added = 0;

struct ch201_comms_struct {
	__u16   len;
	__u8    *buf;
};

ch_thresholds_t chirp_ch201_thresholds = { 
	.threshold = {
		{0, 	5000},		/* threshold 0 */
		{26,	2000},		/* threshold 1 */
		{39,	800},		/* threshold 2 */
		{56,	400},		/* threshold 3 */
		{79,	250},		/* threshold 4 */
		{89,	175}		/* threshold 5 */
	}
};   

/* Forward declarations */
static uint8_t  handle_data_ready(ch_group_t *grp_ptr);

static const struct i2c_device_id chirp_i2c_device_ids[] = {
	{ CHIRP_NAME, CHIRP_PART_CH201 },
	{},
};

MODULE_DEVICE_TABLE(i2c, chirp_i2c_device_ids);

static const struct of_device_id chirp_of_match[] = {
	{ .compatible = "tdk,ch201", },
	{}
};

MODULE_DEVICE_TABLE(of, chirp_of_match);

static uint8_t chirp_i2c_addrs[] = {CHIRP_I2C_ADDRS};
static uint8_t chirp_i2c_buses[] = {CHIRP_I2C_BUSES};
struct ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS];

struct chirp_data_t{
  uint32_t range;               // from ch_get_range()
  uint16_t amplitude;           // from ch_get_amplitude()
  uint16_t num_samples;         // from ch_get_num_samples()
};

/* Array of structs to hold measurement data, one for each possible device */
struct chirp_data_t	chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t	chirp_devices[CHIRP_MAX_NUM_SENSORS];

ch_group_t chirp_group;

/*!
 * \brief Initialize the host's I2C hardware.
 * Return 0 if successful, non-zero otherwise
 */
int chbsp_i2c_init(void)
{
    return 0;
}

uint32_t chbsp_timestamp_ms(void)
{
	uint32_t time_get;
	struct timespec64 ts;

	ktime_get_ts64(&ts);
	time_get = timespec64_to_ns(&ts);
	do_div(time_get, 1000000); /* ns => ms */

	return time_get;
}

/*!
 * \brief Write a byte to the slave.  Function assumes start condition has been issued.
 *
 * \param data Data byte to be transmitted.
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * \note The blocking-mode driver does not always seem to wait for the bus to be free.  This function checks the busy flag
 * explicitly.
 */
int chbsp_i2c_write(ch_dev_t * dev_ptr, uint8_t * data, uint16_t n)
{
    struct i2c_msg msg;
	int ret = 0;

	msg.addr = ch201_data->i2c_addr;
	msg.flags = 0;
	msg.len = n;
	msg.buf = data;
	ret = i2c_transfer(ch201_data->i2c->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_err(&ch201_data->i2c->dev, " ch201 w err\n");
		ret = -EIO;
	}

	return ret;
}

int chbsp_i2c_mem_write(ch_dev_t * dev_ptr, uint16_t mem_addr,
                        uint8_t * data_ptr, uint16_t num_bytes)
{
    struct i2c_msg msg;
	uint8_t trans_buf[CH201_COMMS_CHUNK_SIZE];
	int ret = 0;

	if((mem_addr >> 8) == 0)
	{
		trans_buf[0] = mem_addr;

		memcpy(&trans_buf[1], data_ptr, num_bytes);

		msg.addr = ch201_data->i2c_addr;
		msg.flags = 0;
		msg.buf = trans_buf;
		msg.len = num_bytes + 1;

	} else {
		trans_buf[0] = mem_addr >> 8;
		trans_buf[1] = mem_addr & 0xFF;

		memcpy(&trans_buf[2], data_ptr, num_bytes);

		msg.addr = ch201_data->i2c_addr;
		msg.flags = 0;
		msg.buf = trans_buf;
		msg.len = num_bytes + 2;
	}

	ret = i2c_transfer(ch201_data->i2c->adapter, &msg, 1);
	if (ret == 1)
	{
		ret = 0;
	} else {
		dev_err(&ch201_data->i2c->dev, " ch201 w16 err\n");
		ret = -EIO;
	}

    return ret;
}

int chbsp_i2c_write_nb(ch_dev_t * dev_ptr, uint8_t * data, uint16_t n)
{
    return 0;
}

int chbsp_i2c_mem_write_nb(ch_dev_t * dev_ptr, uint16_t mem_addr,
                           uint8_t * data, uint16_t len)
{
    return 0;
}

/*!
 * \brief Read a specified number of bytes from an I2C slave.
 *
 * \param data Pointer to receive data buffer.
 * \param length Number of bytes to read.
 * \return Currently, always 0.  TODO: add error checking
 *
 * \note The blocking-mode driver does not always seem to wait for the bus to be free.  This function checks the busy flag
 * explicitly.
 */
int chbsp_i2c_read(ch_dev_t * dev_ptr, uint8_t * data, uint16_t len)
{
    struct i2c_msg msg;
	int ret = 0;

	msg.addr = ch201_data->i2c_addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(ch201_data->i2c->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_err(&ch201_data->i2c->dev, " ch201 r err\n");
		ret = -EIO;
	}

	return ret;
}

int chbsp_i2c_mem_read(ch_dev_t * dev_ptr, uint16_t mem_addr,
                       uint8_t * data_ptr, uint16_t num_bytes)
{
	struct i2c_msg msg;
	uint8_t reg_addr[2];
	int ret = 0;

	reg_addr[0] = mem_addr >> 8;
	reg_addr[1] = mem_addr & 0xFF;

	if(reg_addr[0] == 0)
	{
		msg.addr = ch201_data->i2c_addr;
		msg.flags = 0;
		msg.buf = reg_addr + 1;
		msg.len = 2;

	} else {
		msg.addr = ch201_data->i2c_addr;
		msg.flags = 0;
		msg.buf = reg_addr;
		msg.len = 2;
	}

	ret = i2c_transfer(ch201_data->i2c->adapter, &msg, 1);
	if (ret == 1)
	{
		ret = 0;
	} else {
		dev_err(&ch201_data->i2c->dev, " ch201 r16-w err\n");
		ret = -EIO;
	}
		
	msg.addr = ch201_data->i2c_addr;
	msg.flags = 1;
	msg.buf = data_ptr;
	msg.len = num_bytes;

	ret = i2c_transfer(ch201_data->i2c->adapter, &msg, 1);
	if (ret == 1)
	{
		ret = 0;
	} else {
		dev_err(&ch201_data->i2c->dev, " ch201 r16-r err\n");
		ret = -EIO;
	}

    return ret;
}

int chbsp_i2c_read_nb(ch_dev_t * dev_ptr, uint8_t * data, uint16_t len)
{
    return 0;
}

int chbsp_i2c_mem_read_nb(ch_dev_t * dev_ptr, uint16_t mem_addr, uint8_t * data,
                          uint16_t len)
{
    return 0;
}

void chbsp_print_str(char *str)
{
}

void chbsp_i2c_reset(ch_dev_t * dev_ptr)
{
}

/*!
 * \brief Busy wait delay for us microseconds
 *
 * \note
 */
void chbsp_delay_us(uint32_t us)
{
    udelay(us);
}

/*!
 * \brief Busy wait delay for ms milliseconds
 *
 * \note
 */
void chbsp_delay_ms(uint32_t ms)
{
    mdelay(ms);
}

/* Interrupt handler */
static irqreturn_t tdk_ultra_intr_handler(int st_tof_irq_num, void *dev_id)
{
	return IRQ_HANDLED;
}

/*!
 * \brief Assert the reset pin
 *
 * \note This function should drive the Chirp sensor reset pin low.
 */
void chbsp_reset_assert(void)
{
	if (ch201_data->i2c_rst_gpio >= 0) {
        gpio_set_value(ch201_data->i2c_rst_gpio, 0);
		if(gpio_get_value(ch201_data->i2c_rst_gpio) != 0)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 rst low err.");
		} else {
			dev_err(&ch201_data->i2c->dev, "ch201 rst low success.");
		}
	}
}

/*!
 * \brief Deassert the reset pin
 *
 * \note This function should drive the Chirp sensor reset pin high (or open drain if there is a pull-up).
 */
void chbsp_reset_release(void)
{
	if (ch201_data->i2c_rst_gpio >= 0) {
        gpio_set_value(ch201_data->i2c_rst_gpio, 1);
		if(gpio_get_value(ch201_data->i2c_rst_gpio) != 1)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 reset high err.");
		} else {
			dev_err(&ch201_data->i2c->dev, "ch201 reset high success.");
		}
	}
}

/*!
 * \brief Assert the PROG pin
 *
 * \note This function should drive the Chirp sensor PROG pin high on the specified port.
 */
void chbsp_program_enable(ch_dev_t * dev_ptr)
{
	if (ch201_data->prog_gpio >= 0) {
        gpio_set_value(ch201_data->prog_gpio, 1);
		if(gpio_get_value(ch201_data->prog_gpio) != 1)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 prog high err.");
		} else {
			//dev_err(&ch201_data->i2c->dev, "ch201 prog high success.");
		}
	}
}

/*!
 * \brief Deassert the PROG pin
 *
 * \note This function should drive the Chirp sensor PROG pin low on the specified port.
 */
void chbsp_program_disable(ch_dev_t * dev_ptr)
{
	if (ch201_data->prog_gpio >= 0) {
        gpio_set_value(ch201_data->prog_gpio, 0);
		if(gpio_get_value(ch201_data->prog_gpio) != 0)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 prog low err.");
		} else {
			//dev_err(&ch201_data->i2c->dev, "ch201 prog low success.");
		}
	}
}

/*!
 * \brief Configure the host side of the CH201 interrupt pin as an output
 *
 * \note
 */
void chbsp_group_set_io_dir_out(ch_group_t * grp_ptr)
{
	int8_t ret;

	if (ch201_data->irq_gpio >= 0) {
		ret = gpio_direction_output(ch201_data->irq_gpio, 0);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}
	}
}

void chbsp_set_io_dir_out(ch_dev_t * dev_ptr)
{
  	int8_t ret;

	if (ch201_data->irq_gpio >= 0) {
		ret = gpio_direction_output(ch201_data->irq_gpio, 0);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_outp(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}
	}
}

/*!
 * \brief Configure the host side of the CH201 interrupt pin as an input
 *
 * \note This function assumes a bidirectional level shifter is interfacing the ICs.
 */
void chbsp_group_set_io_dir_in(ch_group_t * grp_ptr)
{
    int8_t ret;

	if (ch201_data->irq_gpio >= 0) {
		ret = gpio_direction_input(ch201_data->irq_gpio);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}
	}
}

void chbsp_set_io_dir_in(ch_dev_t * dev_ptr)
{
	int8_t ret;

	if (ch201_data->irq_gpio >= 0) {
		ret = gpio_direction_input(ch201_data->irq_gpio);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}
	}
}

/*!
 * \brief Initialize the I/O pins.
 *
 * Configure reset and program pins as outputs. Assert reset and program. Configure IO pin as input.
 */
void chbsp_group_pin_init(ch_group_t * grp_ptr)
{
    uint8_t ret;

	dev_err(&ch201_data->i2c->dev, "ch201 enter chbsp_group_pin_init");

    /* default device specific parameters */
	ch201_data->irq_gpio = -1;
    ch201_data->prog_gpio = -1;
    ch201_data->i2c_rst_gpio = -1;
	/* device tree parameters */
	ch201_data->irq_gpio = of_get_named_gpio(ch201_data->i2c->dev.of_node, "irq_gpios", 0);
	if (ch201_data->irq_gpio < 0) {
		dev_err(&ch201_data->i2c->dev,
				"%s gpio irq get named(%d) \n",
				__func__, ch201_data->irq_gpio);
	}
    ch201_data->prog_gpio = of_get_named_gpio(ch201_data->i2c->dev.of_node, "prog_gpios", 0);
	if (ch201_data->prog_gpio < 0) {
		dev_err(&ch201_data->i2c->dev,
				"%s gpio prog get named(%d) \n",
				__func__, ch201_data->prog_gpio);
	}
    ch201_data->i2c_rst_gpio = of_get_named_gpio(ch201_data->i2c->dev.of_node, "i2c_rst_gpios", 0);
	if (ch201_data->i2c_rst_gpio < 0) {
		dev_err(&ch201_data->i2c->dev,
				"%s gpio rst get named(%d) \n",
				__func__, ch201_data->i2c_rst_gpio);
	}

    if (ch201_data->irq_gpio >= 0) {
		ret = gpio_request(ch201_data->irq_gpio, "irq_gpio");
		if (ret) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_request(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}

		ret = gpio_direction_input(ch201_data->irq_gpio);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, ret);
		}

		tdk_ultra_irq_num = gpio_to_irq(ch201_data->irq_gpio);
		if (tdk_ultra_irq_num <= 0) {
			dev_err(&ch201_data->i2c->dev, 
				"%s gpio_to_irq(%d) ERR:%d\n",
				__func__, ch201_data->irq_gpio, tdk_ultra_irq_num);
		}

		ret = request_threaded_irq(tdk_ultra_irq_num,
						NULL,
						tdk_ultra_intr_handler,
						IRQF_TRIGGER_RISING|IRQF_ONESHOT,
						"st_tof_sensor",
						NULL);

		if (ret) {
			pr_err("ch201: Failed to Register IRQ handler,"
					" GPIO = %d, st_tof_irq_num = %d\n",
					ch201_data->irq_gpio, tdk_ultra_irq_num);
		} else {
			printk("ch201: Registered IRQ handler, GPIO = %d, "
					"st_tof_irq_num = %d\n",
					ch201_data->irq_gpio, tdk_ultra_irq_num);
			irq_handler_registered = 1;
		}
    }

    if (ch201_data->prog_gpio >= 0) {
		ret = gpio_request(ch201_data->prog_gpio, "prog_gpio");
		if (ret) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_request(%d %s) ERR:%d\n",
				__func__, ch201_data->prog_gpio, CHIRP_NAME, ret);
		}

		ret = gpio_direction_output(ch201_data->prog_gpio, 0);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_prog(%d) ERR:%d\n",
				__func__, ch201_data->prog_gpio, ret);
		}
    }

    if (ch201_data->i2c_rst_gpio >= 0) {
		ret = gpio_request(ch201_data->i2c_rst_gpio, "i2c_rst_gpio");
		if (ret) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_request(%d %s) ERR:%d\n",
				__func__, ch201_data->i2c_rst_gpio, CHIRP_NAME, ret);
		}

		ret = gpio_direction_output(ch201_data->i2c_rst_gpio, 0);
		if (ret < 0) {
			dev_err(&ch201_data->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, ch201_data->i2c_rst_gpio, ret);
		}
    }

}

/*!
 * \brief Set the IO pin low.
 *
 * \note If directly coupled to the Chirp sensor it is recommended to use a passive pull-down to ensure the supply is never shorted through the I/O.
 */
void chbsp_group_io_clear(ch_group_t * grp_ptr)
{

	if (ch201_data->irq_gpio >= 0) {
        gpio_set_value(ch201_data->irq_gpio, 0);
		if(gpio_get_value(ch201_data->irq_gpio) != 0)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 irq low err.");
		} else {
			//dev_err(&ch201_data->i2c->dev, "ch201 irq low success.");
		}
	}
}


/*!
 * \brief Set the IO pin high.
 *
 * \note
 */
void chbsp_group_io_set(ch_group_t * grp_ptr)
{
    if (ch201_data->irq_gpio >= 0) {
        gpio_set_value(ch201_data->irq_gpio, 1);
		if(gpio_get_value(ch201_data->irq_gpio) != 1)
		{
			dev_err(&ch201_data->i2c->dev, "ch201 irq high err.");
		} else {
			//dev_err(&ch201_data->i2c->dev, "ch201 irq high success.");
		}
	}
}

/*!
 * \brief Enable the interrupt
 *
 * \note
 */
void chbsp_io_interrupt_enable(ch_dev_t * dev_ptr)
{
	dev_err(&ch201_data->i2c->dev, "ch201: enter chbsp_io_interrupt_enable.");
}

void chbsp_group_io_interrupt_enable(ch_group_t * grp_ptr)
{
    uint8_t i;

    for (i = 0; i < grp_ptr->num_ports; i++) {
        chbsp_io_interrupt_enable(grp_ptr->device[i]);
    }
}

/*!
 * \brief Disable the interrupt
 *
 * \note
 */
void chbsp_io_interrupt_disable(ch_dev_t * dev_ptr)
{
	dev_err(&ch201_data->i2c->dev, "ch201: enter chbsp_io_interrupt_disable.");
}

void chbsp_group_io_interrupt_disable(ch_group_t * grp_ptr)
{
    uint8_t i;
    for (i = 0; i < grp_ptr->num_ports; i++) {
        chbsp_io_interrupt_disable(grp_ptr->device[i]);
    }
}

/*
 * This function is called to obtain the I2C address and bus info for a device specified by the
 * group and I/O index value.
 *
 * Note: grp_ptr is not used by this implementation - all I2C addresses are from same set
 */
uint8_t chbsp_i2c_get_info(ch_group_t * grp_ptr, uint8_t io_index,
                           ch_i2c_info_t * info_ptr)
{
  uint8_t ret_val = 1;

  if (io_index < CHBSP_MAX_DEVICES) {
    info_ptr->address = chirp_i2c_addrs[io_index];
    info_ptr->bus_num = chirp_i2c_buses[io_index];

    info_ptr->drv_flags = I2C_DRV_FLAGS;        // i2c driver special handling flags, from board header file

    ret_val = 0;
  }

  return ret_val;
}

/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */
void chbsp_board_init(ch_group_t * grp_ptr)
{

  /* Make local copy of group pointer */
  ch201_data->chirp_group = grp_ptr;

  /* Initialize group descriptor */
  grp_ptr->num_ports = CHBSP_MAX_DEVICES;
  grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;

}

/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual 
 * sensor.  The operating mode, maximum range, and static target rejection 
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also 
 * displayed.
 */
static uint8_t display_config_info(ch_dev_t * dev_ptr)
{
	ch_config_t read_config;
	uint8_t chirp_error;	 
	ch_thresholds_t read_thresholds;
	int i;
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

	/* Read configuration values for the device into ch_config_t structure */
	chirp_error = ch_get_config(dev_ptr, &read_config);

	if (!chirp_error) {
		char *mode_string;

		switch (read_config.mode) {
		case CH_MODE_IDLE:
		mode_string = "IDLE";
		break;
		case CH_MODE_FREERUN:
		mode_string = "FREERUN";
		break;
		case CH_MODE_TRIGGERED_TX_RX:
		mode_string = "TRIGGERED_TX_RX";
		break;
		case CH_MODE_TRIGGERED_RX_ONLY:
		mode_string = "TRIGGERED_RX_ONLY";
		break;
		default:
		mode_string = "UNKNOWN";
		}

		/* Display sensor number, mode and max range */
		printk("ch201 Sensor %d:\tmax_range=%dmm \tmode=%s  \r\n", dev_num,
			read_config.max_range, mode_string);

		/* Display static target rejection range, if used */
		if (read_config.static_range != 0) {
		printk("ch201 static_range=%d samples \r\n", read_config.static_range);
		}

		/* Display detection thresholds (only supported on CH201) */
		if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {

		/* Get threshold values in structure */
		chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

		if (!chirp_error) {
			printk("\r\n  Detection thresholds:\r\n");
			for (i = 0; i < CH_NUM_THRESHOLDS; i++) {
			printk("ch201     %d\tstart: %2d\tlevel: %d\r\n", i,
					read_thresholds.threshold[i].start_sample,
					read_thresholds.threshold[i].level);
			}
		} else {
			printk("ch201 Device %d: Error during ch_get_thresholds()\r\n", dev_num);
		}
		}
		printk("\n");

	} else {
		printk("ch201 Device %d: Error during ch_get_config()\r\n", dev_num);
	}

	return chirp_error;
}

static uint8_t handle_data_ready(ch_group_t *grp_ptr) {
	uint8_t 	dev_num;
	struct ch_dev_t 	*dev_ptr;
	int 		num_samples = 0;
    uint8_t 	ret_val = 0;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
			} else {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);
			}

			if (chirp_data[dev_num].range == CH_NO_TARGET) {
				// printk("ch201: CH_NO_TARGET: %d", dev_num);
				chirp_data[dev_num].amplitude = 0;
			} else {
				chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
				// printk("ch201 Port %d:  Range: %d mm  Amplitude: %u  \r\n",
				// 	dev_num, chirp_data[dev_num].range / 32,
				// 	chirp_data[dev_num].amplitude);
			}

			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;
		}
	}
	interrupt_flag = 1;
	return ret_val;
}

static int ch201_open(struct inode *inode, struct file *file)
{
	pr_debug("ch201 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int ch201_release(struct inode *inode, struct file *file)
{
	pr_debug("ch201 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static long ch201_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	uint32_t range;
	uint16_t amplitude;
	uint16_t dev_id;
	struct ch_group_t *grp_ptr;
	struct ch201_comms_struct comms_struct = {0};
	void __user *data_ptr = NULL;
	int32_t ret = 0;
	
	grp_ptr = &chirp_group;

	ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
	if (ret) {
		pr_err("Error at %s(%d)\n", __func__, __LINE__);
		return -EINVAL;
	}

	data_ptr = (u8 __user *)(comms_struct.buf);
	comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);

	switch (cmd) {
		case SEN_ULTRASONIC_IOCTL_MEASURE:
			ch_group_trigger(&chirp_group);
			chbsp_delay_ms(1);
			handle_data_ready(grp_ptr);
			while(interrupt_flag != 1)
			{
				mdelay(1);
			}
			range = chirp_data[0].range / 32;
			amplitude = chirp_data[0].amplitude;
			interrupt_flag = 0;

			// printk("ch201 Range: %d mm  Amplitude: %u  \r\n", range, amplitude);

			// copy to user buffer the read transfer
			ret = copy_to_user(data_ptr, (uint8_t *)&range, 4);

			if (ret) {
				pr_err("ch201: Error at %s(%d) when copy range to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			ret = copy_to_user(data_ptr + 4, (uint8_t *)&amplitude, 2);

			if (ret) {
				pr_err("sht30: Error at %s(%d) when copy amplitude to user.\n", __func__, __LINE__);
				return -EINVAL;
			}

			break;
		case SEN_ULTRASONIC_IOCTL_DEVID:
			dev_id = chirp_devices[0].part_number;

			ret = copy_to_user(data_ptr, &dev_id, 2);
			if (ret) {
				pr_err("ch201: Error at %s(%d) when copy dev_id to user.\n", __func__, __LINE__);
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations ch201_fops = {
	.owner 			= THIS_MODULE,
	.unlocked_ioctl		= ch201_ioctl,
	.open 			= ch201_open,
	.release 		= ch201_release,
};

static int chirp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ch_dev_t *dev_ptr;
	struct ch_group_t *grp_ptr;
	ch_config_t dev_config;
	uint8_t num_ports;
    uint8_t dev_num;
	uint8_t prog_tries = 0;

	int ret = 0;

	ch201_data = devm_kzalloc(&client->dev, sizeof(*ch201_data), GFP_KERNEL);
	if (ch201_data == NULL)
		return -ENOMEM;

	ch201_data->i2c = client;
	ch201_data->i2c_addr = CHIRP_I2C_ADDRS;
	i2c_set_clientdata(client, ch201_data);

	grp_ptr = &chirp_group;
	ch201_data->chirp_group = grp_ptr;
	ch201_data->ch201 = dev_ptr;
	chbsp_board_init(ch201_data->chirp_group);

	num_ports = ch_get_num_ports(ch201_data->chirp_group);

    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        dev_ptr = &(chirp_devices[dev_num]);      // init struct in array
        ret |= ch_init(dev_ptr, ch201_data->chirp_group, dev_num,
                            CHIRP_SENSOR_FW_INIT_FUNC);
    }

	ret = !ch201_data->chirp_group;

	if (!ret) {
    	ret = chdrv_group_prepare(grp_ptr);
  	}

	if (ret) {
		dev_err(&client->dev, "chdrv_group_prepare failed.\n");
		return ret;
	}

RESET_AND_LOAD:
	if (ret == 0) {
        dev_err(&client->dev, "starting group... \r\n");
        ret = ch_group_start(ch201_data->chirp_group);
    }

	if (ret) {
		dev_err(&client->dev, "starting group failed \r\n");
        return -1;
	}

	ch201_data->i2c_addr = CHIRP_APP_I2C_ADDRS;
	chbsp_delay_ms(1);

	ret = chdrv_group_wait_for_lock(grp_ptr);

	if (ret != 0 && prog_tries++ < CH_PROG_XFER_RETRY + 1) {
		dev_err(&client->dev, "FAILED: %d", CH_PROG_XFER_RETRY + 1);
		ch201_data->i2c_addr = CHIRP_I2C_ADDRS;
		goto RESET_AND_LOAD;
    }

	if (ret == 0) {
		dev_err(&client->dev, "starting group r... \r\n");
        ret = ch_group_start_r(ch201_data->chirp_group);
    }

	if (ret) {
		dev_err(&client->dev, "starting group r failed \r\n");
        return -1;
	}

	/* Configure each sensor with its operating parameters
	 *   Initialize a ch_config_t structure with values defined in the
	 *   app_config.h header file, then write the configuration to the
	 *   sensor using ch_set_config().
	 */
	dev_info(&client->dev, "ch201 Configuring sensor(s)...\n");
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			/* Select sensor mode
			 *   All connected sensors are placed in CHIRP_SENSOR_MODE.
 	 		 */

			num_connected_sensors++;	
			active_devices |= (1 << dev_num);

			dev_config.mode = CHIRP_SENSOR_MODE;

			/* Init config structure with values from app_config.h */
			dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
			dev_config.static_range = CHIRP_SENSOR_STATIC_RANGE;

			/* If sensor will be free-running, set internal sample interval */
			if (dev_config.mode == CH_MODE_FREERUN) {
				dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
			} else {
				dev_config.sample_interval = 0;
			}
			dev_config.thresh_ptr = &chirp_ch201_thresholds;

			/* Apply sensor configuration */
			ret = ch_set_config(dev_ptr, &dev_config);

			/* Enable sensor interrupt if using free-running mode
			 *   Note that interrupt is automatically enabled if using
			 *   triggered modes.
			 */
			if ((!ret) && (dev_config.mode == CH_MODE_FREERUN)) {
				chbsp_group_set_io_dir_in(grp_ptr);
				chbsp_group_io_interrupt_enable(grp_ptr);
			}

			ret = ch_set_thresholds(dev_ptr,&chirp_ch201_thresholds);

			/* Read back and display config settings */
			if (!ret) {
				display_config_info(dev_ptr);
			} else {
				dev_err(&client->dev, "ch201 Device %d: Error during ch_set_config()\n", dev_num);
			}

			/* Get number of active samples per measurement */
			chirp_data[dev_num].num_samples = ch_get_num_samples(dev_ptr);
		}
	}

	sen_ulsonic_miscdev.minor = MISC_DYNAMIC_MINOR;
	sen_ulsonic_miscdev.name = "ch201";
	sen_ulsonic_miscdev.fops = &ch201_fops;
	sen_ulsonic_miscdev.mode = 0444;

	ret = misc_register(&sen_ulsonic_miscdev);
	if (ret) {
		dev_err(&client->dev, "stmvl53l5cx : Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;

	return 0;
}

static int chirp_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver chirp_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= chirp_probe,
	.remove             = chirp_remove,
	.driver				= {
		.name			= CHIRP_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(chirp_of_match),
	},
	.id_table			= chirp_i2c_device_ids,
};

static int __init ch201_module_init(void)
{
	int ret = 0;

	printk("ch201: module init\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&chirp_driver);

	if (ret) {
		i2c_del_driver(&chirp_driver);
		printk("ch201: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

	return ret;
}

static void __exit ch201_module_exit(void)
{

	pr_debug("ch201 : module exit\n");

	if (misc_registered) {
		misc_deregister(&sen_ulsonic_miscdev);
		misc_registered = 0;
	}

	if (i2c_driver_added) {
		i2c_del_driver(&chirp_driver);
		i2c_driver_added = 0;
	}

}

module_init(ch201_module_init);
module_exit(ch201_module_exit);

MODULE_AUTHOR("DDT");
MODULE_DESCRIPTION("CH201 I2C driver");
MODULE_LICENSE("GPL");