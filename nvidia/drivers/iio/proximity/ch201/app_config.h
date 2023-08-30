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
#include "types.h"

#define CHIRP_SENSOR_MODE		CH_MODE_FREERUN

#define CH_PROG_XFER_RETRY 4

//#define CHIRP_SENSOR_MODE CH_MODE_TRIGGERED_TX_RX

struct chirp_snsr {
	struct iio_dev *chirp_iio;
	//struct bmi_rrs *rrs;
	//struct sensor_cfg cfg;
	//unsigned int usr_cfg;
	//unsigned int period_us;
};

struct chirp_state {
	struct i2c_client   *i2c;
    struct ch_dev_t		    *ch201;
    struct ch_group_t          *chirp_group;

    uint16_t            i2c_addr;
    const char  	    *name;
	uint32_t		    range;							// from ch_get_range()
	uint16_t		    amplitude;						// from ch_get_amplitude()
	uint16_t		    num_samples;					// from ch_get_num_samples()
	uint16_t            i2c_drv_flags;

    int                 irq_gpio;			/* interrupt GPIO */
    int                 prog_gpio;
    int                 i2c_rst_gpio;

    bool                started;
	bool                connected;
	bool                active;
	bool                data_ready;

    
	// struct bmi_snsr snsrs[BMI_HW_N];
	// struct bmi_gte_irq gis[BMI_HW_N];
	// bool iio_init_done[BMI_HW_N];
	// unsigned int part;
	// unsigned int sts;
	// unsigned int errs_bus;
	// unsigned int err_ts_thread;
	// unsigned int sam_dropped;
	// unsigned int enabled;
	// unsigned int suspend_en_st;
	// unsigned int hw_n;
	// unsigned int hw_en;
};

struct chirp_state *ch201_data;