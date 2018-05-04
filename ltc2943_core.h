/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : ltc2943_core.h
 @brief  : the driver of ltc2943 I2C .
 @author : pengrui
 @history:
           2018-05-04    pengrui    Created file
           ...
******************************************************************************/
#ifndef _LTC2943_CORE_H_
#define _LTC2943_CORE_H_
#include <linux/cdev.h>		//char device register
#include <linux/device.h>
#include <linux/module.h>
#include "ltc2943lib.h"

/* Input events used by lsm303agr driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#if defined(CONFIG_INPUT_HTS221_SPI) || \
    defined(CONFIG_INPUT_HTS221_SPI_MODULE)
#define LTC2943_RX_MAX_LENGTH		500
#define LTC2943_TX_MAX_LENGTH		500

struct ltc2943_transfer_buffer {
	u8 rx_buf[LTC2943_RX_MAX_LENGTH];
	u8 tx_buf[LTC2943_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_HTS221_SPI */

struct ltc2943_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};


typedef enum _ltc2943_alcc_mode_en {
	ALCC_MODE_PIN_DISABLE = 0,
	ALCC_MODE_CHARGE_COMPLETE,
	ALCC_MODE_ALERT,
	ALCC_MODE_END
}ltc2943_alcc_mode_en;

typedef enum _ltc2943_prescaler_en {
	PRESCALER_M_1,
	PRESCALER_M_4,
	PRESCALER_M_16,
	PRESCALER_M_64,
	PRESCALER_M_256,
	PRESCALER_M_1024,
	PRESCALER_M_4096,
	PRESCALER_M_END
}ltc2943_prescaler_en;

typedef enum _ltc2943_adc_mode_en{
	ADC_MODE_SLEEP,
	ADC_MODE_MANUAL,
	ADC_MODE_SCAN,
	ADC_MODE_AUTOMATIC,
	ADC_MODE_END
}ltc2943_adc_mode_en;
	
enum ltc2943_sensor_type {
	LTC2943_SENSOR_CHARGE,
	LTC2943_SENSOR_VOLTAGE,
	LTC2943_SENSOR_CURRENT,
	LTC2943_SENSOR_TEMPERATURE,
	LTC2943_SENSOR_END
};

struct ltc2943_sensor_data {
	u16 max_thresholds;
	u16 min_thresholds;
	u16 value;
};

struct ltc2943_dev {
	const char *name;
	atomic_t opened;
	u16 bus_type;
	struct mutex lock;
	struct device *dev;
	dev_t drv_dev_num;
	struct cdev drv_cdev;
    struct class *drv_class;
	struct delayed_work input_work;
	const struct ltc2943_transfer_function *tf;
#if defined(CONFIG_INPUT_LTC2943_SPI) || \
    defined(CONFIG_INPUT_LTC2943_SPI_MODULE)
	struct ltc2943_transfer_buffer tb;
#endif /* CONFIG_INPUT_HTS221_SPI */
	//resistor is 50 m from datasheet
	u16 resistor;

	u18 adc_mode;
	u16 alcc_mode;
	u16 prescalar;
	u16 poll_interval;
    bool enabled;

	struct ltc2943_sensor_data sensors[LTC2943_SENSOR_END];
};

static inline s64 hts221_get_time_ns(void)
{
	struct timespec ts;

	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

int hts221_probe(struct hts221_dev *dev);
void hts221_remove(struct hts221_dev *dev);
int hts221_enable(struct hts221_dev *dev);
int hts221_disable(struct hts221_dev *dev);

#endif /* _HTC2943_CORE_H_ */
