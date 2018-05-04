#ifndef _LTC2943LIB_H_
#define _LTC2943LIB_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#define HTS_IOC_MAGIC 0xE3


#define SET_ADC_MODE		    		_IO(HTS_IOC_MAGIC,  0)

#define SET_ALCC_MODE		    		_IO(HTS_IOC_MAGIC,  1)

#define SET_PRESCALER		    		_IO(HTS_IOC_MAGIC,  2)

#define SET_HIGH_CHARGE_THRESHOLD		_IO(HTS_IOC_MAGIC,  3)

#define SET_LOW_CHARGE_THRESHOLD		_IO(HTS_IOC_MAGIC,  4)

#define SET_HIGH_VOLTAGE_THRESHOLD		_IO(HTS_IOC_MAGIC,  5)

#define SET_LOW_VOLTAGE_THRESHOLD		_IO(HTS_IOC_MAGIC,  6)

#define SET_HIGH_CURRENT_THRESHOLD		_IO(HTS_IOC_MAGIC,  7)

#define SET_LOW_CURRENT_THRESHOLD		_IO(HTS_IOC_MAGIC,  8)

#define SET_HIGH_TEMP_THRESHOLD			_IO(HTS_IOC_MAGIC,  9)

#define SET_LOW_TEMP_THRESHOLD			_IO(HTS_IOC_MAGIC,  10)

#define SET_ENABLE						_IO(HTS_IOC_MAGIC,  11)

#endif /*_LTC2943LIB_H_*/