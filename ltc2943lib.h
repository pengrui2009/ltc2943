#ifndef _LTC2943LIB_H_
#define _LTC2943LIB_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#define HTS_IOC_MAGIC 0xE3

//set the work mode
/*
  0	---	sleep mode,sleep
  1	---	manual mode,performing signal conversions voltage,current,and temp  then sleep
  2	---	scan mode,performing voltage,current,and temp conversions every 10s
  3	---	automatic mode, continuously performing voltage,current,and temp conversions
  default:0
  */
#define SET_ADC_MODE		    		_IO(HTS_IOC_MAGIC,  0)
//set alcc mode
/*
  0	---	Pin Disable Mode,ALCC pin disabled
  1	---	Charge Complete Mode. Pin becomes logic input and accepts
		charge complete inverted signal (e.g., from a charger) to set
		accumulated charge register (C,D) to FFFFh.
  2	---	Alert mode, Alert functionality enabled. Pin becomes logic output.
  default:2
  */
#define SET_ALCC_MODE		    		_IO(HTS_IOC_MAGIC,  1)
//set Prescaler
/*
  0	---	M:1
  1	---	M:4
  2	---	M:15
  3	---	M:64
  4	---	M:256  
  5	---	M:1024
  6	---	M:4096
  7	---	M:4096
  default:7
  */
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