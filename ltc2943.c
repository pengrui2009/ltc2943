/*!
LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.
LTC2943-1: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.

@verbatim

The LTC2943 measures battery charge state, battery voltage,
battery current and its own temperature in portable
product applications. The wide input voltage range allows
use with multicell batteries up to 20V. A precision coulomb
counter integrates current through a sense resistor between
the battery’s positive terminal and the load or charger.
Voltage, current and temperature are measured with an
internal 14-bit No Latency ΔΣ™ ADC. The measurements
are stored in internal registers accessible via the onboard
I2C/SMBus Interface

@endverbatim

http://www.linear.com/product/LTC2943
http://www.linear.com/product/LTC2943-1

http://www.linear.com/product/LTC2943#demoboards
http://www.linear.com/product/LTC2943-1#demoboards

REVISION HISTORY
$Revision: 5672 $
$Date: 2016-09-02 11:42:55 -0700 (Fri, 02 Sep 2016) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

//! @defgroup LTC2943 LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement

/*! @file
   @ingroup LTC2943
   Library for LTC2943 Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement
*/



#include <linux/types.h>

#include "ltc2943.h"

// Write an 8-bit code to the LTC2943.
#if 0
int8_t LTC2943_write(uint8_t i2c_address, uint8_t adc_command, uint8_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_write_byte_data(i2c_address, adc_command, code);
  return(ack);
}


// Write a 16-bit code to the LTC2943.
int8_t LTC2943_write_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t ack;

  ack = i2c_write_word_data(i2c_address, adc_command, code);
  return(ack);
}

// Reads an 8-bit adc_code from LTC2943
int8_t LTC2943_read(uint8_t i2c_address, uint8_t adc_command, uint8_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_read_byte_data(i2c_address, adc_command, adc_code);

  return(ack);
}

// Reads a 16-bit adc_code from LTC2943
int8_t LTC2943_read_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_read_word_data(i2c_address, adc_command, adc_code);

  return(ack);
}
#endif
/******************************************************************************
*    Function: LTC2943_code_to_coulombs
*    Descriptions: convert charge code to coulombs
*    Paramters:
            	adc_code               - the adc value 
		resistor			- the value of register(uint 0.001R)
		prescalar			- the value of prescaler
*    Return:
            > 0                     	- the value of coulombs(uint 0.000001coulombs)
            < 0                     	- failed
*    Comments: 
******************************************************************************/
int LTC2943_code_to_coulombs(u16 adc_code, u16 resistor, u16 prescalar)
// The function converts the 16-bit RAW adc_code to Coulombs
{
  int coulomb_charge;
  
  //coulomb_charge =  1000*(float)(adc_code*LTC2943_CHARGE_lsb*prescalar*50E-3)/(resistor*4096);
  coulomb_charge =  1000 * (int)(adc_code * LTC2943_CHARGE_LSB * prescalar * 50)/(resistor*4096);
  coulomb_charge = coulomb_charge * 36;
  
  return coulomb_charge;
}

/******************************************************************************
*    Function: LTC2943_code_to_mAh
*    Descriptions: convert charge code to mAh
*    Paramters:
            	adc_code               - the adc value 
		resistor			- the value of register(uint 0.001R)
		prescalar			- the value of prescaler
*    Return:
            > 0                     	- the value of coulombs(uint 0.00001mAh)
            < 0                     	- failed
*    Comments: 
******************************************************************************/

int LTC2943_code_to_mAh(u16 adc_code, u16 resistor, u16 prescalar)
// The function converts the 16-bit RAW adc_code to mAh
{
  int mAh_charge;
  
  mAh_charge = 1000 * (int)(adc_code * LTC2943_CHARGE_LSB * prescalar * 50) / (resistor * 4096);
  
  return mAh_charge;
}

/******************************************************************************
*    Function: LTC2943_code_to_voltage
*    Descriptions: convert voltage code to coulombs
*    Paramters:
            	adc_code               - the adc value 
*    Return:
            > 0                     	- the value of coulombs(uint 0.1mV)
            < 0                     	- failed
*    Comments: 
******************************************************************************/
int LTC2943_code_to_voltage(u16 adc_code)
// The function converts the 16-bit RAW adc_code to Volts
{
  int voltage;
  
  voltage = ((int)adc_code * LTC2943_FULLSCALE_VOLTAGE / (65535)) ;
  
  return(voltage);
}

/******************************************************************************
*    Function: LTC2943_code_to_current
*    Descriptions: convert charge code to coulombs
*    Paramters:
            	adc_code               - the adc value 
		resistor			- the value of register(uint 0.001R)
*    Return:
            > 0                     	- the value of coulombs(uint 0.00001mA)
            < 0                     	- failed
*    Comments: 
******************************************************************************/
int LTC2943_code_to_current(u16 adc_code, u16 resistor)
// The function converts the 16-bit RAW adc_code to Amperes
{
  int current;
  
  current = (((int)adc_code - 32767) * LTC2943_FULLSCALE_CURRENT / (32767) / resistor);
  
  return current;
}

/******************************************************************************
*    Function: LTC2943_code_to_kelvin_temperature
*    Descriptions: convert tempurate code to coulombs
*    Paramters:
            	adc_code               - the adc value 
*    Return:
            > 0                     	- the value of coulombs(uint 1K)
            < 0                     	- failed
*    Comments: 
******************************************************************************/
int LTC2943_code_to_kelvin_temperature(uint16_t adc_code)
// The function converts the 16-bit RAW adc_code to Kelvin
{
  int temperature;
  
  temperature = (int) adc_code *  LTC2943_FULLSCALE_TEMPERATURE / 65535 ;
  
  return temperature;
}

/******************************************************************************
*    Function: LTC2943_code_to_celcius_temperature
*    Descriptions: convert tempurate code to celcius
*    Paramters:
            	adc_code               - the adc value 
*    Return:
            > 0                     	- the value of coulombs(uint 1C)
            < 0                     	- failed
*    Comments: 
******************************************************************************/
int LTC2943_code_to_celcius_temperature(u16 adc_code)
// The function converts the 16-bit RAW adc_code to Celcius
{
  int temperature;
  
  temperature = (int) adc_code * LTC2943_FULLSCALE_TEMPERATURE / 65535  - 273;
  
  return(temperature);
}

#if 0
// Used to set and clear bits in a control register.  bits_to_set will be bitwise OR'd with the register.
// bits_to_clear will be inverted and bitwise AND'd with the register so that every location with a 1 will result in a 0 in the register.
int8_t LTC2943_register_set_clear_bits(uint8_t i2c_address, uint8_t register_address, uint8_t bits_to_set, uint8_t bits_to_clear)
{
  uint8_t register_data;
  int8_t ack = 0;

  ack |= LTC2943_read(i2c_address, register_address, &register_data);
  register_data = register_data & (~bits_to_clear);
  register_data = register_data | bits_to_set;
  ack |= LTC2943_write(i2c_address, register_address, register_data);
  return(ack);
}
#endif


