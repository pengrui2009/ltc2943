/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : ltc2943_core.c
 @brief  : the driver of ltc2943 I2C .
 @author : pengrui
 @history:
           2018-05-04    pengrui    Created file
           ...
******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include "ltc2943_core.h"

/*************************************************
  the driver info
*************************************************/
#define DRIVER_VER1		0					//
#define DRIVER_VER2		1					//
#define DRV_MAJOR		104 				//
#define DRV_MINOR	 	1					//

//debug macro
#undef	DEBUG
//#define DEBUG
#ifdef DEBUG
#define	DPRINTK( x... )		printk(DRV_NAME":" x)
#else
#define DPRINTK( x... )
#endif


#define LTC2943_DEFAULT_POLL_PERIOD_MS 1000

static struct ltc2943_dev *ltc2943_ptr = NULL;

int ltc2943_read_reg(struct ltc2943_dev *ltc2943_ptr, u8 addr, int data_len, u8 *data_ptr)
{
	int ret = 0;

	ret = ltc2943_ptr->tf->read(ltc2943_ptr->dev, addr, data_len, data_ptr);

	return ret;
}

int ltc2943_write_reg(struct ltc2943_dev *ltc2943_ptr, u8 addr, int data_len, u8 *data_ptr)
{
	int ret = 0;

	ret = ltc2943_ptr->tf->write(ltc2943_ptr->dev, addr, data_len, data_ptr);

	return ret;
}

int ltc2943_update_charge_thresholds(struct ltc2943_dev *ltc2943_ptr, u16 max_charge_threshold, u16 min_charge_threshold)
{
	int ret = 0;
	u8 charge_msb = 0;
	u8 charge_lsb = 0;
	mutex_lock(&ltc2943_ptr->lock);

	charge_msb = (u8)(( max_charge_threshold & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CHARGE_THRESH_HIGH_MSB_REG, 1, &charge_msb);
	if(ret < 0)
	{
		goto error;
	}

	charge_lsb = (u8)( max_charge_threshold & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CHARGE_THRESH_HIGH_LSB_REG, 1, &charge_lsb);
	if(ret < 0)
	{
		goto error;
	}

	charge_msb = (u8)(( min_charge_threshold & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CHARGE_THRESH_LOW_MSB_REG, 1, &charge_msb);
	if(ret < 0)
	{
		goto error;
	}

	charge_lsb = (u8)( min_charge_threshold & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CHARGE_THRESH_LOW_LSB_REG, 1, &charge_lsb);
	if(ret < 0)
	{
		goto error;
	}
	
	mutex_unlock(&ltc2943_ptr->lock);

error:

	return ret;
}

int ltc2943_update_voltage_thresholds(struct ltc2943_dev *ltc2943_ptr, u16 max_voltage_threshold, u16 min_voltage_threshold)
{
	int ret = 0;
	u8 voltage_msb = 0;
	u8 voltage_lsb = 0;
	mutex_lock(&ltc2943_ptr->lock);

	voltage_msb = (u8)(( max_voltage_threshold & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG, 1, &voltage_msb);
	if(ret < 0)
	{
		goto error;
	}

	voltage_msb = (u8)( max_voltage_threshold & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG, 1, &voltage_msb);
	if(ret < 0)
	{
		goto error;
	}

	voltage_msb = (u8)(( min_voltage_threshold & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_VOLTAGE_THRESH_LOW_MSB_REG, 1, &voltage_msb);
	if(ret < 0)
	{
		goto error;
	}

	voltage_lsb = (u8)( min_voltage_threshold & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_VOLTAGE_THRESH_LOW_LSB_REG, 1, &voltage_lsb);
	if(ret < 0)
	{
		goto error;
	}
	
	mutex_unlock(&ltc2943_ptr->lock);

error:

	return ret;
}

int ltc2943_update_current_thresholds(struct ltc2943_dev *ltc2943_ptr, u16 max_current_thresholds, u16 min_current_thresholds)
{
	int ret = 0;
	u8 current_msb = 0;
	u8 current_lsb = 0;
	mutex_lock(&ltc2943_ptr->lock);

	current_msb = (u8)(( max_current_thresholds & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CURRENT_THRESH_HIGH_MSB_REG, 1, &current_msb);
	if(ret < 0)
	{
		goto error;
	}

	current_lsb = (u8)( max_current_thresholds & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CURRENT_THRESH_HIGH_LSB_REG, 1, &current_lsb);
	if(ret < 0)
	{
		goto error;
	}

	current_msb = (u8)(( min_current_thresholds & 0xFF00) >> 8);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CURRENT_THRESH_LOW_MSB_REG, 1, &current_msb);
	if(ret < 0)
	{
		goto error;
	}

	current_lsb = (u8)( min_current_thresholds & 0x00FF);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CURRENT_THRESH_LOW_LSB_REG, 1, &current_lsb);
	if(ret < 0)
	{
		goto error;
	}
	
	mutex_unlock(&ltc2943_ptr->lock);

error:

	return ret;	
}


int ltc2943_update_temperature_thresholds(struct ltc2943_dev *ltc2943_ptr, u8 max_temp_thresholds, u8 min_temp_thresholds)
{
	int ret = 0;
	u8 temp_msb = 0;
	u8 temp_lsb = 0;
	mutex_lock(&ltc2943_ptr->lock);

	temp_msb = max_temp_thresholds ;
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_TEMPERATURE_THRESH_HIGH_REG, 1, &temp_msb);
	if(ret < 0)
	{
		goto error;
	}

	temp_lsb =  min_temp_thresholds;
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_TEMPERATURE_THRESH_LOW_REG, 1, &temp_lsb);
	if(ret < 0)
	{
		goto error;
	}

	mutex_unlock(&ltc2943_ptr->lock);

error:

	return ret;	
}

int ltc2943_update_prescaler(struct ltc2943_dev *ltc2943_ptr, ltc2943_prescaler_en prescalar)
{
	int ret = 0;
	u8 regval = 0;
	
	if(NULL == ltc2943_ptr)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}

	switch(prescalar)
	{
	case PRESCALER_M_1:
		regval |= LTC2943_PRESCALAR_M_1;
		break;
	case PRESCALER_M_4:
		regval |= LTC2943_PRESCALAR_M_4;
		break;
	case PRESCALER_M_16:
		regval |= LTC2943_PRESCALAR_M_16;
		break;
	case PRESCALER_M_64:
		regval |= LTC2943_PRESCALAR_M_64;
		break;
	case PRESCALER_M_256:
		regval |= LTC2943_PRESCALAR_M_256;
		break;
	case PRESCALER_M_1024:
		regval |= LTC2943_PRESCALAR_M_1024;
		break;
	case PRESCALER_M_4096:
		regval |= LTC2943_PRESCALAR_M_4096;
		break;
	default:
		regval |= LTC2943_PRESCALAR_M_4096_2;
		break;
	}

	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}

error:

	return ret;
}

int ltc2943_update_alcc_mode(struct ltc2943_dev *ltc2943_ptr, ltc2943_alcc_mode_en mode)
{
	int ret = 0;
	u8 regval = 0;

	if(NULL == ltc2943_ptr)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
	
	switch(mode)
	{
	case ALCC_MODE_PIN_DISABLE:
		regval |= LTC2943_DISABLE_ALCC_PIN;
		break;
	case ALCC_MODE_CHARGE_COMPLETE:
		regval |= LTC2943_CHARGE_COMPLETE_MODE;
		break;
	case ALCC_MODE_ALERT:
		regval |= LTC2943_ALERT_MODE;
		break;
	default:
		regval |= LTC2943_ALERT_MODE;
		break;
	}

	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
	
error:

	return ret;
}


int ltc2943_update_adc_mode(struct ltc2943_dev *ltc2943_ptr, ltc2943_adc_mode_en mode)
{
	int ret = 0;
	u8 regval = 0;

	if(NULL == ltc2943_ptr)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
	
	switch(mode)
	{
	case ADC_MODE_SLEEP:
		regval |= LTC2943_SLEEP_MODE;
		break;
	case ADC_MODE_MANUAL:
		regval |= LTC2943_MANUAL_MODE;
		break;
	case ADC_MODE_SCAN:
		regval |= LTC2943_SCAN_MODE;
		break;
	case ADC_MODE_AUTOMATIC:
		regval |= LTC2943_AUTOMATIC_MODE
		break;		
	default:
		regval |= LTC2943_SLEEP_MODE;
		break;
	}

	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
	
error:

	return ret;
	
}

int ltc2943_device_power_on(struct ltc2943_dev *ltc2943_ptr)
{
	int ret = 0;
	u8 regval = 0;

	if(NULL == ltc2943_ptr)
	{
		ret = -ENOMEM;
		goto error;
	}
	
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}

	regval &= (~LTC2943_SHUTDOWN_MODE);
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
error:
	
	return ret;
}

int ltc2943_device_power_off(struct ltc2943_dev *ltc2943_ptr)
{
	int ret = 0;

	int ret = 0;
	u8 regval = 0;

	if(NULL == ltc2943_ptr)
	{
		ret = -ENOMEM;
		goto error;
	}
	
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}

	regval |= LTC2943_SHUTDOWN_MODE;
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
error:
	
	return ret;
error:
	
	return ret;
}

int ltc2943_get_data(struct ltc2943_dev *ltc2943_ptr, u32 *data_charge, u32 *data_voltage, u32 *data_current, u32 *data_temp)
{
	int ret = 0;
	u8 charge_msb = 0, charge_lsb = 0;
	u8 voltage_msb = 0, voltage_lsb = 0;
	u8 current_msb = 0, current_lsb = 0;
	u8 temp_msb = 0, temp_lsb = 0;
	
    u16 charge_code, current_code, voltage_code, temperature_code;

	
	//! Read MSB and LSB Accumulated Charge Registers for 16 bit charge code
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_ACCUM_CHARGE_MSB_REG, 1, &charge_msb);
	if(ret < 0)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_ACCUM_CHARGE_LSB_REG, 1, &charge_lsb);
	if(ret < 0)
	{
		goto error;
	}

	charge_code = (charge_msb << 8) + charge_lsb;
	//(uint E-6 mhA)
	*data_charge = ((1000 * (u32)charge_code * 340 * (ltc2943_ptr->prescalar) * 50) / (ltc2943_ptr->resistor * 4096));
	
	//! Read MSB and LSB Voltage Registers for 16 bit voltage code
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_VOLTAGE_MSB_REG, 1, &voltage_msb);
	if(ret < 0)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_VOLTAGE_LSB_REG, 1, &voltage_lsb);
	if(ret < 0)
	{
		goto error;
	}

	voltage_code = (voltage_msb << 8) + voltage_lsb;
	//uint mV
	*data_voltage = ((u32)voltage_code * 236 * 100 / 65535);
	
	//! Read MSB and LSB Current Registers for 16 bit current code
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CURRENT_MSB_REG, 1, &current_msb);
	if(ret < 0)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CURRENT_LSB_REG, 1, &current_lsb);
	if(ret < 0)
	{
		goto error;
	}

	current_code = (voltage_msb << 8) + voltage_lsb;
	//(uint 1mA)
	*data_current = (((u32)current_code - 32767) * 60 / 32767 / ltc2943_ptr->resistor);

	//! Read MSB and LSB Temperature Registers for 16 bit temperature code
	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_TEMPERATURE_MSB_REG, 1, &temp_msb);
	if(ret < 0)
	{
		goto error;
	}

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_TEMPERATURE_LSB_REG, 1, &temp_lsb);
	if(ret < 0)
	{
		goto error;
	}

	temperature_code = (temp_msb << 8) + temp_lsb;
	//(uint 0.01degree)
	temperature = (u32)temperature_code * ((LTC2943_FULLSCALE_TEMPERATURE * 100) /65535) - 27315;
	//! Read Status Register for 8 bit status code
    ack |= LTC2943_read(LTC2943_I2C_ADDRESS, LTC2943_STATUS_REG, &status_code);

  
error:

	return ret;
}

int ltc2943_hw_init(struct ltc2943_dev *ltc2943_ptr)
{
	int ret = 0;
	u8 regval = 0;

	ret = ltc2943_read_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}

	regval |= LTC2943_SHUTDOWN_MODE;
	ret = ltc2943_write_reg(ltc2943_ptr, LTC2943_CONTROL_REG, 1, &regval);
	if(ret < 0)
	{
		goto error;
	}
	
error:
	
	return ret;
}

int ltc2943_enable(struct ltc2943_dev *ltc2943_ptr)
{
	int ret = 0;

	mutex_lock(&ltc2943_ptr->lock);
	
	if (!ltc2943_ptr->enabled) {
		ret = ltc2943_device_power_on(ltc2943_ptr);
		if((ADC_MODE_AUTOMATIC == ltc2943_ptr->adc_mode) || (ADC_MODE_SCAN == ltc2943_ptr->adc_mode))
			schedule_delayed_work(&ltc2943_ptr->input_work, msecs_to_jiffies(ltc2943_ptr->poll_interval));
	}
	mutex_unlock(&ltc2943_ptr->lock);

	return ret;
}

int ltc2943_disable(struct ltc2943_dev *ltc2943_ptr)
{
	int ret = 0;

	mutex_lock(&ltc2943_ptr->lock);
	if((ADC_MODE_AUTOMATIC == ltc2943_ptr->adc_mode) || (ADC_MODE_SCAN == ltc2943_ptr->adc_mode))
		cancel_delayed_work_sync(&ltc2943_ptr->input_work);

	
	if (ltc2943_ptr->enabled)
		ret = ltc2943_device_power_off(ltc2943_ptr);
	mutex_unlock(&ltc2943_ptr->lock);

	return ret;
}

static int drv_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	if(!atomic_dec_and_test(&hts221_ptr->opened)) {
		ret = -EBUSY;
		goto error;
	}
	
	ret = hts221_enable(hts221_ptr);
	if(ret < 0)
	{
		goto error;
	}
	
	return ret;

error:
	atomic_inc(&hts221_ptr->opened);
	return ret;
}

static int drv_release(struct inode *inode, struct file *filp)
{
	hts221_disable(hts221_ptr);
	
	atomic_inc(&hts221_ptr->opened);
	return 0;
}

static ssize_t drv_write(struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	int ret = 0;

	return ret;
}

static ssize_t drv_read(struct file *filp, char *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	int data_t = 0;
	int data_h = 0;
	u32 databuf[4] = {0};
	
	mutex_lock(&ltc2943_ptr->lock);

	if(ADC_MODE_MANUAL == ltc2943_ptr->adc_mode)
	{
		ret = ltc2943_get_data(ltc2943_ptr, &databuf[0], &databuf[1], &databuf[2], &databuf[3]);
		if (ret < 0)
			dev_err(ltc2943_ptr->dev, "get data failed\n");
	}else{
		databuf[0] = ltc2943_ptr->sensors[LTC2943_SENSOR_CHARGE].value;
		databuf[1] = ltc2943_ptr->sensors[LTC2943_SENSOR_VOLTAGE].value;
		databuf[2] = ltc2943_ptr->sensors[LTC2943_SENSOR_CURRENT].value;
		databuf[3] = ltc2943_ptr->sensors[LTC2943_SENSOR_TEMPERATURE].value;
	}
	
	mutex_unlock(&hts221_ptr->lock);
	
	ret = put_user(databuf[0], buf);
	if(ret)
	{
		goto error;
	}
	
	ret = put_user(databuf[1], (buf + 4));
	if(ret)
	{
		goto error;
	}

	ret = put_user(databuf[2], (buf + 4));
	if(ret)
	{
		goto error;
	}

	ret = put_user(databuf[3], (buf + 4));
	if(ret)
	{
		goto error;
	}
	ret = 12;
	
error:

	return ret;
}

static long drv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int i = 0;
	//DPRINTK("%s: enter!!\n",__FUNCTION__);
	switch(cmd){
		case SET_ADC_MODE:
		{
			u8 val = 0;
			ltc2943_adc_mode_en mode = 0;
			val = *(u8 *)arg;
			mutex_lock(&ltc2943_ptr->lock);
			switch(val)
			{
			case 0:
				mode = ADC_MODE_SLEEP;
				ret = ltc2943_update_adc_mode(ltc2943_ptr, mode);
				break;
			case 1:
				mode = ADC_MODE_MANUAL;
				ret = ltc2943_update_adc_mode(ltc2943_ptr, mode);
				break;				
			case 2:
				mode = ADC_MODE_SCAN;
				ret = ltc2943_update_adc_mode(ltc2943_ptr, mode);
				break;
			case 3:
				mode = ADC_MODE_AUTOMATIC;
				ret = ltc2943_update_adc_mode(ltc2943_ptr, mode);
				break;
			default:
				ret = -EINVAL;
				break;
			}
			mutex_unlock(&ltc2943_ptr->lock);
			break;
		}
		case SET_ALCC_MODE:
		{
			u8 val = 0;
			ltc2943_alcc_mode_en mode;
			mutex_lock(&ltc2943_ptr->lock);
			val = *(u8 *)arg;
			switch(val)
			{
			case 0:
				mode = ALCC_MODE_PIN_DISABLE;
				ret = ltc2943_update_alcc_mode(ltc2943_ptr, mode);
				break;
			case 0:
				mode = ALCC_MODE_CHARGE_COMPLETE;
				ret = ltc2943_update_alcc_mode(ltc2943_ptr, mode);
				break;
			case 0:
				mode = ALCC_MODE_ALERT;
				ret = ltc2943_update_alcc_mode(ltc2943_ptr, mode);
				break;
			default:
				ret = -EINVAL;
				break;
			}
			mutex_unlock(&ltc2943_ptr->lock);
			break;
		}
		case SET_PRESCALER:
		{
			
			break;
		}	
		case SET_HIGH_CHARGE_THRESHOLD:
		{
			break;
		}	
		case SET_LOW_CHARGE_THRESHOLD:
		{
			
			break;
		}
		case SET_HIGH_VOLTAGE_THRESHOLD:
		{
			break;
		}
		case SET_LOW_VOLTAGE_THRESHOLD:
		{
			break;
		}
		case SET_HIGH_CURRENT_THRESHOLD:
		{
			break;
		}
		case SET_LOW_CURRENT_THRESHOLD:
		{
			break;
		}
		case SET_HIGH_TEMP_THRESHOLD:
		{
			break;
		}
		case SET_LOW_TEMP_THRESHOLD:
		{
			
			break;
		}
		case SET_ENABLE:
		{
			u8 val = (u8 *)arg;

			if(val)
			{
				ltc2943_enable(ltc2943_ptr);
			}else{
				ltc2943_disable(ltc2943_ptr);
			}
			break;
		}
		default:
			break;
	}
	
	return ret;
}

static struct file_operations drv_fops = {
	.owner	= THIS_MODULE,
	.open	= drv_open,
	.write	= drv_write,
	.read	= drv_read,
	.unlocked_ioctl	= drv_ioctl,
	.release = drv_release,
};

static void ltc2943_input_work_fn(struct work_struct *work)
{
	int err, data_t, data_h;
	struct ltc2943_dev *dev;
	u32 charge = 0, voltage = 0, current = 0, tempeurate = 0; 

	dev = container_of((struct delayed_work *)work, struct ltc2943_dev, input_work);

	mutex_lock(&dev->lock);

	err = ltc2943_get_data(dev, &charge, &voltage, current, &tempeurate);
	if (err < 0)
		dev_err(dev->dev, "get data failed\n");
	else
	{
		//hts221_report_data(dev, data_t, data_h, hts221_get_time_ns());
		dev->sensors[LTC2943_SENSOR_CHARGE].value = charge;
		dev->sensors[LTC2943_SENSOR_VOLTAGE].value = voltage;
		dev->sensors[LTC2943_SENSOR_CURRENT].value = current;
		dev->sensors[LTC2943_SENSOR_TEMPERATURE].value = tempeurate;
	}	

	mutex_unlock(&dev->lock);

	schedule_delayed_work(&dev->input_work, msecs_to_jiffies(dev->poll_interval));
}


int ltc2943_probe(struct ltc2943_dev *dev)
{
	int ret = 0;
	//
	printk("%s-V%d.%d\n", dev->name, DRIVER_VER1, DRIVER_VER2);
	
	//BUILD_BUG_ON(!ARRAY_SIZE(hts221_odr_table));

	mutex_lock(&dev->lock);

	//the poll time when in (AUTOMATIC|SCAN) mode
	dev->poll_interval = LTC2943_DEFAULT_POLL_PERIOD_MS;

	//50m resistor get from datasheet
	dev->resistor = 50;

	//
	dev->sensors[LTC2943_SENSOR_CHARGE].max_thresholds = 0xFFFF;
	dev->sensors[LTC2943_SENSOR_CHARGE].min_thresholds = 0x0000;
	dev->sensors[LTC2943_SENSOR_VOLTAGE].max_thresholds = 0xFFFF;
	dev->sensors[LTC2943_SENSOR_VOLTAGE].min_thresholds = 0x0000;
	dev->sensors[LTC2943_SENSOR_CURRENT].max_thresholds = 0xFFFF;
	dev->sensors[LTC2943_SENSOR_CURRENT].min_thresholds = 0x0000;
	dev->sensors[LTC2943_SENSOR_TEMPERATURE].max_thresholds = 0xFFFF;
	dev->sensors[LTC2943_SENSOR_TEMPERATURE].min_thresholds = 0x0000;

	ret =  ltc2943_hw_init(dev);
	if (ret < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", ret);
		goto unlock;
	}

	ret = ltc2943_update_alcc_mode(dev, ALCC_ALERT_MODE);
	if (ret < 0) {
        dev_err(dev->dev, "set alcc mode failed: %d\n", ret);
		goto unlock;
	}

	ret = ltc2943_update_prescaler(dev, PRESCALER_M_4096);
	if (ret < 0) {
        dev_err(dev->dev, "set alcc mode failed: %d\n", ret);
		goto unlock;
	}
	
	//
	ret = register_chrdev_region (dev->drv_dev_num, DRV_MINOR, dev->name);
	if (ret) {
		goto ERR_CHRDEV;
	}
	//
	cdev_init(&dev->drv_cdev, &drv_fops);
	dev->drv_cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev->drv_cdev, dev->drv_dev_num, DRV_MINOR);
	if (ret) {
		goto ERR_CDEV;
	}
	//
	dev->drv_class = class_create(THIS_MODULE, dev->name);
	if (dev->drv_class == NULL) {
		ret = -ENOMEM;
		goto ERR_CLASS;
	}
	
	device_create(dev->drv_class, NULL, dev->drv_dev_num, NULL, dev->name);

	ret = hts221_device_power_off(dev);
	if (err < 0) {
		dev_err(dev->dev, "power off failed: %d\n", ret);
		goto ERR_CLASS;
	}

	/* get calibration data */
	if ((hts221_get_cal_data(dev, HTS221_SENSOR_T) < 0) ||
	    (hts221_get_cal_data(dev, HTS221_SENSOR_H) < 0)) {
		dev_err(dev->dev, "get calibration data failed: %d\n", ret);
		goto ERR_CLASS;
	}

	INIT_DELAYED_WORK(&dev->input_work, ltc2943_input_work_fn);

	ltc2943_ptr = dev;
	
	mutex_unlock(&dev->lock);

	//
	atomic_set(&dev->opened, 1);
	
	return 0;

ERR_CLASS:	
	cdev_del(&dev->drv_cdev);
ERR_CDEV:
	unregister_chrdev_region(dev->drv_dev_num, DRV_MINOR);	
ERR_CHRDEV:

power_off:
	hts221_device_power_off(dev);
unlock:
	mutex_unlock(&dev->lock);

	return ret;
}
EXPORT_SYMBOL(hts221_probe);

void ltc2943_remove(struct hts221_dev *dev)
{
	if(ODR_ONESH != dev->odr)
		cancel_delayed_work_sync(&dev->input_work);
	hts221_device_power_off(dev);
	//hts221_input_cleanup(dev);
	//hts221_sysfs_remove(dev->dev);
	//unregister the chrdev
	device_destroy(dev->drv_class, dev->drv_dev_num);
	class_destroy(dev->drv_class);	
	cdev_del(&dev->drv_cdev);
	unregister_chrdev_region(dev->drv_dev_num, DRV_MINOR);

	hts221_ptr = NULL;
	DPRINTK("exit done\n");
}
EXPORT_SYMBOL(hts221_remove);

MODULE_AUTHOR("pengrui <pengrui_2009@163.com>");
MODULE_LICENSE("GPL v2");

