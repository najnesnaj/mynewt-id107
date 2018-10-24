/*****************************************************************************/
/*!
  @file     si1143.c
  @author   Rip Mayall 

  @section LICENSE

  Software License Agreement (BSD License)


  this is a driver for the si1143 light sensor,
  which is used in the ID 107 HR smartwatch

  it could be extended for more general use,
  but right now it is tailored for the watch


*/
/*****************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/light.h"
#include "si1143/si1143.h"
#include "si1143_priv.h"
#include "log/log.h"
#include "stats/stats.h"
#include "console/console.h"

/* Define the stats section and records */
	STATS_SECT_START(si1143_stat_section)
	STATS_SECT_ENTRY(ints_cleared)
STATS_SECT_ENTRY(errors)
	STATS_SECT_END

	/* Define stat names for querying */
	STATS_NAME_START(si1143_stat_section)
	STATS_NAME(si1143_stat_section, ints_cleared)
	STATS_NAME(si1143_stat_section, errors)
STATS_NAME_END(si1143_stat_section)

	/* Global variable used to hold stats data */
	STATS_SECT_DECL(si1143_stat_section) g_si1143stats;

#define LOG_MODULE_SI1143     (1143)
#define SI1143_INFO(...)     LOG_INFO(&_log, LOG_MODULE_SI1143, __VA_ARGS__)
#define SI1143_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_SI1143, __VA_ARGS__)
	static struct log _log;

	/* Exports for the sensor API */
	static int si1143_sensor_read(struct sensor *, sensor_type_t,
			sensor_data_func_t, void *, uint32_t);
static int si1143_sensor_get_config(struct sensor *, sensor_type_t,
		struct sensor_cfg *);

static const struct sensor_driver g_si1143_sensor_driver = {
	si1143_sensor_read,
	si1143_sensor_get_config
};

	int
si1143_write8(struct sensor_itf *itf, uint8_t reg, uint32_t value)
{
	int rc;
	uint8_t payload[2] = { reg, value & 0xFF };

	struct hal_i2c_master_data data_struct = {
		.address = itf->si_addr,
		.len = 2,
		.buffer = payload
	};

	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		SI1143_ERR("Failed to write 0x%02X:0x%02X with value 0x%02lX\n",
				data_struct.address, reg, value);
		STATS_INC(g_si1143stats, errors);
	}

	return rc;
}

	int
si1143_write16(struct sensor_itf *itf, uint8_t reg, uint16_t value)
{
	int rc;
	uint8_t payload[3] = { reg, value & 0xFF, (value >> 8) & 0xFF };

	struct hal_i2c_master_data data_struct = {
		.address = itf->si_addr,
		.len = 3,
		.buffer = payload
	};

	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		SI1143_ERR("Failed to write @0x%02X with value 0x%02X 0x%02X\n",
				reg, payload[0], payload[1]); //jj this should be [1] and [2] ??
	}

	return rc;
}

	int
si1143_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
	int rc;
	uint8_t payload;

	struct hal_i2c_master_data data_struct = {
		.address = itf->si_addr,
		.len = 1,
		.buffer = &payload
	};

	/* Register write */
	payload = reg;
	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		SI1143_ERR("Failed to address sensor\n");
		goto err;
	}

	/* Read one byte back */
	payload = 0;
	rc = hal_i2c_master_read(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	*value = payload;
	if (rc) {
		SI1143_ERR("Failed to read @0x%02X\n", reg);
	}

	return 0;
err:
	return rc;
}



	int
si1143_read48(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
		uint8_t *buffer)
{
	int rc;
	uint8_t payload[7] = { reg, 0, 0, 0, 0, 0, 0 };

	struct hal_i2c_master_data data_struct = {
		.address = addr,
		.len = 1,
		.buffer = payload
	};

	/* Clear the supplied buffer */
	memset(buffer, 0, 6);

	/* Register write */
	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		SI1143_ERR("i2C access failed at address 0x%02X SI1143 read48\n", addr);
		STATS_INC(g_si1143stats, errors);
		goto err;
	}

	/* Read six bytes back */
	memset(payload, 0, sizeof(payload));
	data_struct.len = 6;
	rc = hal_i2c_master_read(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);

	if (rc) {
		SI1143_ERR("Failed to read from 0x%02X:0x%02X\n", addr, reg);
		STATS_INC(g_si1143stats, errors);
		goto err;
	}

	/* Copy the I2C results into the supplied buffer */
	memcpy(buffer, payload, 6);

err:
	return rc;
}




/*
   int
   si1143_read48(struct sensor_itf *itf, uint8_t reg, uint16_t *value)
   {
   int rc;
   uint8_t payload[6] = { reg, 0 };

   struct hal_i2c_master_data data_struct = {
   .address = itf->si_addr,
   .len = 1,
   .buffer = payload
   };

   rc = hal_i2c_master_write(itf->si_num, &data_struct,
   OS_TICKS_PER_SEC / 10, 1);
   if (rc) {
   SI1143_ERR("Failed to address sensor\n");
   goto err;
   }

   memset(payload, 0, 2);
   data_struct.len = 2;
   rc = hal_i2c_master_read(itf->si_num, &data_struct,
   OS_TICKS_PER_SEC / 10, 1);
 *value = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
 if (rc) {
 SI1143_ERR("Failed to read @0x%02X\n", reg);
 goto err;
 }

 return 0;
err:
return rc;
}
*/
/**
 * Enable or disables the sensor to save power
 *
 * @param The sensor interface
 * @param state  1 to enable the sensor, 0 to disable it
 *
 * @return 0 on success, non-zero on failure
 */
int
  si1143_enable(struct sensor_itf *itf, uint8_t state)
  {
  return si1143_write8(itf, SI114_REG_PS_LED21,
  state ? 0xbb :
  0x00);
  }
  
/**
 * Checks if the sensor in enabled or not
 *
 * @param The sensor interface
 * @param ptr to enabled
 *
 * @return 0 on success, non-zero on fialure
 */
/*int
  si1143_get_enable(struct sensor_itf *itf, uint8_t *enabled)
  {
  int rc;
  uint8_t reg;

  rc =  si1143_read8(itf, PS_LED21,
  &reg);
  if (rc) {
  goto err;
  }

 *enabled = reg & 0xbb ? 1 : 0;

 return 0;
err:
return rc;
}
*/
/**
 * Sets the integration time used when sampling light values.
 *
 * @param The sensor interface
 * @param int_time The integration time which can be one of:
 *                  - 0x00: 13ms
 *                  - 0x01: 101ms
 *                  - 0x02: 402ms
 *
 * @return 0 on success, non-zero on failure
 */
/*
   int
   si1143_set_integration_time(struct sensor_itf *itf,
   uint8_t int_time)
   {
   int rc;
   uint8_t gain;

   rc = si1143_get_gain(itf, &gain);
   if (rc) {
   goto err;
   }

   rc = si1143_write8(itf, SI1143_COMMAND_BIT | SI1143_REGISTER_TIMING,
   int_time | gain);
   if (rc) {
   goto err;
   }

   return 0;
err:
return rc;
}
*/
/**
 * Gets the current integration time used when sampling light values.
 *
 * @param The sensor interface
 * @param ptr to the integration time which can be one of:
 *         - 0x00: 13ms
 *         - 0x01: 101ms
 *         - 0x02: 402ms
 * @return 0 on success, non-zero on failure
 */
/*int
  si1143_get_integration_time(struct sensor_itf *itf, uint8_t *itime)
  {
  int rc;
  uint8_t reg;

  rc = si1143_read8(itf, SI1143_COMMAND_BIT | SI1143_REGISTER_TIMING,
  &reg);
  if (rc) {
  goto err;
  }

 *itime = reg & 0x0F;

 return 0;
err:
return rc;
}
*/
/**
 * Sets the gain increment used when sampling light values.
 *
 * @param The sensor interface
 * @param gain The gain increment which can be one of:
 *                  - 0x00: 1x (no gain)
 *                  - 0x10: 16x gain
 *
 * @return 0 on success, non-zero on failure
 */
/*int
  si1143_set_gain(struct sensor_itf *itf, uint8_t gain)
  {
  int rc;
  uint8_t int_time;

  if (gain > 0x04) {
  SI1143_ERR("Invalid gain value\n");
  rc = SYS_EINVAL;
  goto err;
  }
  rc = si1143_write8(itf, PARAM_PS_ADC_GAIN,
  gain);
  if (rc) {
  goto err;
  }

  return 0;
err:
return rc;
}
*/
/**
 * Gets the current gain increment used when sampling light values.
 *
 * @param The sensor ineterface
 * @param ptr to the gain increment which can be one of:
 *         - 0x00: 1x (no gain)
 *         - 0x10: 16x gain
 * @return 0 on success, non-zero on failure
 */
/*int
  si1143_get_gain(struct sensor_itf *itf, uint8_t *gain)
  {
  int rc;
  uint8_t reg;

  rc = si1143_read8(itf, SI1143_COMMAND_BIT | SI1143_REGISTER_TIMING,
  &reg);
  if (rc) {
  goto err;
  }

 *gain = reg & 0xF0;

 return 0;
err:
return rc;
}
*/


	int
si1143_init(struct os_dev *dev, void *arg)
{
	struct si1143 *si1143;
	struct sensor *sensor;
	int rc;

	if (!arg || !dev) {
		rc = SYS_ENODEV;
		goto err;
	}

	si1143 = (struct si1143 *) dev;

	si1143->cfg.mask = SENSOR_TYPE_ALL;

	log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

	sensor = &si1143->sensor;

	/* Initialise the stats entry */
	rc = stats_init(
			STATS_HDR(g_si1143stats),
			STATS_SIZE_INIT_PARMS(g_si1143stats, STATS_SIZE_32),
			STATS_NAME_INIT_PARMS(si1143_stat_section));
	SYSINIT_PANIC_ASSERT(rc == 0);
	/* Register the entry with the stats registry */
	rc = stats_register(dev->od_name, STATS_HDR(g_si1143stats));
	SYSINIT_PANIC_ASSERT(rc == 0);

	rc = sensor_init(sensor, dev);
	if (rc) {
		goto err;
	}

	/* Add the light driver */
	rc = sensor_set_driver(sensor, SENSOR_TYPE_LIGHT,
			(struct sensor_driver *) &g_si1143_sensor_driver);
	if (rc) {
		goto err;
	}

	/* Set the interface */
	rc = sensor_set_interface(sensor, arg);
	if (rc) {
		goto err;
	}

	rc = sensor_mgr_register(sensor);
	if (rc) {
		goto err;
	}

	return 0;
err:
	return rc;

}

	static int
si1143_sensor_read(struct sensor *sensor, sensor_type_t type,
		sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
	struct si1143 *si1143;
	struct sensor_light_data sld;
	uint8_t payload[6];
	struct sensor_itf *itf;
	int rc;

	/* If the read isn't looking for accel or mag data, don't do anything. */
	if (!(type & SENSOR_TYPE_LIGHT)) {
		rc = SYS_EINVAL;
		goto err;
	}

	itf = SENSOR_GET_ITF(sensor);
	si1143 = (struct si1143 *)SENSOR_GET_DEVICE(sensor);

	/* Get a new accelerometer sample */
	if (type & SENSOR_TYPE_LIGHT) {

		rc = si1143_read48(itf, SI1143_ADDR, SI114_REG_PS1_DATA0, payload);
		if (rc) {
			goto err;
		}

		sld.sld_full = 0;
		sld.sld_ir = 0;
		sld.sld_lux = ((uint32_t) (payload[0] | payload[1] <<8 | payload[2] << 16 | payload[3] << 24)); 
		//jj sld_lux is uint32_t the actual measurement is 6 bytes ...
		si1143->cfg.intensity_measurement=sld.sld_lux;
		sld.sld_full_is_valid = 1;
		sld.sld_ir_is_valid   = 1;
		sld.sld_lux_is_valid  = 1;

		/* Call data function */
		rc = data_func(sensor, data_arg, &sld, SENSOR_TYPE_LIGHT);
		if (rc != 0) {
			goto err;
		}
	}

	return 0;
err:
	return rc;
}

	static int
si1143_sensor_get_config(struct sensor *sensor, sensor_type_t type,
		struct sensor_cfg *cfg)
{
	int rc;

	if ((type != SENSOR_TYPE_LIGHT)) {
		rc = SYS_EINVAL;
		goto err;
	}

	cfg->sc_valtype = SENSOR_VALUE_TYPE_INT32;

	return 0;
err:
	return rc;
}

/**
 * Configure the sensor
 *
 * @param ptr to sensor driver
 * @param ptr to sensor driver config
 */
	int
si1143_config(struct si1143 *si1143, struct si1143_cfg *cfg)
{
	int rc;
	struct sensor_itf *itf;

	itf = SENSOR_GET_ITF(&(si1143->sensor));

	// rc = si1143_enable(itf, 1);
	if (!cfg->lum_addr ) {
		rc = SYS_EINVAL;
		goto err;
	}

	rc= si1143_write8(itf,  SI114_REG_HW_KEY, 0x17); 
	rc= si1143_write8(itf,  SI114_REG_IRQ_CFG, 0x03); // turn on interrupts 
	rc= si1143_write8(itf,  SI114_REG_IRQ_ENABLE, 0x04);// turn on interrupt on PS1 
	rc= si1143_write8(itf,  SI114_REG_IRQ_MODE1, 0x30); 
	rc= si1143_write8(itf,  SI114_REG_MEAS_RATE, 0x99);//setting as sniffed 
	rc= si1143_write8(itf,  SI114_REG_ALS_RATE, 0x08); 
	rc= si1143_write8(itf,  SI114_REG_PS_RATE, 0x08); 

	rc= si1143_write8(itf,  SI114_REG_PS_LED21, 0xbb); 


	rc= si1143_write16(itf,  SI114_REG_PARAM_WR, ((0xa0 |SI114_PARAM_CH_LIST)<<8) | 0x77 ); //all measurements on
	rc= si1143_write16(itf,  SI114_REG_PARAM_WR, ((0xa0 |SI114_PARAM_PS_ADC_GAIN)<<8) | 0x04 ); 
	rc= si1143_write16(itf,  SI114_REG_PARAM_WR, ((0xa0 |SI114_PARAM_PSLED12_SELECT)<<8) | 0x21 ); 
	rc= si1143_write16(itf,  SI114_REG_PARAM_WR, ((0xa0 |SI114_PARAM_PS1_ADC_MUX)<<8) | 0x03 ); //PS1  
	rc= si1143_write16(itf,  SI114_REG_PARAM_WR, ((0xa0 |SI114_PARAM_PS_ADC_COUNTER)<<8) | 0x70 ); 

	rc= si1143_write8(itf,  SI114_REG_COMMAND, 0x0d); 

	// rc = si1143_set_integration_time(itf, cfg->integration_time);
	// if (rc) {
	//    goto err;
	// }

	//si1143->cfg.integration_time = cfg->integration_time;

	//  rc = si1143_set_gain(itf, cfg->gain);
	//if (rc) {
	//   goto err;
	// }

	//si1143->cfg.gain = cfg->gain;

	rc = sensor_set_type_mask(&(si1143->sensor), cfg->mask);
	if (rc) {
		goto err;
	}

	si1143->cfg.mask = cfg->mask;

	return 0;
err:
	console_printf("some error\n");
	return rc;
}
