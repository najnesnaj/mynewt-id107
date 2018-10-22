/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <errno.h>
#include <assert.h>

#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
//#include "sensor/mag.h"
#include "kx022/kx022.h"
#include "kx022_priv.h"
#include "log/log.h"
#include "stats/stats.h"
#include "console/console.h"

/* Define the stats section and records */
	STATS_SECT_START(kx022_stat_section)
	STATS_SECT_ENTRY(samples_acc_2g)
	STATS_SECT_ENTRY(samples_acc_4g)
	STATS_SECT_ENTRY(samples_acc_8g)
STATS_SECT_ENTRY(samples_acc_16g)
STATS_SECT_ENTRY(errors)
	STATS_SECT_END

	/* Define stat names for querying */
	STATS_NAME_START(kx022_stat_section)
	STATS_NAME(kx022_stat_section, samples_acc_2g)
	STATS_NAME(kx022_stat_section, samples_acc_4g)
	STATS_NAME(kx022_stat_section, samples_acc_8g)
	STATS_NAME(kx022_stat_section, errors)
STATS_NAME_END(kx022_stat_section)

	/* Global variable used to hold stats data */
	STATS_SECT_DECL(kx022_stat_section) g_kx022stats;

#define LOG_MODULE_KX022 (303)
#define KX022_INFO(...)  LOG_INFO(&_log, LOG_MODULE_KX022, __VA_ARGS__)
#define KX022_ERR(...)   LOG_ERROR(&_log, LOG_MODULE_KX022, __VA_ARGS__)
	static struct log _log;

	/* Exports for the sensor API */
	static int kx022_sensor_read(struct sensor *, sensor_type_t,
			sensor_data_func_t, void *, uint32_t);
static int kx022_sensor_get_config(struct sensor *, sensor_type_t,
		struct sensor_cfg *);




static int
sensor_driver_set_config(struct sensor *sensor, void *cfg)
{
    struct kx022* kx022 = (struct kx022 *)SENSOR_GET_DEVICE(sensor);

    return kx022_config(kx022, (struct kx022_cfg*)cfg);
}





static const struct sensor_driver g_kx022_sensor_driver = {
	.sd_read = kx022_sensor_read,
	.sd_set_config = sensor_driver_set_config,
	.sd_get_config = kx022_sensor_get_config
};

/**
 * Writes a single byte to the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
	int
kx022_write8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
		uint32_t value)
{
	int rc;
	uint8_t payload[2] = { reg, value & 0xFF };

	struct hal_i2c_master_data data_struct = {
		.address = addr,
		.len = 2,
		.buffer = payload
	};

	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		KX022_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02lX\n",
				addr, reg, value);
		STATS_INC(g_kx022stats, errors);
	}

	return rc;
}

/**
 * Reads a single byte from the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
	int
kx022_read8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
		uint8_t *value)
{
	int rc;
	uint8_t payload;

	struct hal_i2c_master_data data_struct = {
		.address = addr,
		.len = 1,
		.buffer = &payload
	};

	/* Register write */
	payload = reg;
	rc = hal_i2c_master_write(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	if (rc) {
		KX022_ERR("KX022 read8 I2C access failed at address 0x%02X\n", addr);
		STATS_INC(g_kx022stats, errors);
		goto err;
	}

	/* Read one byte back */
	payload = 0;
	rc = hal_i2c_master_read(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);
	*value = payload;
	if (rc) {
		KX022_ERR("Failed to read from 0x%02X:0x%02X\n", addr, reg);
		STATS_INC(g_kx022stats, errors);
	}

err:
	return rc;
}

/**
 * Reads a six bytes from the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
	int
kx022_read48(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
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
		KX022_ERR("i2C access failed at address 0x%02X KX022 read48\n", addr);
		STATS_INC(g_kx022stats, errors);
		goto err;
	}

	/* Read six bytes back */
	memset(payload, 0, sizeof(payload));
	data_struct.len = 6;
	rc = hal_i2c_master_read(itf->si_num, &data_struct,
			OS_TICKS_PER_SEC / 10, 1);

	if (rc) {
		KX022_ERR("Failed to read from 0x%02X:0x%02X\n", addr, reg);
		STATS_INC(g_kx022stats, errors);
		goto err;
	}

	/* Copy the I2C results into the supplied buffer */
	memcpy(buffer, payload, 6);

err:
	return rc;
}

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accelerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
	int
kx022_init(struct os_dev *dev, void *arg)
{
	struct kx022 *kx022;
	struct sensor *sensor;
//	struct sensor_itf *itf;
	int rc;
//console_printf("kx022 init called by os dev create\n");
	if (!arg || !dev) {
		rc = SYS_ENODEV;
		goto err;
	}

	kx022 = (struct kx022 *) dev;
//	itf = SENSOR_GET_ITF(&(kx022->sensor));

	kx022->cfg.lc_s_mask = SENSOR_TYPE_ALL;

	log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

	sensor = &kx022->sensor;

	/* Initialise the stats entry */
	rc = stats_init(
			STATS_HDR(g_kx022stats),
			STATS_SIZE_INIT_PARMS(g_kx022stats, STATS_SIZE_32),
			STATS_NAME_INIT_PARMS(kx022_stat_section));
	SYSINIT_PANIC_ASSERT(rc == 0);
	/* Register the entry with the stats registry */
	rc = stats_register(dev->od_name, STATS_HDR(g_kx022stats));
	SYSINIT_PANIC_ASSERT(rc == 0);

	rc = sensor_init(sensor, dev);
	if (rc != 0) {
		goto err;
	}

	/* Add the accelerometer driver */
	rc = sensor_set_driver(sensor, SENSOR_TYPE_ACCELEROMETER ,
			(struct sensor_driver *) &g_kx022_sensor_driver);
	if (rc != 0) {
		goto err;
	}

	/* Set the interface */
	rc = sensor_set_interface(sensor, arg);
	if (rc) {
		goto err;
	}

	rc = sensor_mgr_register(sensor);
	if (rc != 0) {
		goto err;
	}

//kx022_config(kx022, &kx022->cfg); //jj test if calling config gives different results

        return (0);
err:
	return(rc);
}

	int
kx022_config(struct kx022 *kx022, struct kx022_cfg *cfg)
{
	int rc;
	console_printf("kx022_config is called\n");
	struct sensor_itf *itf;
	itf = SENSOR_GET_ITF(&(kx022->sensor));

	/* Most sensor chips have a single address and just use different
	 * registers to get data for different sensors
	 */
	if (!cfg->acc_addr ) {
		rc = SYS_EINVAL;
		goto err;
	}
	console_printf("addrd\n");

	/* Set accel data rate (or power down) and enable XYZ output */
	rc = kx022_write8(itf, cfg->acc_addr,
			KX022_CNTL1_REG, (KX022_CNTL1_VALUE_STANDBY & ~KX022_ACC_OUTPUT_RATE_MASK) |
			(cfg->range & KX022_ACC_OUTPUT_RATE_MASK));
	if (rc) {
		goto err;
	}
//	os_time_delay((OS_TICKS_PER_SEC)/2); //jj

	kx022->cfg.rate = cfg->rate; 

	/* Set accel scale */
	rc = kx022_write8(itf, cfg->acc_addr,
			KX022_ODCNTL_REG,
			(KX022_ODCNTL_VALUE & ~KX022_ACC_OUTPUT_RATE_MASK) |
			(cfg->rate & KX022_ACC_OUTPUT_RATE_MASK));
	if (rc) {
		goto err;
	}

	rc = kx022_write8(itf, cfg->acc_addr,
			KX022_CNTL3_REG, KX022_CNTL3_VALUE);
	if (rc) {
		goto err;
	}

	rc = kx022_write8(itf, cfg->acc_addr,
			KX022_TILT_TIMER_REG, KX022_TILT_TIMER_VALUE);
	if (rc) {
		goto err;
	}

	//hier moet nog een delay jj
	os_time_delay((OS_TICKS_PER_SEC)/2);
	rc = kx022_write8(itf, cfg->acc_addr,
			KX022_CNTL1_REG, (KX022_CNTL1_VALUE_OPERATE & ~KX022_ACC_OUTPUT_RATE_MASK) |
			(cfg->range & KX022_ACC_OUTPUT_RATE_MASK));
	if (rc) {
		goto err;
	}


	console_printf("applying settings\n");


	kx022->cfg.range = cfg->range;

	rc = sensor_set_type_mask(&(kx022->sensor),  cfg->lc_s_mask);
	if (rc) {
		goto err;
	}
	console_printf("mask\n");

	kx022->cfg.lc_s_mask = cfg->lc_s_mask;
	// kx022->cfg.mag_addr = cfg->mag_addr;
	kx022->cfg.acc_addr = cfg->acc_addr;

	return 0;
err:
	return (rc);
}









	static int
kx022_sensor_read(struct sensor *sensor, sensor_type_t type,
		sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
	int rc;
	int16_t x, y, z;
	uint8_t payload[6];
	struct sensor_itf *itf;
	float sensitivity=1;
	struct kx022 *kx022;
	union {
		struct sensor_accel_data sad;
		//struct sensor_mag_data smd;
	}databuf;

	/* If the read isn't looking for accel or mag data, don't do anything. */
	if (!(type & SENSOR_TYPE_ACCELEROMETER) ) {
		rc = SYS_EINVAL;
		goto err;
	}

	itf = SENSOR_GET_ITF(sensor);
	kx022 = (struct kx022 *)SENSOR_GET_DEVICE(sensor);
//kx022_config(kx022, &kx022->cfg); //jj test if calling config gives different results

	/* Get a new accelerometer sample */
	if (type & SENSOR_TYPE_ACCELEROMETER) {
		x = y = z = 0;
		//rc = kx022_read48(itf, kx022->cfg.acc_addr, DATA_OUT_BASE ,
		rc = kx022_read48(itf, KX022_ADDR_H, DATA_OUT_BASE ,
				payload);
		if (rc != 0) {
			goto err;
		}

		x = ((int16_t)(payload[0] | (payload[1] << 8))) ;
		y = ((int16_t)(payload[2] | (payload[3] << 8))) ;
		z = ((int16_t)(payload[4] | (payload[5] << 8))) ;
		/* Determine mg per lsb based on range */
		switch(kx022->cfg.range) {
			case KX022_RANGE_2G:
				STATS_INC(g_kx022stats, samples_acc_2g);
				sensitivity = 16384.0;
				break;
			case KX022_RANGE_4G:
				STATS_INC(g_kx022stats, samples_acc_4g);
				sensitivity = 8192.0;
				break;
			case KX022_RANGE_8G:
				STATS_INC(g_kx022stats, samples_acc_8g);
				sensitivity = 4096.0;
				break;
			default:
				KX022_ERR("Unknown accel range: 0x%02X. Assuming +/-2G.\n",
						kx022->cfg.range);
				sensitivity = 16384.0;
				break;
		}

		databuf.sad.sad_x = (float)x /sensitivity;
		databuf.sad.sad_y = (float)y /sensitivity;
		databuf.sad.sad_z = (float)z /sensitivity;

		databuf.sad.sad_x_is_valid = 1;
		databuf.sad.sad_y_is_valid = 1;
		databuf.sad.sad_z_is_valid = 1;

		/* Call data function */
		rc = data_func(sensor, data_arg, &databuf.sad, SENSOR_TYPE_ACCELEROMETER);
		if (rc != 0) {
			goto err;
		}
	}


	return (0);
err:
	return (rc);
}

	static int
kx022_sensor_get_config(struct sensor *sensor, sensor_type_t type,
		struct sensor_cfg *cfg)
{
	int rc;

	if (type != SENSOR_TYPE_ACCELEROMETER)
	{
		rc = SYS_EINVAL;
		goto err;
	}

	cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;

	return (0);
err:
	return (rc);
}
