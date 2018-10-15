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

/* Define the stats section and records */
STATS_SECT_START(kx022_stat_section)
    STATS_SECT_ENTRY(samples_acc_2g)
    STATS_SECT_ENTRY(samples_acc_4g)
    STATS_SECT_ENTRY(samples_acc_8g)
    STATS_SECT_ENTRY(samples_acc_16g)
 /*   STATS_SECT_ENTRY(samples_mag_1_3g)
    STATS_SECT_ENTRY(samples_mag_1_9g)
    STATS_SECT_ENTRY(samples_mag_2_5g)
    STATS_SECT_ENTRY(samples_mag_4_0g)
    STATS_SECT_ENTRY(samples_mag_4_7g)
    STATS_SECT_ENTRY(samples_mag_5_6g)
    STATS_SECT_ENTRY(samples_mag_8_1g)*/
    STATS_SECT_ENTRY(errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(kx022_stat_section)
    STATS_NAME(kx022_stat_section, samples_acc_2g)
    STATS_NAME(kx022_stat_section, samples_acc_4g)
    STATS_NAME(kx022_stat_section, samples_acc_8g)
    STATS_NAME(kx022_stat_section, samples_acc_16g)
/*    STATS_NAME(kx022_stat_section, samples_mag_1_3g)
    STATS_NAME(kx022_stat_section, samples_mag_1_9g)
    STATS_NAME(kx022_stat_section, samples_mag_2_5g)
    STATS_NAME(kx022_stat_section, samples_mag_4_0g)
    STATS_NAME(kx022_stat_section, samples_mag_4_7g)
    STATS_NAME(kx022_stat_section, samples_mag_5_6g)
    STATS_NAME(kx022_stat_section, samples_mag_8_1g)*/
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

static const struct sensor_driver g_kx022_sensor_driver = {
    kx022_sensor_read,
    kx022_sensor_get_config
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
        KX022_ERR("I2C access failed at address 0x%02X\n", addr);
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
        KX022_ERR("I2C access failed at address 0x%02X\n", addr);
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
    struct kx022 *lsm;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    lsm = (struct kx022 *) dev;

    lsm->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lsm->sensor;

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

    /* Add the accelerometer/magnetometer driver */
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

    return (0);
err:
    return (rc);
}

int
kx022_config(struct kx022 *lsm, struct kx022_cfg *cfg)
{
    int rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(lsm->sensor));

    /* Most sensor chips have a single address and just use different
     * registers to get data for different sensors
     */
    if (!cfg->acc_addr ) {
        rc = SYS_EINVAL;
        goto err;
    }

    /* Set accel data rate (or power down) and enable XYZ output */
    rc = kx022_write8(itf, cfg->acc_addr,
                           KX022_REGISTER_ACCEL_CTRL_REG1_A,
                           cfg->rate | 0x07);
    if (rc) {
        goto err;
    }

    lsm->cfg.rate = cfg->rate;

    /* Set accel scale */
    rc = kx022_write8(itf, cfg->acc_addr,
                           KX022_REGISTER_ACCEL_CTRL_REG4_A,
                           cfg->range);
    if (rc) {
        goto err;
    }

    lsm->cfg.range = cfg->range;

    /* Enable the magnetomer (set to continuous conversion mode) */
//    rc = kx022_write8(itf, cfg->mag_addr,
 //                          KX022_REGISTER_MAG_MR_REG_M, 0x00);
   // if (rc) {
    //    goto err;
   // }

    /* Set mag rate */
  //  rc = kx022_write8(itf, cfg->mag_addr,
   //                        KX022_REGISTER_MAG_CRA_REG_M,
    //                       cfg->mag_rate);
   // if (rc) {
    //    goto err;
   // }

  //  lsm->cfg.mag_rate = cfg->mag_rate;

    /* Set mag gain */
   // rc = kx022_write8(itf, cfg->mag_addr,
    //                       KX022_REGISTER_MAG_CRB_REG_M,
     //                      cfg->mag_gain);
    //if (rc) {
     //   goto err;
    //}

  //  lsm->cfg.mag_gain = cfg->mag_gain;
//skunky is een munky
    rc = sensor_set_type_mask(&(lsm->sensor),  cfg->mask);
    if (rc) {
        goto err;
    }

    lsm->cfg.mask = cfg->mask;
   // lsm->cfg.mag_addr = cfg->mag_addr;
    lsm->cfg.acc_addr = cfg->acc_addr;

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
    float mg_lsb;
    //int16_t gauss_lsb_xy;
    //int16_t gauss_lsb_z;
    uint8_t payload[6];
    struct sensor_itf *itf;
    struct kx022 *lsm;
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
    lsm = (struct kx022 *) SENSOR_GET_DEVICE(sensor);

    /* Get a new accelerometer sample */
    if (type & SENSOR_TYPE_ACCELEROMETER) {
        x = y = z = 0;
        rc = kx022_read48(itf, lsm->cfg.acc_addr,
                               KX022_REGISTER_ACCEL_OUT_X_L_A | 0x80,
                               payload);
        if (rc != 0) {
            goto err;
        }

        /* Shift 12-bit left-aligned accel values into 16-bit int */
        x = ((int16_t)(payload[0] | (payload[1] << 8))) >> 4;
        y = ((int16_t)(payload[2] | (payload[3] << 8))) >> 4;
        z = ((int16_t)(payload[4] | (payload[5] << 8))) >> 4;

        /* Determine mg per lsb based on range */
        switch(lsm->cfg.range) {
            case KX022_ACCEL_RANGE_2:
                STATS_INC(g_kx022stats, samples_acc_2g);
                mg_lsb = 0.001F;
                break;
            case KX022_ACCEL_RANGE_4:
                STATS_INC(g_kx022stats, samples_acc_4g);
                mg_lsb = 0.002F;
                break;
            case KX022_ACCEL_RANGE_8:
                STATS_INC(g_kx022stats, samples_acc_8g);
                mg_lsb = 0.004F;
                break;
            case KX022_ACCEL_RANGE_16:
                STATS_INC(g_kx022stats, samples_acc_16g);
                mg_lsb = 0.012F;
                break;
            default:
                KX022_ERR("Unknown accel range: 0x%02X. Assuming +/-2G.\n",
                    lsm->cfg.range);
                mg_lsb = 0.001F;
                break;
        }

        /* Convert from mg to Earth gravity in m/s^2 */
        databuf.sad.sad_x = (float)x * mg_lsb * 9.80665F;
        databuf.sad.sad_y = (float)y * mg_lsb * 9.80665F;
        databuf.sad.sad_z = (float)z * mg_lsb * 9.80665F;

        databuf.sad.sad_x_is_valid = 1;
        databuf.sad.sad_y_is_valid = 1;
        databuf.sad.sad_z_is_valid = 1;

        /* Call data function */
        rc = data_func(sensor, data_arg, &databuf.sad, SENSOR_TYPE_ACCELEROMETER);
        if (rc != 0) {
            goto err;
        }
    }

    /* Get a new magnetometer sample */
 /*   if (type & SENSOR_TYPE_MAGNETIC_FIELD) {
        x = y = z = 0;
        rc = kx022_read48(itf, lsm->cfg.mag_addr,
                               KX022_REGISTER_MAG_OUT_X_H_M,
                               payload);
        if (rc != 0) {
            goto err;
        }

        x = (int16_t)(payload[1] | ((int16_t)payload[0] << 8));
        y = (int16_t)(payload[3] | ((int16_t)payload[2] << 8));
        z = (int16_t)(payload[5] | ((int16_t)payload[4] << 8));

        switch (lsm->cfg.mag_gain) {
            case KX022_MAG_GAIN_1_3:
                STATS_INC(g_kx022stats, samples_mag_1_3g);
                gauss_lsb_xy = 1100;
                gauss_lsb_z = 980;
                break;
            case KX022_MAG_GAIN_1_9:
                STATS_INC(g_kx022stats, samples_mag_1_9g);
                gauss_lsb_xy = 855;
                gauss_lsb_z = 760;
                break;
            case KX022_MAG_GAIN_2_5:
                STATS_INC(g_kx022stats, samples_mag_2_5g);
                gauss_lsb_xy = 670;
                gauss_lsb_z = 600;
                break;
            case KX022_MAG_GAIN_4_0:
                STATS_INC(g_kx022stats, samples_mag_4_0g);
                gauss_lsb_xy = 450;
                gauss_lsb_z = 400;
                break;
            case KX022_MAG_GAIN_4_7:
                STATS_INC(g_kx022stats, samples_mag_4_7g);
                gauss_lsb_xy = 400;
                gauss_lsb_z = 355;
                break;
            case KX022_MAG_GAIN_5_6:
                STATS_INC(g_kx022stats, samples_mag_5_6g);
                gauss_lsb_xy = 330;
                gauss_lsb_z = 295;
                break;
            case KX022_MAG_GAIN_8_1:
                STATS_INC(g_kx022stats, samples_mag_8_1g);
                gauss_lsb_xy = 230;
                gauss_lsb_z = 205;
                break;
            default:
                KX022_ERR("Unknown mag gain: 0x%02X. Assuming +/-1.3g.\n",
                    lsm->cfg.mag_gain);
                gauss_lsb_xy = 1100;
                gauss_lsb_z = 980;
                break;
        }

        databuf.smd.smd_x = (float)x / gauss_lsb_xy * 100.0F;
        databuf.smd.smd_y = (float)y / gauss_lsb_xy * 100.0F;
        databuf.smd.smd_z = (float)z / gauss_lsb_z * 100.0F;

        databuf.smd.smd_x_is_valid = 1;
        databuf.smd.smd_y_is_valid = 1;
        databuf.smd.smd_z_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.smd, SENSOR_TYPE_MAGNETIC_FIELD);
        if (rc != 0) {
            goto err;
        }
    }*/

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
