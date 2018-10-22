/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
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

#ifndef __SENSOR_KX022_H__
#define __SENSOR_KX022_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

//	enum kx022_accel_range {
//		KX022_ACCEL_RANGE_2            = 0x00 << 4, /* +/- 2g  */
//		KX022_ACCEL_RANGE_4            = 0x01 << 4, /* +/- 4g  */
//		KX022_ACCEL_RANGE_8            = 0x02 << 4, /* +/- 8g  */
//		KX022_ACCEL_RANGE_16           = 0x03 << 4, /* +/- 16g */
//	};

	enum kx022_accel_rate {
		KX022_ACCEL_RATE_POWER_DOWN    = 0x00 << 4,
		KX022_ACCEL_RATE_1             = 0x01 << 4,
		KX022_ACCEL_RATE_10            = 0x02 << 4,
		KX022_ACCEL_RATE_25            = 0x03 << 4,
		KX022_ACCEL_RATE_50            = 0x04 << 4,
		KX022_ACCEL_RATE_100           = 0x05 << 4,
		KX022_ACCEL_RATE_200           = 0x06 << 4,
		KX022_ACCEL_RATE_400           = 0x07 << 4,
		KX022_ACCEL_RATE_1620          = 0x08 << 4
	};


	//#define KX022_ADDR_ACCEL                  0x1F  /* */
#define KX022_ADDR_H                  0x1F  /* */

//#define KX022_FS_2G 0x00
#define KX022_RANGE_2G 0b00000000  // range +/-2g
#define KX022_RANGE_4G 0b00001000  // range +/-4g
#define KX022_RANGE_8G 0b00011000  // range +/-8g
#define KX022_RANGE_MASK 0b00011000

//#define KX022_1020_OUTPUT_RATE_50_HZ                  0b00000010  //  3     0                       50 Hz
	enum KX022_ACC_OUTPUT_RATE
	{
		KX022_ACC_OUTPUT_RATE_12_5_HZ,
		KX022_ACC_OUTPUT_RATE_25_HZ,
		KX022_ACC_OUTPUT_RATE_50_HZ,
		KX022_ACC_OUTPUT_RATE_100_HZ,
		KX022_ACC_OUTPUT_RATE_200_HZ,
		KX022_ACC_OUTPUT_RATE_400_HZ,
		KX022_ACC_OUTPUT_RATE_800_HZ,
		KX022_ACC_OUTPUT_RATE_1600_HZ,
		KX022_ACC_OUTPUT_RATE_0_781_HZ,
		KX022_ACC_OUTPUT_RATE_1_563_HZ,
		KX022_ACC_OUTPUT_RATE_3_125_HZ,
		KX022_ACC_OUTPUT_RATE_6_25_HZ
	};

	struct kx022_cfg {
		uint8_t rate;
		uint8_t range;
		//    uint8_t lc_pull_up_disc;
		//    enum kx022_mag_gain mag_gain;
		//    enum kx022_mag_rate mag_rate;
		//    uint8_t mag_addr;
		uint8_t acc_addr;
		sensor_type_t lc_s_mask;
	};

	struct kx022 {
		struct os_dev dev;
		struct sensor sensor;
		struct kx022_cfg cfg;
		os_time_t last_read_time;
	};

	int kx022_init(struct os_dev *, void * arg);
	int kx022_config(struct kx022 * kx022, struct kx022_cfg * cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_KX022_H__ */
