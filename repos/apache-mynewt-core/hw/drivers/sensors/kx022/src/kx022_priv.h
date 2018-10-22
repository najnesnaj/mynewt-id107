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

#ifndef __KX022_PRIV_H__
#define __KX022_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

#define KX022_ADDR_L 0x1E
//#define KX022_ADDR_H 0x1F
#define KX022_ADDR_RESET 0x1D  // ? I cant find the reference to this
#define KX022_WHOAMI_REG 0x0F
#define KX022_WHOAMI_RESPONSE 0x14
#define KX022_CNTL1_REG 0x18
#define KX022_CNTL1_VALUE_STANDBY 0x41
#define KX022_ODCNTL_REG 0x1B
#define KX022_ODCNTL_VALUE 0x02
#define KX022_CNTL3_REG 0x1A
#define KX022_CNTL3_VALUE 0xD8
#define KX022_TILT_TIMER_REG 0x22
#define KX022_TILT_TIMER_VALUE 0x01
#define KX022_CNTL2_REG 0x18
#define KX022_CNTL1_VALUE_OPERATE 0xC1


#define KX022_ACC_OUTPUT_RATE_MASK 0x00001111

#define DATA_OUT_BASE 0x06

int kx022_write8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
                      uint32_t value);
int kx022_read8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
                     uint8_t *value);
int kx022_read48(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
                      uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif /* __KX022_PRIV_H__ */
