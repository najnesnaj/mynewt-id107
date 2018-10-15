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

enum kx022_registers_accel {                         /* DEFAULT    TYPE */
    KX022_REGISTER_ACCEL_CTRL_REG1_A         = 0x20, /* 00000111   rw   */
    KX022_REGISTER_ACCEL_CTRL_REG2_A         = 0x21, /* 00000000   rw   */
    KX022_REGISTER_ACCEL_CTRL_REG3_A         = 0x22, /* 00000000   rw   */
    KX022_REGISTER_ACCEL_CTRL_REG4_A         = 0x23, /* 00000000   rw   */
    KX022_REGISTER_ACCEL_CTRL_REG5_A         = 0x24, /* 00000000   rw   */
    KX022_REGISTER_ACCEL_CTRL_REG6_A         = 0x25, /* 00000000   rw   */
    KX022_REGISTER_ACCEL_REFERENCE_A         = 0x26, /* 00000000   r    */
    KX022_REGISTER_ACCEL_STATUS_REG_A        = 0x27, /* 00000000   r    */
    KX022_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
    KX022_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
    KX022_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
    KX022_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
    KX022_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
    KX022_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
    KX022_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
    KX022_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
    KX022_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
    KX022_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
    KX022_REGISTER_ACCEL_INT1_THS_A          = 0x32,
    KX022_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
    KX022_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
    KX022_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
    KX022_REGISTER_ACCEL_INT2_THS_A          = 0x36,
    KX022_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
    KX022_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
    KX022_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
    KX022_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
    KX022_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
    KX022_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
    KX022_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
};

enum kx022_registers_mag {
    KX022_REGISTER_MAG_CRA_REG_M             = 0x00,
    KX022_REGISTER_MAG_CRB_REG_M             = 0x01,
    KX022_REGISTER_MAG_MR_REG_M              = 0x02,
    KX022_REGISTER_MAG_OUT_X_H_M             = 0x03,
    KX022_REGISTER_MAG_OUT_X_L_M             = 0x04,
    KX022_REGISTER_MAG_OUT_Z_H_M             = 0x05,
    KX022_REGISTER_MAG_OUT_Z_L_M             = 0x06,
    KX022_REGISTER_MAG_OUT_Y_H_M             = 0x07,
    KX022_REGISTER_MAG_OUT_Y_L_M             = 0x08,
    KX022_REGISTER_MAG_SR_REG_Mg             = 0x09,
    KX022_REGISTER_MAG_IRA_REG_M             = 0x0A,
    KX022_REGISTER_MAG_IRB_REG_M             = 0x0B,
    KX022_REGISTER_MAG_IRC_REG_M             = 0x0C,
    KX022_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
    KX022_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
};

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
