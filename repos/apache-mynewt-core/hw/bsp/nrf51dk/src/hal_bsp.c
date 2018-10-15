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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "os/mynewt.h"
#include "hal/hal_bsp.h"
#include "hal/hal_system.h"
#include "mcu/nrf51_hal.h"
#include "bsp/bsp.h"
#include "flash_map/flash_map.h"
#include "hal/hal_flash.h"
#include "hal/hal_spi.h"
#include "hal/hal_i2c.h"

#if MYNEWT_VAL(ADC_0)
#include <adc_nrf51/adc_nrf51.h>
#include <nrfx_adc.h>
#endif

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1)
#include "uart/uart.h"
#endif
#if MYNEWT_VAL(UART_0)
#include "uart_hal/uart_hal.h"
#endif


#if MYNEWT_VAL(LIS2DH12_ONB)
#include <lis2dh12/lis2dh12.h>
static struct lis2dh12 lis2dh12;
#endif

#if MYNEWT_VAL(KX022_ONB)
#include <kx022/kx022.h>
static struct kx022 kx022;
#endif


#if MYNEWT_VAL(UART_0)
static struct uart_dev os_bsp_uart0;
static const struct nrf51_uart_cfg os_bsp_uart0_cfg = {
    .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
    .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
    .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
    .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
};
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
/*
 * NOTE: Our HAL expects that the SS pin, if used, is treated as a gpio line
 * and is handled outside the SPI routines.
 */
static const struct nrf51_hal_spi_cfg os_bsp_spi0m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_0_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MISO),
};
#endif

#if MYNEWT_VAL(SPI_1_SLAVE)
static const struct nrf51_hal_spi_cfg os_bsp_spi1s_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_1_SLAVE_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_1_SLAVE_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_1_SLAVE_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_1_SLAVE_PIN_SS),
};
#endif

#if MYNEWT_VAL(I2C_0)
static const struct nrf51_hal_i2c_cfg hal_i2c_cfg = {
    .scl_pin = MYNEWT_VAL(I2C_0_PIN_SCL),
    .sda_pin = MYNEWT_VAL(I2C_0_PIN_SDA),
    .i2c_frequency = MYNEWT_VAL(I2C_0_FREQ_KHZ),
};

#if MYNEWT_VAL(LIS2DH12_ONB)
static struct sensor_itf i2c_0_itf_lis = {
	.si_type = SENSOR_ITF_I2C,
	.si_num = 0,
	.si_addr = 0x19
};
#endif

#if MYNEWT_VAL(KX022_ONB)
static struct sensor_itf i2c_0_itf_kx = {
	.si_type = SENSOR_ITF_I2C,
	.si_num = 0,
	.si_addr = 0x1f
};


#endif
#endif


#if MYNEWT_VAL(ADC_0)
static struct adc_dev os_bsp_adc0;
static struct nrf51_adc_dev_cfg os_bsp_adc0_config = {
    .nadc_refmv0    = MYNEWT_VAL(ADC_0_REFMV_0),
    .nadc_refmv1    = MYNEWT_VAL(ADC_0_REFMV_1),
    .nadc_refmv_vdd = MYNEWT_VAL(ADC_0_REFMV_VDD)
};
#endif




/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    }
};

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &nrf51_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

int
hal_bsp_power_state(int state)
{
    return (0);
}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    uint32_t cfg_pri;

    switch (irq_num) {
    /* Radio gets highest priority */
    case RADIO_IRQn:
        cfg_pri = 0;
        break;
    default:
        cfg_pri = pri;
    }
    return cfg_pri;
}





/**
 * KX022 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_kx022_sensor(void)
{
#if MYNEWT_VAL(KX022_ONB)
    int rc;
    struct os_dev *dev;
    struct kx022_cfg cfg;

    dev = (struct os_dev *) os_dev_open("kx022_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.mask = SENSOR_TYPE_ACCELEROMETER;
    cfg.rate = KX022_1020_OUTPUT_RATE_50_HZ;
    cfg.range = KX022_FS_2G;
   // cfg.lc_pull_up_disc = 1;

    rc = kx022_config((struct kx022 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}




/**
 * LIS2Dh12 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_lis2dh12_sensor(void)
{
#if MYNEWT_VAL(LIS2DH12_ONB)
    int rc;
    struct os_dev *dev;
    struct lis2dh12_cfg cfg;

    dev = (struct os_dev *) os_dev_open("lis2dh12_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.lc_s_mask = SENSOR_TYPE_ACCELEROMETER;
    cfg.lc_rate = LIS2DH12_DATA_RATE_HN_1344HZ_L_5376HZ;
    cfg.lc_fs = LIS2DH12_FS_2G;
    cfg.lc_pull_up_disc = 1;

    rc = lis2dh12_config((struct lis2dh12 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

static void
sensor_dev_create(void)
{
    int rc;
    (void)rc;

#if MYNEWT_VAL(LIS2DH12_ONB)
    rc = os_dev_create((struct os_dev *) &lis2dh12, "lis2dh12_0",
      OS_DEV_INIT_PRIMARY, 0, lis2dh12_init, (void *)&i2c_0_itf_lis);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(KX022_ONB)
    rc = os_dev_create((struct os_dev *) &kx022, "kx022_0",
      OS_DEV_INIT_PRIMARY, 0, kx022_init, (void *)&i2c_0_itf_kx);
    assert(rc == 0);
#endif
}




void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

    /* Make sure system clocks have started */
    hal_system_clock_start();

#if MYNEWT_VAL(ADC_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_adc0, "adc0",
      OS_DEV_INIT_KERNEL,
      OS_DEV_INIT_PRIO_DEFAULT,
      nrf51_adc_dev_init,
      &os_bsp_adc0_config);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart0_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(TIMER_0)
    rc = hal_timer_init(0, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_1)
    rc = hal_timer_init(1, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_2)
    rc = hal_timer_init(2, NULL);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(TIMER_3)
    rc = hal_timer_init(3, NULL);
    assert(rc == 0);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_SLAVE)
    rc = hal_spi_init(1, (void *)&os_bsp_spi1s_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_0)
    rc = hal_i2c_init(0, (void *)&hal_i2c_cfg);
    assert(rc == 0);
#endif
    sensor_dev_create();
}
