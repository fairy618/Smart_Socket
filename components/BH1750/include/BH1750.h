#ifndef _DRIVER_BH1750_H_
#define _DRIVER_BH1750_H_

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_NUM 0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS 1000
#define BH1750_SENSOR_ADDRESS 0x23

#define BH1750_INS_POWER_DOWN 0x00
#define BH1750_INS_POWER_ON 0x01    // Waiting for measurement command
#define BH1750_INS_RESET 0x07       // Reset Data register value. Reset command is not acceptable in Power Down mode.
#define BH1750_INS_CNT_H1_MOD 0x10  // Start measurement at 1lx resolution. Measurement Time is typically 120ms
#define BH1750_INS_CNT_H2_MOD 0x11  // Start measurement at 0.5lx resolution. Measurement Time is typically 120ms.
#define BH1750_INS_CNT_L_MOD 0x13   // Start measurement at 4lx resolution. Measurement Time is typically 16ms
#define BH1750_INS_ONCE_H1_MOD 0x20 // Start measurement at 1lx resolution. Measurement Time is typically 120ms. It is automatically set to Power Down mode after measurement
#define BH1750_INS_ONCE_H2_MOD 0x21 // Start measurement at 0.5lx resolution. Measurement Time is typically 120ms. It is automatically set to Power Down mode after measurement.
#define BH1750_INS_ONCE_L_MOD 0x23  // Start measurement at 4lx resolution. Measurement Time is typically 16ms. It is automatically set to Power Down mode after measurement.

void Task_bh1750(void *pvParameters);
esp_err_t bh1750_power_cmd(uint8_t bh1750_cmd);
esp_err_t bh1750_cnt_meas(uint8_t meas_mod);
esp_err_t bh1750_once_meas(uint8_t meas_mod);
esp_err_t bh1750_read_data(uint16_t *light_data);

#endif /*_DRIVER_BH1750_H_*/
