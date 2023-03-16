#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_err.h"

#include "BH1750.h"

/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_bh1750(void *pvParameters)
{
    uint16_t LightData = 0;

    bh1750_power_cmd(BH1750_INS_POWER_ON);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    bh1750_power_cmd(BH1750_INS_RESET);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    bh1750_cnt_meas(BH1750_INS_CNT_H1_MOD);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    while (1)
    {
        bh1750_read_data(&LightData);

        ESP_LOGI("BH1750", "The light intensity is %d(lm). ", LightData);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

/*
 * @description: Control BH1750 to power on, power down, or reset
 * @param {uint8_t} bh1750_cmd      It should be one of [BH1750_INS_POWER_DOWN | BH1750_INS_POWER_ON | BH1750_INS_RESET]
 * @return {*}
 */
esp_err_t bh1750_power_cmd(uint8_t bh1750_cmd)
{
    uint8_t write_buffer[1] = {BH1750_INS_POWER_DOWN};

    if ((bh1750_cmd == BH1750_INS_POWER_DOWN) || (bh1750_cmd == BH1750_INS_POWER_ON) || (bh1750_cmd == BH1750_INS_RESET))
    {
        write_buffer[0] = bh1750_cmd;
    }

    return i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDRESS, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @description: Continuous measurement mode
 * @param {uint8_t} meas_mod    It should be one of [BH1750_INS_CNT_H1_MOD | BH1750_INS_CNT_H2_MOD | BH1750_INS_CNT_L_MOD]
 * @return {*}
 */
esp_err_t bh1750_cnt_meas(uint8_t meas_mod)
{
    uint8_t write_buffer[1] = {BH1750_INS_CNT_L_MOD}; // default value: BH1750_INS_CNT_L_MOD

    if ((meas_mod == BH1750_INS_CNT_H1_MOD) || (meas_mod == BH1750_INS_CNT_H2_MOD) || (meas_mod == BH1750_INS_CNT_L_MOD))
    {
        write_buffer[0] = meas_mod;
    }

    return i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDRESS, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @description: Once measurement mode
 * @param {uint8_t} meas_mod    It should be one of [BH1750_INS_ONCE_H1_MOD | BH1750_INS_ONCE_H2_MOD | BH1750_INS_ONCE_L_MOD]
 * @return {*}
 */
esp_err_t bh1750_once_meas(uint8_t meas_mod)
{
    uint8_t write_buffer[1] = {BH1750_INS_ONCE_L_MOD};

    if ((meas_mod == BH1750_INS_ONCE_H1_MOD) || (meas_mod == BH1750_INS_ONCE_H2_MOD) || (meas_mod == BH1750_INS_ONCE_L_MOD))
    {
        write_buffer[0] = meas_mod;
    }

    return i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDRESS, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// get the light intensity in lm
/*
 * @description: Read data form IIC bus and converted into light intensity uint in lm
 * @param {uint16_t} *light_data
 * @return {*}
 */
esp_err_t bh1750_read_data(uint16_t *light_data)
{
    esp_err_t err;
    uint8_t read_buf[2];

    err = i2c_master_read_from_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDRESS, read_buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    *light_data = (read_buf[0] << 8) | (read_buf[1]);

    *light_data = (uint16_t)(*light_data / 1.2f);

    return err;
}
