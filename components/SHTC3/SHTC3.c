#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "SHTC3.h"

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t shtc3_read_out_id(uint8_t *id_reg)
{
    uint8_t write_buffer[2] = {(uint8_t)(SHTC3_READ_ID_REGISTER >> 8), (uint8_t)SHTC3_READ_ID_REGISTER};
    size_t read_size = 3;

    return i2c_master_write_read_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, write_buffer, sizeof(write_buffer), id_reg, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t shtc3_write_cmd(uint16_t shtc3_cmd)
{
    uint8_t write_buffer[2] = {(uint8_t)(shtc3_cmd >> 8), (uint8_t)shtc3_cmd};

    return i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t shtc3_sleep(void)
{
    return shtc3_write_cmd(SHTC3_SLEEP_COMMAND);
}

esp_err_t shtc3_wakeup(void)
{
    return shtc3_write_cmd(SHTC3_WAKEUP_COMMAND);
}

// Normal Mode/ Read RH First/ Clock Stretching Enabled/ 0x5C24
esp_err_t shtc3_measure_normal_rh_en_clocks(uint8_t *Humidity, uint8_t *Temperature)
{
    uint8_t read_buf[6];
    uint8_t i;
    esp_err_t err = ESP_OK;

    err = shtc3_write_cmd(SHTC3_WAKEUP_COMMAND);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    err = shtc3_write_cmd(SHTC3_MEASURE_CMD_4);
    vTaskDelay(40 / portTICK_PERIOD_MS);

    err = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, read_buf, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    err = shtc3_write_cmd(SHTC3_SLEEP_COMMAND);

    for (i = 0; i < 6; i++)
    {
        if (i < 3)
        {
            Humidity[i] = read_buf[i];
        }
        else if (i >= 3 && i < 6)
        {
            Temperature[i - 3] = read_buf[i];
        }
    }
    return err;
}

esp_err_t shtc3_crc_check(unsigned char Inputdata[], unsigned char BytesNbr, unsigned char CheckSum)
{
    unsigned char bit;
    unsigned char crc = 0xFF;
    unsigned char byteCtr;

    esp_err_t err;

    for (byteCtr = 0; byteCtr < BytesNbr; byteCtr++)
    {
        crc ^= Inputdata[byteCtr];
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }

    if (crc != CheckSum)
    {
        err = ESP_FAIL;
    }
    else
    {
        err = ESP_OK;
    }

    return err;
}

// /**
//  * @brief Read a sequence of bytes from a MPU9250 sensor registers
//  */
// static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
//     return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }