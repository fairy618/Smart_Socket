#include "SHTC3.h"
#include "BH1750.h"

/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_sensor(void *pvParameters)
{
    QueueHandle_t xQueueSensor = (QueueHandle_t)pvParameters;

    uint8_t ID_Register[2];
    shtc3_t struct_shtc3_data;
    Sensor_data_t Sensor_data;
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = {-10, 80, TEMPERATURE_SENSOR_CLK_SRC_DEFAULT};
    bool HighWaterMark = 1;

    i2c_master_init();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    shtc3_wakeup();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    shtc3_read_out_id(ID_Register);
    ESP_LOGI("SHTC3 ID", "The ID Reg is 0x%02x%02x ", ID_Register[0], ID_Register[1]);

    shtc3_sleep();

    ESP_LOGI("CHIP TEMP", "Install temperature sensor, expected temp ranger range: -10~80 ℃");

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI("CHIP TEMP", "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    bh1750_power_cmd(BH1750_INS_POWER_ON);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    bh1750_power_cmd(BH1750_INS_RESET);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    bh1750_cnt_meas(BH1750_INS_CNT_H1_MOD);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    while (1)
    {
        vTaskDelay(SENSOR_INTERVAL_TIME_MS / portTICK_PERIOD_MS);

        shtc3_measure_normal_rh_dis_clocks(struct_shtc3_data.row_data);

        struct_shtc3_data.flag_humidity = shtc3_crc_check(struct_shtc3_data.row_data, 2, struct_shtc3_data.row_data[2]);
        struct_shtc3_data.flag_temperature = shtc3_crc_check(struct_shtc3_data.row_data + 3 * sizeof(uint8_t), 2, struct_shtc3_data.row_data[5]);

        if (struct_shtc3_data.flag_humidity != ESP_OK || struct_shtc3_data.flag_temperature != ESP_OK)
        {
            ESP_LOGE("SHTC3 CRC CHECK", "There are somgthing wrong whit shtc3");
            continue;
        }
        else // Valid crc check
        {
            struct_shtc3_data.row_humidity = (struct_shtc3_data.row_data[0] << 8) | struct_shtc3_data.row_data[1];
            struct_shtc3_data.row_temperature = (struct_shtc3_data.row_data[3] << 8) + struct_shtc3_data.row_data[4];

            struct_shtc3_data.humidity = (float)(struct_shtc3_data.row_humidity * 100.0 / 65536.0);
            struct_shtc3_data.temperature = struct_shtc3_data.row_temperature * 175.0 / 65536.0 - 45.0;

            temperature_sensor_get_celsius(temp_sensor, &Sensor_data.ChipTemperature);
            Sensor_data.EnvHumidity = struct_shtc3_data.humidity;
            Sensor_data.EnvironmentTemperature = struct_shtc3_data.temperature;
            bh1750_read_data(&Sensor_data.LightIntensity);

            if (xQueueSend(xQueueSensor, (void *)&Sensor_data, 0) == pdPASS)
            {
                ESP_LOGI("SENSORe", " --- Send Sensor_data to xQueue done! --- ");
            }
            else
            {
                ESP_LOGE("SENSORe", " --- Send Sensor_data to xQueue failed! --- ");
            }

            ESP_LOGI("SENSOR", "EnvironmentTemperature is %.2f℃, ChipTemperature is %.2f℃, EnvHumidity is %.2f%%, LightIntensity is %d. ", Sensor_data.EnvironmentTemperature, Sensor_data.ChipTemperature, Sensor_data.EnvHumidity, Sensor_data.LightIntensity);
        }
        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("SHTC3 HighWaterMark", "Stack`s free depth : %d/2048. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description: Initialize the IIC bus for SHTC3 (and BH1750)
 * @return {*}
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
 * @description: Read the ID register of SHTC3
 * @param {uint8_t} *id_reg     save thr ID of SHTC3
 * @return {*}
 */
esp_err_t shtc3_read_out_id(uint8_t *id_reg)
{
    uint8_t write_buffer[2] = {(uint8_t)(SHTC3_READ_ID_REGISTER >> 8), (uint8_t)SHTC3_READ_ID_REGISTER};
    size_t read_size = 3;

    return i2c_master_write_read_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, write_buffer, sizeof(write_buffer), id_reg, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @description:
 * @param {uint16_t} shtc3_cmd
 * @return {*}
 */
esp_err_t shtc3_write_cmd(uint16_t shtc3_cmd)
{
    uint8_t write_buffer[2] = {(uint8_t)(shtc3_cmd >> 8), (uint8_t)shtc3_cmd};

    return i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @description:
 * @return {*}
 */
esp_err_t shtc3_sleep(void)
{
    return shtc3_write_cmd(SHTC3_SLEEP_COMMAND);
}

/*
 * @description:
 * @return {*}
 */
esp_err_t shtc3_wakeup(void)
{
    return shtc3_write_cmd(SHTC3_WAKEUP_COMMAND);
}

/*
 * @description:  Normal Mode/ Read RH First/ Clock Stretching Disabled/ 0x58E0
 * @param {uint8_t} *read_buf
 * @return {*}
 */
esp_err_t shtc3_measure_normal_rh_dis_clocks(uint8_t *read_buf)
{
    esp_err_t err = ESP_OK;

    err = shtc3_wakeup();
    vTaskDelay(50 / portTICK_PERIOD_MS);

    err = shtc3_write_cmd(SHTC3_MEASURE_CMD_4);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // RH first
    err = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, read_buf, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    err = shtc3_sleep();

    return err;
}

/*
 * @description: CRC calculates the data of shtc3
 * @param {unsigned char} Inputdata     intputdata
 * @param {unsigned char} ByteNbr       the length of intputdata`s need CRC
 * @param {unsigned char} CheckSum      the Checknum from SHTC3
 * @return {*}
 */
esp_err_t shtc3_crc_check(unsigned char Inputdata[], unsigned char ByteNbr, unsigned char CheckSum)
{
    unsigned char bit, byte;
    unsigned char crc = 0xFF;

    esp_err_t err;

    for (byte = 0; byte < ByteNbr; byte++)
    {
        crc ^= Inputdata[byte];
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
