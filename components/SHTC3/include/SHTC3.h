#ifndef _DRIVER_SHTC3_H_
#define _DRIVER_SHTC3_H_

#define I2C_MASTER_SCL_IO 19        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 18        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define SHTC3_SENSOR_ADDR 0x70 /*!< Slave address of the SHTC3 sensor */
#define SHTC3_READ_ID_REGISTER 0xEFC8
#define SHTC3_SLEEP_COMMAND 0xB098
#define SHTC3_WAKEUP_COMMAND 0x3517
#define SHTC3_RESET_COMMAND 0x805D /*!< Soft Reset, olny use when shtc3 is idle state */
#define SHTC3_MEASURE_CMD_1 0x7CA2
#define SHTC3_MEASURE_CMD_2 0x5C24
#define SHTC3_MEASURE_CMD_3 0x7866
#define SHTC3_MEASURE_CMD_4 0x58E0
#define SHTC3_MEASURE_CMD_5 0x6458
#define SHTC3_MEASURE_CMD_6 0x44DE
#define SHTC3_MEASURE_CMD_7 0x609C
#define SHTC3_MEASURE_CMD_8 0x401A

esp_err_t i2c_master_init(void);
esp_err_t shtc3_read_out_id(uint8_t *data);
esp_err_t shtc3_write_cmd(uint16_t shtc3_cmd);
esp_err_t shtc3_sleep(void);
esp_err_t shtc3_wakeup(void);
esp_err_t shtc3_measure_normal_rh_en_clocks(uint8_t *Humidity, uint8_t *Temperature);
esp_err_t shtc3_crc_check(unsigned char Inputdata[], unsigned char BytesNbr, unsigned char CheckSum);

#endif /*_DRIVER_SHTC3_H_*/
