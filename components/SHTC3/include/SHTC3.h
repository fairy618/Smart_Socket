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

esp_err_t i2c_master_init(void);
esp_err_t bh1750_read_out_id(uint8_t *data);

#endif /*_DRIVER_SHTC3_H_*/
