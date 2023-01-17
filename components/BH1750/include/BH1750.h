#ifndef _DRIVER_BH1750_H_
#define _DRIVER_BH1750_H_

#define I2C_MASTER_NUM 0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS 1000
#define BH1750_SENSOR_ADDRESS 0x23

#define BH1750_INS_POWER_DOWN 0x00
#define BH1750_INS_POWER_ON 0x01
#define BH1750_INS_RESET 0x07
#define BH1750_INS_CNT_H1_MOD 0x10
#define BH1750_INS_CNT_H2_MOD 0x11
#define BH1750_INS_CNT_L_MOD 0x13
#define BH1750_INS_ONCE_H1_MOD 0x20
#define BH1750_INS_ONCE_H2_MOD 0x21
#define BH1750_INS_ONCE_L_MOD 0x23

esp_err_t bh1750_power_cmd(uint8_t bh1750_cmd);
esp_err_t bh1750_cnt_meas(uint8_t meas_mod);
esp_err_t bh1750_once_meas(uint8_t meas_mod);
esp_err_t bh1750_read_data(uint16_t *light_data);

#endif /*_DRIVER_BH1750_H_*/
