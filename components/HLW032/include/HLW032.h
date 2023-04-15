#ifndef _DRIVER_HLW8032_H_
#define _DRIVER_HLW8032_H_

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_err.h"

#include "BasicDrive.h"

#define GET_BIT(x, bit) ((x & (0x01 << bit)) >> bit) /* 获取第bit位 */

#define GPIO_NUM_HLW8032_TX (GPIO_NUM_0)
#define GPIO_NUM_HLW8032_PF (GPIO_NUM_1)

#define ESP_INTR_FLAG_DEFAULT (0)

#define UART_BUF_SIZE (1024)

#define HLW8032_UART_BAUD_RATE (4800)
#define HLW8032_UART_PORT_NUM (UART_NUM_1)
#define HLW8032_UART_RXD (GPIO_NUM_HLW8032_TX)
#define HLW8032_UART_TXD (UART_PIN_NO_CHANGE)
#define HLW8032_UART_RTS (UART_PIN_NO_CHANGE)
#define HLW8032_UART_CTS (UART_PIN_NO_CHANGE)

// Non-isolated sampling
// #define HLW8032_VOLTAGE_COEF (1.88f)
// #define HLW8032_CURRENT_COFE (1.00f)

// Isolated sampling
#define HLW8032_VOLTAGE_REG_PARAMETER (0x02E6F8)
#define HLW8032_CURRENT_REG_PARAMETER (0x003E6C)
#define HLW8032_POWER_REG_PARAMETER (0x4F1230)

#define HLW8032_K_V (1.960161f)
#define HLW8032_K_I (1.021739021f)
#define HLW8032_K_P (2.021356643f)

#define HLW8032_UART_DATA_LEN (24)

typedef struct
{
    uint8_t State;      // 状态寄存器
    uint8_t Check;      // 检测寄存器
    uint32_t Voltage;   // 电压寄存器
    uint32_t Current;   // 电流寄存器
    uint32_t Power;     // 功率寄存器
    uint8_t DataUpdata; // 数据更新寄存器
    uint8_t CheckSum;   // 包尾
} HLW8032_data_t;

typedef struct
{
    float VoltageRMS; // 电压 有效值
    float CurrentRMS;
    float ActivePower;
    float ApparentPower;
    float PowerFactor;
    float ElectricityConsumption;
    bool flag;
} ElectricalParameter_t;

void Task_Hlw8032(void *pvParameters);
float hlw8032_get_voltage(void);
float hlw8032_get_current(void);
float hlw8032_get_active_power(void);
float hlw8032_get_apparent_power(void);
float hlw8032_get_power_factor(void);
float hlw8032_get_Electricity_consumption(void);
char *hlw8032_get_debugIfo(void);

#endif /*_DRIVER_HLW8032_H_*/
