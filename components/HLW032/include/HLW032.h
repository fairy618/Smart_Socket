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

#define GET_BIT(x, bit) ((x & (0x01 << bit)) >> bit) /* 获取第bit位 */

#define GPIO_NUM_HLW8032_TX (GPIO_NUM_0)
#define GPIO_NUM_HLW8032_PF (GPIO_NUM_1)

#define UART_BUF_SIZE (1024)

#define HLW8032_UART_BAUD_RATE (4800)
#define HLW8032_UART_PORT_NUM (UART_NUM_1)
#define HLW8032_UART_RXD (GPIO_NUM_HLW8032_TX)
#define HLW8032_UART_TXD (UART_PIN_NO_CHANGE)
#define HLW8032_UART_RTS (UART_PIN_NO_CHANGE)
#define HLW8032_UART_CTS (UART_PIN_NO_CHANGE)

#define HLW8032_VOLTAGE_COEF (1.88f)
#define HLW8032_CURRENT_COFE (1.00f)

#define HLW8032_UART_DATA_LEN (24)

extern QueueHandle_t xQueueElectric_g;


typedef struct
{
    uint8_t State;
    uint8_t Check;
    uint32_t VoltageParameter;
    uint32_t Voltage;
    uint32_t CurrentParameter;
    uint32_t Current;
    uint32_t PowerParameter;
    uint32_t Power;
    uint8_t DataUpdata;
    uint16_t PF_reg_value;
    uint8_t CheckSum;
    uint16_t PF_reverse_cnt;
} HLW8032_data_t;

typedef struct
{
    float VoltageRMS;
    float CurrentRMS;
    float ActivePower;
    float ApparentPower;
    float PowerFactor;
    uint64_t PF_value;
    float ElectricityConsumption;
} ElectricalParameter_t;

void Task_Hlw8032(void *pvParameters);

#endif /*_DRIVER_HLW8032_H_*/
