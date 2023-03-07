#ifndef _DRIVER_HLW8032_H_
#define _DRIVER_HLW8032_H_

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

#define HLW8032_UATR_DATA_LEN (24)

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
    uint16_t PF;
    uint8_t CheckSum;
} HLW8032_data_t;

typedef struct
{
    float VoltageRMS;
    float CurrentRMS;
    float ActivePower;
    float ApparentPower;
    float PowerFactor;
} ElectricalParameter_t;

void Task_Hlw8032(void *pvParameters);
esp_err_t hlw8032_data_processing(HLW8032_data_t *hlw8032_row_data_, ElectricalParameter_t *ElectricalParameter, uint8_t row_data[], int DataLen);

// static uint8_t hlw8032_lowbytes_checksum(uint8_t data[], uint8_t length);
// static uint32_t hlw8032_uint8_2_uint32(uint8_t HighByte, uint8_t MidByte, uint8_t LowByte);
// static void hlw8032_init(void);

#endif /*_DRIVER_HLW8032_H_*/
