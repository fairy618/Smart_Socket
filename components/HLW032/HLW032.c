#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#include "HLW032.h"

static const char *TAG_HLW8032 = "HLW8032";

QueueHandle_t xQueue_HLW8032 = NULL;

uint32_t HLW8032_VoltageREGParameter;
uint32_t HLW8032_Voltager;
uint32_t HLW8032_CurrentREGParameter;
uint32_t HLW8032_Current;
uint32_t HLW8032_PowerREGParameter;
uint32_t HLW8032_Power;

uint8_t HLW8032_CheckSum;

uint16_t voltage_eff_tenfold;     // 电压有效值 十倍放大
uint32_t current_eff_hundredfold; // 电流有效值 百倍放大
uint16_t active_power_tenfold;    // 有功功率 十倍放大
uint16_t apparent_power_tenfold;  // 视在功率 十倍放大
float power_factor;               // 功率因素
uint32_t PF_invert_cnt = 0;       // pf 取反次数
uint32_t HLW8032_pf_reg;          // pf 寄存器的值
uint64_t electricity_consumption; // 用电量

/*
 * @description:
 * @param {void} *arg
 * @return {*}
 */
void Task_Hlw8032(void *arg)
{
    uint8_t *hlw8032_uart_data = (uint8_t *)malloc(UART_BUF_SIZE);
    unsigned char flag_State = 0x00;
    uint8_t i = 0;
    uart_event_t Event_uart1;
    BaseType_t Queue_rec;
    bool HighWaterMark = 1;

    hlw8032_init();
    ESP_LOGI("Task_Hlw8032", "HLW8032 init finish. ");

    while (1)
    {
        Queue_rec = xQueueReceive(xQueue_HLW8032, (void *)&Event_uart1, portMAX_DELAY);

        if (Queue_rec == pdPASS)
        {
            switch (Event_uart1.type)
            {
            case UART_DATA:
                int len = uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
                if (len)
                {
                    ESP_LOGI(TAG_HLW8032, "UART[%d] received data.", HLW8032_UART_PORT_NUM);

                    flag_State = 0x00;
                    // 1 State REG
                    if (hlw8032_uart_data[0] == 0xAA)
                    {
                        ESP_LOGE(TAG_HLW8032, "State REG = %#X. HLW8032 error correction function fails! Voltage parameter REG, Current Parameter REG and Power parameter REG are unavailable", hlw8032_uart_data[0]);
                    }
                    else if (hlw8032_uart_data[0] == 0x55)
                    {
                        flag_State = 0x10;
                        ESP_LOGE(TAG_HLW8032, "State REG = %#X. HLW8032 error correction function is normal. Voltage parameter REG, Current Parameter REG and Power parameter REG are unavailable.Voltage REG, Current REG and Power REG do not overflow.", hlw8032_uart_data[0]);
                    }
                    else if ((hlw8032_uart_data[0] & 0xF0) == 0xF0)
                    {
                        flag_State = 0x10 | (hlw8032_uart_data[0] & 0x0F);
                        ESP_LOGI(TAG_HLW8032, "State REG = %#X. HLW8032 error correction function is normal.  Voltage parameter REG, Current Parameter REG and Power parameter REG are available.", hlw8032_uart_data[0]);
                    }
                    ESP_LOGD(TAG_HLW8032, "flag_State = %#X.", flag_State);

                    // 1 Check REG
                    if (hlw8032_uart_data[1] != 0x5A)
                    {
                        ESP_LOGE(TAG_HLW8032, "Check REG = %#X. It should be equal to 0x5A.", hlw8032_uart_data[1]);
                    }

                    if (flag_State != 0x00)
                    {
                        // 1 CheckSum REG
                        HLW8032_CheckSum = 0;
                        for (uint8_t i = 2; i < 23; i++)
                        {
                            HLW8032_CheckSum += hlw8032_uart_data[i];
                        }
                        if (HLW8032_CheckSum != hlw8032_uart_data[23])
                        {
                            ESP_LOGE(TAG_HLW8032, "CheckSum REG = %#X, but HLW8032_CheckSum = %#X. This time the data should be discarded .", hlw8032_uart_data[23], HLW8032_CheckSum);
                        }

                        // 3 Voltage Parameter REG & 3 Voltage REG
                        HLW8032_VoltageREGParameter = (hlw8032_uart_data[2] << 16) | (hlw8032_uart_data[3] << 8) | (hlw8032_uart_data[4] << 0);
                        HLW8032_Voltager = (hlw8032_uart_data[5] << 16) | (hlw8032_uart_data[6] << 8) | (hlw8032_uart_data[7] << 0);
                        ESP_LOGD(TAG_HLW8032, "HLW8032_VoltageREGParameter = %ld, HLW8032_Voltager = %ld. ", HLW8032_VoltageREGParameter, HLW8032_Voltager);

                        // 3 Current Parameter REG & 3 Current REG
                        HLW8032_CurrentREGParameter = (hlw8032_uart_data[8] << 16) | (hlw8032_uart_data[9] << 8) | (hlw8032_uart_data[10] << 0);
                        HLW8032_Current = (hlw8032_uart_data[11] << 16) | (hlw8032_uart_data[12] << 8) | (hlw8032_uart_data[13] << 0);
                        ESP_LOGD(TAG_HLW8032, "HLW8032_CurrentREGParameter = %ld, HLW8032_Current = %ld. ", HLW8032_CurrentREGParameter, HLW8032_Current);

                        // 3 Power Parameter REG & 3 Power REG
                        HLW8032_PowerREGParameter = (hlw8032_uart_data[14] << 16) | (hlw8032_uart_data[15] << 8) | (hlw8032_uart_data[16] << 0);
                        HLW8032_Power = (hlw8032_uart_data[17] << 16) | (hlw8032_uart_data[18] << 8) | (hlw8032_uart_data[19] << 0);
                        ESP_LOGD(TAG_HLW8032, "HLW8032_PowerREGParameter = %ld, HLW8032_Power = %ld. ", HLW8032_PowerREGParameter, HLW8032_Power);

                        // 1 Data Updata REG
                        if (hlw8032_uart_data[20] & 0x80)
                        {
                            // pf REG carry flag bit
                            PF_invert_cnt++;
                        }
                        if (hlw8032_uart_data[20] & 0x40)
                        {
                            // Voltager REG updata
                            voltage_eff_tenfold = (uint16_t)(HLW8032_VoltageREGParameter * 10.0f * HLW8032_VOLTAGE_COEF / HLW8032_Voltager);
                        }
                        if (hlw8032_uart_data[20] & 0x20)
                        {
                            // Current REG updata
                            current_eff_hundredfold = (uint32_t)(HLW8032_CurrentREGParameter * 100.0f * HLW8032_CURRENT_COFE / HLW8032_Current);
                        }
                        if (hlw8032_uart_data[20] & 0x10)
                        {
                            // power REG updata
                            active_power_tenfold = (uint16_t)(HLW8032_PowerREGParameter * 10.0f * HLW8032_VOLTAGE_COEF * HLW8032_CURRENT_COFE / HLW8032_Power);
                            apparent_power_tenfold = (uint16_t)(voltage_eff_tenfold * current_eff_hundredfold / 100.0f);
                            power_factor = 1.0f * active_power_tenfold / apparent_power_tenfold;
                        }

                        // 2 PF REG
                        HLW8032_pf_reg = (hlw8032_uart_data[21] << 8) | (hlw8032_uart_data[22]);
                    }
                }
                for (i = 0; i < 24; i++)
                {
                    printf("%#2x ", hlw8032_uart_data[i]);
                    hlw8032_uart_data[i] = 0;
                }
                printf("\n");
                // memset(hlw8032_uart_data, 0, UART_BUF_SIZE);
                break;

            case UART_BREAK:
                ESP_LOGE("UART1 EVENT", "UART_BREAK");
                break;
            case UART_BUFFER_FULL:
                ESP_LOGE("UART1 EVENT", "UART_BUFFER_FULL");
                break;
            case UART_FIFO_OVF:
                ESP_LOGE("UART1 EVENT", "UART_FIFO_OVF");
                break;
            case UART_FRAME_ERR:
                ESP_LOGE("UART1 EVENT", "UART_FRAME_ERR");
                break;
            case UART_PARITY_ERR:
                ESP_LOGE("UART1 EVENT", "UART_PARITY_ERR");
                break;
            case UART_DATA_BREAK:
                ESP_LOGE("UART1 EVENT", "UART_DATA_BREAK");
                break;
            case UART_PATTERN_DET:
                ESP_LOGE("UART1 EVENT", "UART_PATTERN_DET");
                break;
            case UART_WAKEUP:
                ESP_LOGE("UART1 EVENT", "UART_WAKEUP");
                break;
            case UART_EVENT_MAX:
                ESP_LOGE("UART1 EVENT", "UART_EVENT_MAX");
                break;

            default:
                break;
            }
        }
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        if (false)
        {
            // need debug
            electricity_consumption = (HLW8032_pf_reg + PF_invert_cnt * 65536) / (3.6 * (10 ^ 12) / HLW8032_PowerREGParameter / HLW8032_VOLTAGE_COEF / HLW8032_CURRENT_COFE);
        }
        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("HLW8032 HighWaterMark", "Stack`s free depth : %d/2048. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description:
 * @return {*}
 */
void hlw8032_init(void)
{
    gpio_reset_pin(GPIO_NUM_HLW8032_PF);
    gpio_set_direction(GPIO_NUM_HLW8032_PF, GPIO_MODE_INPUT);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = HLW8032_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    uart_driver_install(HLW8032_UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 20, &xQueue_HLW8032, intr_alloc_flags);
    uart_param_config(HLW8032_UART_PORT_NUM, &uart_config);
    uart_set_pin(HLW8032_UART_PORT_NUM, HLW8032_UART_TXD, HLW8032_UART_RXD, HLW8032_UART_RTS, HLW8032_UART_CTS);
}