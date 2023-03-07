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

QueueHandle_t xQueue_HLW8032 = NULL;

static void hlw8032_init(void);
static uint8_t hlw8032_lowbytes_checksum(uint8_t data[], uint8_t length);
static uint32_t hlw8032_uint8_2_uint32(uint8_t HighByte, uint8_t MidByte, uint8_t LowByte);

/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_Hlw8032(void *pvParameters)
{
    static int UartRecCnt = 0;
    HLW8032_data_t hlw8032_row_data;
    ElectricalParameter_t ElectricalParameter;
    uart_event_t Event_uart1;
    // unsigned char flag_State = 0x00;
    uint8_t *hlw8032_uart_data = (uint8_t *)malloc(UART_BUF_SIZE);
    bool HighWaterMark = 1;

    size_t buffered_size;
    // uart_intr_config_t

    hlw8032_init();
    ESP_LOGI("Task_Hlw8032", "HLW8032 init finish. ");

    while (1)
    {
        /*
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(HLW8032_UART_PORT_NUM, (size_t*)&length));

        uart_flush(); // clear FIFO
        */
        if (xQueueReceive(xQueue_HLW8032, (void *)&Event_uart1, (TickType_t)portMAX_DELAY) == pdPASS)
        {
            // memset(hlw8032_uart_data, 0, UART_BUF_SIZE);
            UartRecCnt++;
            // if (UartRecCnt % 20 == 0)
            if (1)
            {
                switch (Event_uart1.type)
                {
                case UART_DATA:
                    ESP_LOGI("UART1 EVENT", "uart data: %d. ", Event_uart1.size);
                    int len = uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
                    // ESP_ERROR_CHECK(hlw8032_data_processing(&hlw8032_row_data, &ElectricalParameter, hlw8032_uart_data, len));
                    hlw8032_data_processing(&hlw8032_row_data, &ElectricalParameter, hlw8032_uart_data, len);
                    ESP_LOGI("UART_DATA", "len = %d. ", len);
                    break;
                case UART_BREAK:
                    ESP_LOGE("UART1 EVENT", "uart rx break");
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGE("UART1 EVENT", "ring buffer full");
                    uart_flush_input(HLW8032_UART_PORT_NUM);
                    xQueueReset(xQueue_HLW8032);
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE("UART1 EVENT", "hw fifo overflow");
                    uart_flush_input(HLW8032_UART_PORT_NUM);
                    xQueueReset(xQueue_HLW8032);
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGE("UART1 EVENT", "uart frame error");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGE("UART1 EVENT", "uart parity error");
                    break;
                case UART_DATA_BREAK:
                    ESP_LOGE("UART1 EVENT", "UART_DATA_BREAK");
                    break;
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(HLW8032_UART_PORT_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(HLW8032_UART_PORT_NUM);
                    ESP_LOGI("UART_PATTERN_DET", "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1)
                    {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(HLW8032_UART_PORT_NUM);
                    }
                    else
                    {
                        uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[3 + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(HLW8032_UART_PORT_NUM, pat, 3, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI("UART_PATTERN_DET", "read data: %s", hlw8032_uart_data);
                        ESP_LOGI("UART_PATTERN_DET", "read pat : %s", pat);
                    }
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
            else
            {
                uart_flush_input(HLW8032_UART_PORT_NUM);
                xQueueReset(xQueue_HLW8032);
            }
            memset(hlw8032_uart_data, 0, UART_BUF_SIZE);
            // vTaskDelay(10 / portTICK_PERIOD_MS);

            // if (false)
            // {
            //     // need debug
            //     electricity_consumption = (HLW8032_pf_reg + PF_invert_cnt * 65536) / (3.6 * (10 ^ 12) / HLW8032_PowerREGParameter / HLW8032_VOLTAGE_COEF / HLW8032_CURRENT_COFE);
            // }
        }
        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("HLW8032 HighWaterMark", "Stack`s free depth : %d/4096. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description:
 * @return {*}
 */
static void hlw8032_init(void)
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

    ESP_ERROR_CHECK(uart_driver_install(HLW8032_UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 20, &xQueue_HLW8032, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(HLW8032_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(HLW8032_UART_PORT_NUM, HLW8032_UART_TXD, HLW8032_UART_RXD, HLW8032_UART_RTS, HLW8032_UART_CTS));
}

/**
 *
 * int len = uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
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
*/

esp_err_t hlw8032_data_processing(HLW8032_data_t *hlw8032_row_data_, ElectricalParameter_t *ElectricalParameter, uint8_t row_data[], int DataLen)
{
    esp_err_t ret = ESP_OK;
    uint8_t index = 0;

    ESP_LOGD("hlw8032_data_processing", "Begin.");

    if (DataLen != HLW8032_UATR_DATA_LEN)
    {
        ESP_LOGE("HLW8032 DATA", "The length of data is %d. It should be 24!", DataLen);
        ret = ESP_FAIL;
    }
    else
    {
        hlw8032_row_data_->Check = row_data[1];
        if (hlw8032_row_data_->Check != 0x5A)
        {
            ESP_LOGE("HLW8032 DATA", "The value of Check REG  is %#X, it should be 0x5A. ", hlw8032_row_data_->Check);
            ret = ESP_FAIL;
        }
        else
        {
            uint8_t CheckSum = hlw8032_lowbytes_checksum(row_data, DataLen);
            hlw8032_row_data_->CheckSum = row_data[23];
            if (hlw8032_row_data_->CheckSum != CheckSum)
            {
                ESP_LOGE("HLW8032 DATA", "The checksum is %#X, but the value of checksum REG  is %#X. ", CheckSum, hlw8032_row_data_->CheckSum);
                ret = ESP_FAIL;
            }
            else
            {
                hlw8032_row_data_->State = row_data[0];
                switch (hlw8032_row_data_->State)
                {
                case 0xAA:
                    ESP_LOGE("HLW8032 DATA", " The value of State REG is %#X. The chip is error.", hlw8032_row_data_->State);
                    ret = ESP_FAIL;
                    break;

                case 0x55:
                    ESP_LOGI("HLW8032 DATA", " The value of State REG is %#X. The chip is fine and no overflow.", hlw8032_row_data_->State);

                    index = 2;
                    hlw8032_row_data_->VoltageParameter = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);
                    index = 5;
                    hlw8032_row_data_->Voltage = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                    index = 8;
                    hlw8032_row_data_->CurrentParameter = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);
                    index = 11;
                    hlw8032_row_data_->Current = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                    index = 14;
                    hlw8032_row_data_->PowerParameter = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);
                    index = 17;
                    hlw8032_row_data_->Power = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                    ElectricalParameter->VoltageRMS = hlw8032_row_data_->VoltageParameter * HLW8032_VOLTAGE_COEF / hlw8032_row_data_->Voltage;
                    ElectricalParameter->CurrentRMS = hlw8032_row_data_->CurrentParameter * HLW8032_CURRENT_COFE / hlw8032_row_data_->Current;
                    ElectricalParameter->ActivePower = hlw8032_row_data_->PowerParameter * HLW8032_VOLTAGE_COEF * HLW8032_CURRENT_COFE / hlw8032_row_data_->Power;
                    ElectricalParameter->ApparentPower = ElectricalParameter->VoltageRMS * ElectricalParameter->CurrentRMS;
                    ElectricalParameter->PowerFactor = ElectricalParameter->ActivePower / ElectricalParameter->ApparentPower;

                    ESP_LOGI("HLW8023 ELEPAR", "Voltage RMS is %f. ", ElectricalParameter->VoltageRMS);
                    ESP_LOGI("HLW8023 ELEPAR", "Current RMS is %f. ", ElectricalParameter->CurrentRMS);
                    ESP_LOGI("HLW8023 ELEPAR", "Active Power is %f. ", ElectricalParameter->ActivePower);
                    ESP_LOGI("HLW8023 ELEPAR", "Apparent Power is %f. ", ElectricalParameter->ApparentPower);
                    ESP_LOGI("HLW8023 ELEPAR", "Power Factor is %f. ", ElectricalParameter->PowerFactor);
                    break;

                default:
                    ESP_LOGI("HLW8023 ELEPAR", "default");
                    if ((hlw8032_row_data_->State & 0xF0) == 0xF0)
                    {
                        if (GET_BIT(hlw8032_row_data_->State, 3))
                        {
                            ESP_LOGE("HLW8023 DATA VOERFLOW", "Voltage REG overflow. ");
                        }
                        if (GET_BIT(hlw8032_row_data_->State, 2))
                        {
                            ESP_LOGE("HLW8023 DATA VOERFLOW", "Current REG overflow. ");
                        }
                        if (GET_BIT(hlw8032_row_data_->State, 1))
                        {
                            ESP_LOGE("HLW8023 DATA VOERFLOW", "Power REG overflow. ");
                        }
                        if (GET_BIT(hlw8032_row_data_->State, 0))
                        {
                            ESP_LOGE("HLW8023 DATA ERROR", "Parameters REGs all error. ");
                        }
                    }
                    else
                    {
                        ESP_LOGE("HLW8023 STATE REG ERROR", "The value of state REG is %#X. ", hlw8032_row_data_->State);
                    }

                    break;
                }
            }
        }
    }

    return ret;
}

static uint8_t hlw8032_lowbytes_checksum(uint8_t data[], uint8_t length)
{
    uint32_t DataSum = 0;

    for (uint8_t i = 2; i < length - 1; i++)
    {
        DataSum += data[i];
    }

    return (uint8_t)DataSum;
}

static uint32_t hlw8032_uint8_2_uint32(uint8_t HighByte, uint8_t MidByte, uint8_t LowByte)
{
    // return (((uint32_t)(HighByte & 0x000000FF) << 16) | ((uint32_t)(MidByte & 0x000000FF) << 8) | ((uint32_t)(LowByte & 0x000000FF) << 0));
    return (HighByte << 16) | (MidByte << 8) | (LowByte);
}
