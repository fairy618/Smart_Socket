#include <stdio.h>
#include "HLW032.h"

static const char *TAG_HLW8032 = "HLW8032";

uint32_t HLW8032_VoltageREGParameter;
uint32_t HLW8032_Voltager;
uint32_t HLW8032_CurrentREGParameter;
uint32_t HLW8032_Current;
uint32_t HLW8032_PowerREGParameter;
uint32_t HLW8032_Power;

uint64_t HLW8032_pf;

uint8_t HLW8032_CheckSum;

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

    ESP_ERROR_CHECK(uart_driver_install(HLW8032_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(HLW8032_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(HLW8032_UART_PORT_NUM, HLW8032_UART_TXD, HLW8032_UART_RXD, HLW8032_UART_RTS, HLW8032_UART_CTS));
}

/*
 * @description:
 * @param {void} *arg
 * @return {*}
 */
void Task_Hlw8032(void *arg)
{
    hlw8032_init();

    // Configure a temporary buffer for the incoming data
    uint8_t *hlw8032_uart_data = (uint8_t *)malloc(BUF_SIZE);

    unsigned char flag_State = 0x00;

    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        if (len)
        {
            ESP_LOGI(TAG_HLW8032, "UART%d received data.", HLW8032_UART_PORT_NUM);

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
            else if (hlw8032_uart_data[0] & 0xF0 == 0xF0)
            {
                flag_State = 0x10 | (hlw8032_uart_data[0] & 0x0F);
                ESP_LOGI(TAG_HLW8032, "State REG = %#X. HLW8032 error correction function is normal.  Voltage parameter REG, Current Parameter REG and Power parameter REG are available.", hlw8032_uart_data[0]);
            }
            ESP_LOGD(TAG_HLW8032, "flag_State = %#X." flag_State);

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
                ESP_LOGD(TAG_HLW8032, "HLW8032_VoltageREGParameter = %#X, HLW8032_Voltager = %#X. " HLW8032_VoltageREGParameter, HLW8032_Voltager);

                // 3 Current Parameter REG & 3 Current REG
                HLW8032_CurrentREGParameter = (hlw8032_uart_data[8] << 16) | (hlw8032_uart_data[9] << 8) | (hlw8032_uart_data[10] << 0);
                HLW8032_Current = (hlw8032_uart_data[11] << 16) | (hlw8032_uart_data[12] << 8) | (hlw8032_uart_data[13] << 0);
                ESP_LOGD(TAG_HLW8032, "HLW8032_CurrentREGParameter = %#X, HLW8032_Current = %#X. " HLW8032_CurrentREGParameter, HLW8032_Current);

                // 3 Power Parameter REG & 3 Power REG
                HLW8032_PowerREGParameter = (hlw8032_uart_data[14] << 16) | (hlw8032_uart_data[15] << 8) | (hlw8032_uart_data[16] << 0);
                HLW8032_Power = (hlw8032_uart_data[17] << 16) | (hlw8032_uart_data[18] << 8) | (hlw8032_uart_data[19] << 0);
                ESP_LOGD(TAG_HLW8032, "HLW8032_PowerREGParameter = %#X, HLW8032_Power = %#X. " HLW8032_PowerREGParameter, HLW8032_Power);

                // 1 Data Updata REG
                if (hlw8032_uart_data[20] & 0x80)
                {
                    // pf REG carry flag bit
                }
                if (hlw8032_uart_data[20] & 0x40)
                {
                    // Voltager REG updata
                }
                if (hlw8032_uart_data[20] & 0x20)
                {
                    // Current REG updata
                }
                if (hlw8032_uart_data[20] & 0x10)
                {
                    // power REG updata
                }

                // 2 PF REG
                HLW8032_pf = (hlw8032_uart_data[21] << 8) | (hlw8032_uart_data[22]);
            }
        }
    }
}
