#include "HLW032.h"

QueueHandle_t xQueue_UART2HLW8032 = NULL;
ElectricalParameter_t ElectricalParameter;
HLW8032_data_t hlw8032_row_data;

uint32_t pf_counter_isr = 0;

bool EleSensorDataRefreshFlag = false;

static esp_err_t hlw8032_data_processing(uint8_t row_data[], int DataLen);
static uint8_t hlw8032_lowbytes_checksum(uint8_t data[], uint8_t length);
static uint32_t hlw8032_uint8_2_uint32(uint8_t HighByte, uint8_t MidByte, uint8_t LowByte);
static void hlw8032_init(void);

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    pf_counter_isr++;
}

/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_Hlw8032(void *pvParameters)
{
    uart_event_t Event_uart1;

    size_t buffered_size;
    uint8_t *hlw8032_uart_data = (uint8_t *)malloc(UART_BUF_SIZE);

    bool HighWaterMark = 1;
    static int UartRecCnt = 10;

    hlw8032_init();

    while (1)
    {
        if (xQueueReceive(xQueue_UART2HLW8032, (void *)&Event_uart1, (TickType_t)portMAX_DELAY) == pdPASS)
        {
            UartRecCnt++;
            if (UartRecCnt % 20 == 0)
            {
                switch (Event_uart1.type)
                {
                case UART_DATA:
                    ESP_LOGI("UART1 EVENT", "uart data: %d. ", Event_uart1.size);
                    int len = uart_read_bytes(HLW8032_UART_PORT_NUM, hlw8032_uart_data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
                    ESP_LOGI("UART_DATA", "len = %d. ", len);
                    hlw8032_data_processing(hlw8032_uart_data, len);
                    break;
                case UART_BREAK:
                    ESP_LOGE("UART1 EVENT", "uart rx break");
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGE("UART1 EVENT", "ring buffer full");
                    uart_flush_input(HLW8032_UART_PORT_NUM);
                    xQueueReset(xQueue_UART2HLW8032);
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE("UART1 EVENT", "hw fifo overflow");
                    uart_flush_input(HLW8032_UART_PORT_NUM);
                    xQueueReset(xQueue_UART2HLW8032);
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
                    ESP_LOGI("UART_PATTERN_DET", "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos,
                             buffered_size);
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
                    ESP_LOGE("UART1 EVENT", "UNKNOW");
                    break;
                }
            }
            else
            {
                uart_flush_input(HLW8032_UART_PORT_NUM);
                xQueueReset(xQueue_UART2HLW8032);
            }
            memset(hlw8032_uart_data, 0, UART_BUF_SIZE);
        }
        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("HLW8032 HighWaterMark", "Stack`s free depth : %d/4096. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description: Processing hlw8032`s raw data
 * @param {HLW8032_data_t} *hlw8032_row_data           Save the raw data in format
 * @param {ElectricalParameter_t} *ElectricalParameter  The calculated power data
 * @param {uint8_t} row_data                            Raw data received from the UART1
 * @param {int} DataLen                                 The length of row_data[]
 * @return {*}
 */
static esp_err_t hlw8032_data_processing(uint8_t row_data[], int DataLen)
{
    static esp_err_t ret = ESP_OK;
    static uint8_t index = 0;
    static uint8_t errorflag = false;

    if (RelayState)
    {
        // Check data length
        if (DataLen != HLW8032_UART_DATA_LEN)
        {
            ESP_LOGE("HLW8032 DATA", "The length of data is %d. It should be 24!", DataLen);
            ret = ESP_FAIL;
        }
        else
        {
            // The data length meets expectations
            // Then check the "check REG"
            hlw8032_row_data.Check = row_data[1];
            if (hlw8032_row_data.Check != 0x5A)
            {
                ESP_LOGE("HLW8032 DATA", "The value of Check REG  is %#X, it should be 0x5A. ", hlw8032_row_data.Check);
                ret = ESP_FAIL;
            }
            else
            {
                // The data length meets expectations
                // The "check REG" is also as expected
                // Then check the data checksum
                uint8_t CheckSum = hlw8032_lowbytes_checksum(row_data, DataLen);
                hlw8032_row_data.CheckSum = row_data[23];
                if (hlw8032_row_data.CheckSum != CheckSum)
                {
                    ESP_LOGE("HLW8032 DATA", "The checksum is %#X, but the value of checksum REG  is %#X. ", CheckSum, hlw8032_row_data.CheckSum);
                    ret = ESP_FAIL;
                }
                else
                {
                    // The data length meets expectations
                    // The "check REG" is also as expected
                    // Data validity
                    // Calculate the electric energy data according to "State REG"

                    // ESP_LOGI("UART_DATA", "here 001 ");
                    // for (uint8_t cnt = 0; cnt < 24; cnt++)
                    // {
                    //     ESP_LOGI("row_data", "row_data [%d] = %X", cnt, row_data[cnt]);
                    // }
                    // ESP_LOGI("UART_DATA", "here 002 ");

                    hlw8032_row_data.State = row_data[0];
                    hlw8032_row_data.DataUpdata = row_data[20];
                    errorflag = false;

                    switch (hlw8032_row_data.State)
                    {
                    case 0xAA:
                        ESP_LOGE("HLW8032 DATA", " The value of State REG is %#X. The chip is error.", hlw8032_row_data.State);
                        errorflag = true;
                        ret = ESP_FAIL;
                        break;

                    case 0x55:
                        index = 5;
                        hlw8032_row_data.Voltage = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                        index = 11;
                        hlw8032_row_data.Current = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                        index = 17;
                        hlw8032_row_data.Power = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                        ESP_LOGI("HLW8032 DATA", " The value of State REG is 0x%X. The chip is fine and no overflow.", hlw8032_row_data.State);
                        break;

                    default:
                        if ((hlw8032_row_data.State & 0xF0) == 0xF0)
                        {
                            index = 5;
                            hlw8032_row_data.Voltage = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                            index = 11;
                            hlw8032_row_data.Current = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                            index = 17;
                            hlw8032_row_data.Power = hlw8032_uint8_2_uint32(row_data[index + 0], row_data[index + 1], row_data[index + 2]);

                            if (GET_BIT(hlw8032_row_data.State, 3))
                            {
                                ESP_LOGE("HLW8023 DATA VOERFLOW", "Voltage REG overflow. ");
                            }
                            if (GET_BIT(hlw8032_row_data.State, 2))
                            {
                                ESP_LOGE("HLW8023 DATA VOERFLOW", "Current REG overflow. ");
                            }
                            if (GET_BIT(hlw8032_row_data.State, 1))
                            {
                                ESP_LOGE("HLW8023 DATA VOERFLOW", "Power REG overflow. ");
                            }
                            if (GET_BIT(hlw8032_row_data.State, 0))
                            {
                                errorflag = true;
                                ESP_LOGE("HLW8023 DATA ERROR", "Parameters REGs all error. ");
                            }
                        }
                        else
                        {
                            errorflag = true;
                            ESP_LOGE("HLW8023 STATE REG ERROR", "The value of state REG is %#X. ", hlw8032_row_data.State);
                        }
                        break;
                    }

                    if (errorflag)
                    {
                        ElectricalParameter.VoltageRMS = -1;
                        ElectricalParameter.CurrentRMS = -1;
                        ElectricalParameter.ActivePower = -1;
                        ElectricalParameter.ApparentPower = -1;
                        ElectricalParameter.PowerFactor = -1;
                    }
                    else
                    {
                        if (GET_BIT(hlw8032_row_data.DataUpdata, 6))
                        {
                            ElectricalParameter.VoltageRMS = HLW8032_VOLTAGE_REG_PARAMETER * HLW8032_K_V / hlw8032_row_data.Voltage;
                        }
                        if (GET_BIT(hlw8032_row_data.DataUpdata, 5))
                        {
                            ElectricalParameter.CurrentRMS = HLW8032_CURRENT_REG_PARAMETER * HLW8032_K_I / hlw8032_row_data.Current;
                        }
                        if (GET_BIT(hlw8032_row_data.DataUpdata, 4))
                        {
                            ElectricalParameter.ActivePower = HLW8032_POWER_REG_PARAMETER * HLW8032_K_P / hlw8032_row_data.Power;
                        }

                        ElectricalParameter.ApparentPower = ElectricalParameter.VoltageRMS * ElectricalParameter.CurrentRMS;

                        if (ElectricalParameter.ApparentPower <= 0.0001)
                        {
                            ElectricalParameter.PowerFactor = 0;
                        }
                        else
                        {
                            ElectricalParameter.PowerFactor = ElectricalParameter.ActivePower / ElectricalParameter.ApparentPower;
                        }
                        EleSensorDataRefreshFlag = true;
                    }
                }
            }
        }
    }
    else
    {
        ElectricalParameter.VoltageRMS = 0;
        ElectricalParameter.CurrentRMS = 0;
        ElectricalParameter.ActivePower = 0;
        ElectricalParameter.ApparentPower = 0;
        ElectricalParameter.PowerFactor = 0;
    }

    ESP_LOGI("ElectricalParameter", "VoltageRMS    = %.2f. ", ElectricalParameter.VoltageRMS);
    ESP_LOGI("ElectricalParameter", "CurrentRMS    = %.2f. ", ElectricalParameter.CurrentRMS);
    ESP_LOGI("ElectricalParameter", "ActivePower   = %.2f. ", ElectricalParameter.ActivePower);
    ESP_LOGI("ElectricalParameter", "ApparentPower = %.2f. ", ElectricalParameter.ApparentPower);
    ESP_LOGI("ElectricalParameter", "PowerFactor   = %.2f. ", ElectricalParameter.PowerFactor);
    ESP_LOGI("ElectricalParameter", "PF Counter    = %lu. ", pf_counter_isr);

    return ret;
}

/*
 * @description:Initialize UART1 (and GPIO) for hlw8032
 * @return {*}
 */
static void hlw8032_init(void)
{
    gpio_reset_pin(GPIO_NUM_HLW8032_PF);
    gpio_set_direction(GPIO_NUM_HLW8032_PF, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_NUM_HLW8032_PF, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_HLW8032_PF, gpio_isr_handler, NULL);

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

    ESP_ERROR_CHECK(uart_driver_install(HLW8032_UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 20, &xQueue_UART2HLW8032, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(HLW8032_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(HLW8032_UART_PORT_NUM, HLW8032_UART_TXD, HLW8032_UART_RXD, HLW8032_UART_RTS, HLW8032_UART_CTS));
}

/*
 * @description: Calculate the checksum of the received data
 * @param {uint8_t} data        Raw data received from the UART1
 * @param {uint8_t} length      The length of data[]
 * @return {*}
 */
static uint8_t hlw8032_lowbytes_checksum(uint8_t data[], uint8_t length)
{
    uint32_t DataSum = 0;

    for (uint8_t i = 2; i < length - 1; i++)
    {
        DataSum += data[i];
    }

    return (uint8_t)DataSum;
}

/*
 * @description: Merge three 8-bit data into one 32-bit data
 * @param {uint8_t} HighByte
 * @param {uint8_t} MidByte
 * @param {uint8_t} LowByte
 * @return {*}
 */
static uint32_t hlw8032_uint8_2_uint32(uint8_t HighByte, uint8_t MidByte, uint8_t LowByte)
{
    return (HighByte << 16) | (MidByte << 8) | (LowByte);
}

float hlw8032_get_voltage(void)
{
    return (ElectricalParameter.VoltageRMS);
}
float hlw8032_get_current(void)
{
    return (ElectricalParameter.CurrentRMS);
}
float hlw8032_get_active_power(void)
{
    return (ElectricalParameter.ActivePower);
}
float hlw8032_get_apparent_power(void)
{
    return (ElectricalParameter.ApparentPower);
}
float hlw8032_get_power_factor(void)
{
    return (ElectricalParameter.PowerFactor);
}
float hlw8032_get_Electricity_consumption(void)
{
    return (ElectricalParameter.ElectricityConsumption);
}

char *hlw8032_get_debugIfo(void)
{
    static char debugInfomotain[2048];

    // sprintf(debugInfomotain, "vol:%lu, Cur:%lu,Power:%lu.", hlw8032_row_data.Voltage, hlw8032_row_data.Current, hlw8032_row_data.Power);
    sprintf(debugInfomotain, "V:%lu, I:%lu, P:%lu, PF:%lu. ## State=0x%X, DataUpdata=0x%X",
            hlw8032_row_data.Voltage,
            hlw8032_row_data.Current,
            hlw8032_row_data.Power,
            pf_counter_isr,
            hlw8032_row_data.State,
            hlw8032_row_data.DataUpdata);

    ESP_LOGI("debug", "%s", debugInfomotain);

    return debugInfomotain;
}
