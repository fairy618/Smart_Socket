#include "led_strip_encoder.h"
#include "BasicDrive.h"

static uint8_t led_strip_pixels[RMT_LED_NUMBERS * 3];

/*
 * @description: polling
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_key(void *pvParameters)
{
    // QueueHandle_t xQueueRelay = (QueueHandle_t)pvParameters;
    static uint8_t LongPressCnt = 0;
    uint8_t KeyValue = 0;
    bool RelayState = 0;

    gpio_reset_pin(GPIO_NUM_KEY);
    gpio_set_direction(GPIO_NUM_KEY, GPIO_MODE_INPUT);

    while (1)
    {
        if (gpio_get_level(GPIO_NUM_KEY) == 0)
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            LongPressCnt = 0;
            if (gpio_get_level(GPIO_NUM_KEY) == 0)
            {
                ESP_LOGI("TASK KEY", "Key is press. ");
                KeyValue = 1;
                while (gpio_get_level(GPIO_NUM_KEY) == 0)
                {
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                    if (LongPressCnt < 0xFF)
                    {
                        LongPressCnt++;
                    }
                    if (LongPressCnt == 100)
                    {
                        KeyValue = 2;
                        ESP_LOGI("TASK KEY", "Key is long press. ");
                    }
                }

                if (LongPressCnt < 100)
                {
                    ESP_LOGI("TASK KEY", "Key is free. ");
                }
                else
                {
                    ESP_LOGI("TASK KEY", "Key is free after long press. ");
                }

                if (KeyValue == 1)
                {
                    RelayState = !RelayState;
                    if (xQueueSend(xQueueRelay_g, (void *)&RelayState, 0) == pdPASS)
                    {
                        ESP_LOGI("KEY", " --- Send RelayState to xQueue done! --- ");
                    }
                    else
                    {
                        ESP_LOGE("KEY", " --- Send RelayState to xQueue fail! --- ");
                    }
                }
                else if (KeyValue == 2)
                {
                    // wifi config
                }
                KeyValue = 0;
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_Relay(void *pvParameters)
{
    bool HighWaterMark = 1;
    bool RelayState = 0;

    Relay_ledc_init();
    Relay_ledc_set_duty(0);

    while (1)
    {
        if (xQueueReceive(xQueueRelay_g, &RelayState, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI("RELAY", "Rec Data, RelayState is %d. ", (RelayState == 0 ? 0 : 1));
            if (RelayState)
            {
                Relay_ledc_set_duty(80);
            }
            else
            {
                Relay_ledc_set_duty(0);
            }
        }
        else
        {
            ESP_LOGE("RELAY", "Rec Data timeout. ");
        }

        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("RELAY HighWaterMark", "Stack`s free depth : %d/2048. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description: blink
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_LED(void *pvParameters)
{
    bool HighWaterMark = 1;

    int LedBlinkTime = *((int *)pvParameters);

    gpio_reset_pin(GPIO_NUM_LED);
    gpio_set_direction(GPIO_NUM_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_LED, 1);

    while (1)
    {
        gpio_set_level(GPIO_NUM_LED, 1);
        vTaskDelay(LedBlinkTime / portTICK_PERIOD_MS);

        gpio_set_level(GPIO_NUM_LED, 0);
        vTaskDelay(LedBlinkTime / portTICK_PERIOD_MS);

        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("LED HighWaterMark", "Stack`s free depth : %d/2048. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

/*
 * @description:
 * @param {void} *pvParameters
 * @return {*}
 */
void Task_WS2812(void *pvParameters)
{
    rgb_data_t RgbData = {0, 0, 0};

    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI("WS2812", "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI("WS2812", "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI("WS2812", "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI("WS2812", "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    led_strip_pixels[0] = 0;
    led_strip_pixels[1] = 100;
    led_strip_pixels[2] = 100;
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

    while (1)
    {
        if (xQueueReceive(xQueueRgb_g, &RgbData, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI("RELAY", "Rec Data, RgbData is %d-%d-%d. ", (int)RgbData.red, (int)RgbData.green, (int)RgbData.blue);
            led_strip_pixels[0] = (uint8_t)RgbData.green; // green
            led_strip_pixels[1] = (uint8_t)RgbData.red;   // red
            led_strip_pixels[2] = (uint8_t)RgbData.blue;
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        }
        else
        {
            ESP_LOGE("RELAY", "Rec Data timeout. ");
        }
    }
}

/*
 * @description:
 * @return {*}
 */
void Relay_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at ? kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/*
 * @description: set the duty of relay
 * @param {uint8_t} duty, range 0 ~ 100
 * @return {*}
 */
void Relay_ledc_set_duty(uint8_t duty)
{
    uint32_t ledc_duty;

    if (duty > 100)
    {
        duty = 100;
    }

    ledc_duty = (uint32_t)(10.23f * duty);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, ledc_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
