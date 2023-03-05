#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "BasicDrive.h"

void Task_key(void *arg)
{

    while (1)
    {
        /* code */
    }
}

void Task_Relay(void *arg)
{
    // gpio_reset_pin(GPIO_NUM_RELAY_PWM);
    // gpio_set_direction(GPIO_NUM_RELAY_PWM, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_NUM_RELAY_PWM, RELAY_OPEN);

    Relay_ledc_init();
    // Relay_ledc_set_duty(0);

    while (1)
    {
    }
}

void Task_LED(void *arg)
{
    bool HighWaterMark = 1;

    gpio_reset_pin(GPIO_NUM_LED);
    gpio_set_direction(GPIO_NUM_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_LED, 1);

    while (1)
    {
        gpio_set_level(GPIO_NUM_LED, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(GPIO_NUM_LED, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (HighWaterMark)
        {
            HighWaterMark = 0;
            ESP_LOGI("LED HighWaterMark", "Stack`s free depth : %d/2048. ", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

void Task_WS2812(void *arg)
{
    //
    while (1)
    {
        //
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
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
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

    ledc_duty = (uint32_t)(10.23 * duty);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, ledc_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}