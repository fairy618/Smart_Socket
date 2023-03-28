#ifndef _DRIVER_BASICDRIVE_H_
#define _DRIVER_BASICDRIVE_H_

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#include "esp_log.h"
#include "esp_err.h"

#define GPIO_NUM_RELAY_PWM 4
#define GPIO_NUM_WS2812_DAT 10
#define GPIO_NUM_LED 19
#define GPIO_NUM_KEY 8

#define RELAY_OPEN 0
#define RELAY_CLOSE 1

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO GPIO_NUM_RELAY_PWM // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY (20000)          // Frequency in Hertz. Set frequency at 5 kHz
#define LEDC_DUTY (767)                 // Set duty to 50%. ((2 ** 10) - 1) * 80% = 818

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 10
#define RMT_LED_NUMBERS 8
#define RMT_LED_CHASE_SPEED_MS 10

typedef struct
{
    uint32_t red;
    uint32_t green;
    uint32_t blue;
} rgb_data_t;


extern QueueHandle_t xQueueRelay_g;
extern QueueHandle_t xQueueRgb_g;

void Task_key(void *pvParameters);
void Task_LED(void *pvParameters);
void Task_Relay(void *pvParameters);
void Task_WS2812(void *pvParameters);
void Relay_ledc_init(void);
void Relay_ledc_set_duty(uint8_t duty);
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

#endif /*_DRIVER_BASICDRIVE_H_*/
