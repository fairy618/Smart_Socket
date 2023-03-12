#ifndef _DRIVER_BASICDRIVE_H_
#define _DRIVER_BASICDRIVE_H_

#define GPIO_NUM_RELAY_PWM 4
#define GPIO_NUM_WS2812_DAT 10
#define GPIO_NUM_LED 18
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

void Task_key(void *pvParameters);
void Task_LED(void *pvParameters);
void Task_Relay(void *pvParameters);
void Task_WS2812(void *pvParameters);
void Relay_ledc_init(void);
void Relay_ledc_set_duty(uint8_t duty);

#endif /*_DRIVER_BASICDRIVE_H_*/