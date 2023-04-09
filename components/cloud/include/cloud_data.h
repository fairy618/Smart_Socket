#ifndef __CLOUD_DATA_H__
#define __CLOUD_DATA_H__

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

extern QueueHandle_t MailBox;

float sensor_get_env_temp(void);

#endif /* __CLOUD_DATA_H__ */