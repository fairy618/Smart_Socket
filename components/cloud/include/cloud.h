#ifndef __CLOUD_H__
#define __CLOUD_H__

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_qcloud_log.h"
#include "esp_qcloud_console.h"
#include "esp_qcloud_storage.h"
#include "esp_qcloud_iothub.h"
#include "esp_qcloud_prov.h"

#define CONFIG_LIGHT_PROVISIONING_SOFTAPCONFIG 1
#define CONFIG_LIGHT_PROVISIONING_SOFTAPCONFIG_SSID "SmartSocket_v230407"

#define REBOOT_UNBROKEN_COUNT_RESET 5

void Task_Cloud(void *pvParameters);

#endif /* __CLOUD_H__ */