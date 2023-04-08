#ifndef __CLOUD_H__
#define __CLOUD_H__

#include "esp_err.h"
#include "esp_qcloud_log.h"
#include "esp_qcloud_console.h"
#include "esp_qcloud_storage.h"
#include "esp_qcloud_iothub.h"
#include "esp_qcloud_prov.h"

#define REBOOT_UNBROKEN_COUNT_RESET 5

static esp_err_t get_wifi_config(wifi_config_t *wifi_cfg, uint32_t wait_ms);
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t cloud_set_param(const char *id, const esp_qcloud_param_val_t *val);
static esp_err_t cloud_get_param(const char *id, esp_qcloud_param_val_t *val);
void Task_Cloud(void *pvParameters);

#endif /* __CLOUD_H__ */