#ifndef __CLOUD_H__
#define __CLOUD_H__

#define REBOOT_UNBROKEN_COUNT_RESET 5

static esp_err_t get_wifi_config(wifi_config_t *wifi_cfg, uint32_t wait_ms);
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t cloud_set_param(const char *id, const esp_qcloud_param_val_t *val);
static esp_err_t cloud_get_param(const char *id, esp_qcloud_param_val_t *val);
void Task_Cloud(void *pvParameters);

#endif /* __CLOUD_H__ */