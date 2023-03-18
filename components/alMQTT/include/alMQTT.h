#ifndef _ALI_MQQT_H_
#define _ALI_MQQT_H_

#include "aiot_state_api.h"
#include "aiot_sysdep_api.h"
#include "aiot_mqtt_api.h"
#include "aiot_dm_api.h"

// #define WIFI_SSID_ "Hirain"
// #define WIFI_PASSWORD_ "SZhirain01"
// #define WIFI_SSID_ "ChinaNet-xcYb"
// #define WIFI_PASSWORD_ "pvg249cs"
#define WIFI_SSID_ "fairy"
#define WIFI_PASSWORD_ "12345678"
#define WIFI_MAXIMUM_RETRY_ 5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

void wifi_init_sta(void);
int32_t demo_state_logcb(int32_t code, char *message);
void demo_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata);
void demo_mqtt_default_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata);
void *demo_mqtt_process_thread(void *args);
void *demo_mqtt_recv_thread(void *args);
void Task_ali_mqqt(void *pvParameters);
void WifiConnect(void);
int32_t demo_send_property_post(void *dm_handle, char *params);
static void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
int32_t demo_send_property_batch_post(void *dm_handle, char *params);
int32_t demo_send_event_post(void *dm_handle, char *event_id, char *params);
int32_t demo_send_get_desred_requset(void *dm_handle);
int32_t demo_send_delete_desred_requset(void *dm_handle);
int32_t pal_post_property_EnvTemperature(void *dm_handle, float value);

#endif /*_ALI_MQQT_H_*/