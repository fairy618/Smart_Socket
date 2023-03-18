#include <string.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include "BH1750.h"
#include "SHTC3.h"
#include "HLW032.h"
#include "BasicDrive.h"
#include "alMQTT.h"

void app_main(void)
{
    int LedTaskBlinkTime = 1000;

    WifiConnect();

    xTaskCreate(Task_ali_mqqt, "Task_ali_mqqt", 2048 * 2, NULL, 5, NULL);

    // xTaskCreate(Task_shtc3, "Task_shtc3", 2048, (void *)&Queue_shtc3_2_mqtt, 2, NULL);
    xTaskCreate(Task_shtc3, "Task_shtc3", 2048, NULL, 2, NULL);
    xTaskCreate(Task_bh1750, "Task_bh1750", 2048, NULL, 2, NULL);

    xTaskCreate(Task_LED, "Task_LED", 2048, (void *)&LedTaskBlinkTime, 1, NULL);
    // xTaskCreate(Task_Relay, "Task_Relay", 2048, NULL, 1, NULL);

    xTaskCreate(Task_key, "Task_key", 2048, NULL, 3, NULL);
    xTaskCreate(Task_WS2812, "Task_WS2812", 2048, NULL, 4, NULL);
}