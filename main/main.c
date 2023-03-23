#include "BH1750.h"
#include "SHTC3.h"
#include "HLW032.h"
#include "BasicDrive.h"
#include "alMQTT.h"

// QueueSetHandle_t xQueueSet = NULL;

// QueueHandle_t xQueueElectric = NULL;
// QueueHandle_t xQueueRelay = NULL;
// QueueHandle_t xQueueRgb = NULL;
QueueHandle_t xQueueRelay_g = NULL;
QueueHandle_t xQueuerRgb_g = NULL;
QueueHandle_t xQueueSensor_g = NULL;

void app_main(void)
{
    int LedTaskBlinkTime = 1000;

    // xQueueSet = xQueueCreateSet(20);

    // xQueueElectric = xQueueCreate(5, sizeof(ElectricalParameter_t));
    xQueueRelay_g = xQueueCreate(5, sizeof(bool));
    xQueuerRgb_g = xQueueCreate(5, sizeof(rgb_data_t));
    xQueueSensor_g = xQueueCreate(5, sizeof(Sensor_data_t));

    // if ((xQueueSensor == NULL) || (xQueueElectric == NULL) || (xQueueRelay_g == NULL) || (xQueueRgb == NULL))
    // {
    //     ESP_LOGE("QUEUE", "Can not creat queue!");
    //     esp_restart();
    // }

    xTaskCreate(Task_LED, "Task_LED", 2048, (void *)&LedTaskBlinkTime, 1, NULL);

    xTaskCreate(Task_sensor, "Task_sensor", 2048 * 2, NULL, 5, NULL);

    xTaskCreate(Task_Hlw8032, "Task_Hlw8032", 2048 * 2, NULL, 2, NULL);

    xTaskCreate(Task_Relay, "Task_Relay", 2048, NULL, 1, NULL);

    xTaskCreate(Task_WS2812, "Task_WS2812", 2048, NULL, 3, NULL);

    xTaskCreate(Task_key, "Task_key", 2048, NULL, 1, NULL);

    WifiConnect();

    xTaskCreate(Task_ali_mqqt, "Task_ali_mqqt", 2048 * 2, NULL, 5, NULL);

    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}