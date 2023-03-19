#include "BH1750.h"
#include "SHTC3.h"
#include "HLW032.h"
#include "BasicDrive.h"
#include "alMQTT.h"

QueueSetHandle_t xQueueSet = NULL;

QueueHandle_t xQueueSensor = NULL;
QueueHandle_t xQueueElectric = NULL;
QueueHandle_t xQueueRelay = NULL;
QueueHandle_t xQueueRgb = NULL;

void app_main(void)
{
    int LedTaskBlinkTime = 1000;

    xQueueSet = xQueueCreateSet(20);

    xQueueSensor = xQueueCreate(5, sizeof(Sensor_data_t));
    xQueueElectric = xQueueCreate(5, sizeof(ElectricalParameter_t));
    xQueueRelay = xQueueCreate(5, sizeof(bool));
    xQueueRgb = xQueueCreate(5, sizeof(rgb_data_t));

    if ((xQueueSensor == NULL) || (xQueueElectric == NULL) || (xQueueRelay == NULL) || (xQueueRgb == NULL))
    {
        ESP_LOGE("QUEUE", "Can not creat queue!");
        esp_restart();
    }

    xQueueAddToSet(xQueueSensor, xQueueSet);
    xQueueAddToSet(xQueueElectric, xQueueSet);
    xQueueAddToSet(xQueueRelay, xQueueSet);
    xQueueAddToSet(xQueueRgb, xQueueSet);

    xTaskCreate(Task_LED, "Task_LED", 2048, (void *)&LedTaskBlinkTime, 1, NULL);

    xTaskCreate(Task_sensor, "Task_sensor", 2048 * 2, (void *)xQueueSensor, 5, NULL);

    xTaskCreate(Task_Hlw8032, "Task_Hlw8032", 2048 * 2, (void *)xQueueElectric, 2, NULL);

    xTaskCreate(Task_Relay, "Task_Relay", 2048, (void *)xQueueSet, 1, NULL);

    xTaskCreate(Task_WS2812, "Task_WS2812", 2048, (void *)xQueueSet, 3, NULL);

    xTaskCreate(Task_key, "Task_key", 2048, (void *)xQueueRelay, 1, NULL);

    WifiConnect();

    xTaskCreate(Task_ali_mqqt, "Task_ali_mqqt", 2048 * 2, (void *)xQueueSet, 5, NULL);

    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}