#include "BH1750.h"
#include "SHTC3.h"
#include "HLW032.h"
#include "BasicDrive.h"
#include "alMQTT.h"

_Noreturn void app_main(void)
{
    int LedTaskBlinkTime = 1000;

    // xQueueRelay_g = xQueueCreate(5, sizeof(bool));
    // xQueueRgb_g = xQueueCreate(5, sizeof(rgb_data_t));
    // xQueueSensor_g = xQueueCreate(5, sizeof(Sensor_data_t));
    // xQueueElectric_g = xQueueCreate(5, sizeof(ElectricalParameter_t));

    //  if ((xQueueElectric_g == NULL) || (xQueueRelay_g == NULL) || (xQueueRgb_g == NULL) || (xQueueSensor_g == NULL))
    //  {
    //      ESP_LOGE("QUEUE", "Can not creat queue!");
    //      esp_restart();
    //  }

    xTaskCreate(Task_LED, "Task_LED", 2048, (void *)&LedTaskBlinkTime, 1, NULL);

    xTaskCreate(Task_sensor, "Task_sensor", 2048 * 2, NULL, 5, NULL);

    xTaskCreate(Task_Hlw8032, "Task_Hlw8032", 2048 * 2, NULL, 2, NULL);

    xTaskCreate(Task_Relay, "Task_Relay", 2048, NULL, 1, NULL);

   xTaskCreate(Task_WS2812, "Task_WS2812", 2048, NULL, 3, NULL);

    xTaskCreate(Task_key, "Task_key", 2048, NULL, 1, NULL);

    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}