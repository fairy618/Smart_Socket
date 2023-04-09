#include "BH1750.h"
#include "SHTC3.h"
#include "HLW032.h"
#include "BasicDrive.h"
#include "cloud.h"

void app_main(void)
{
    int LedTaskBlinkTime = 1000;

    xTaskCreate(Task_LED, "Task_LED", 2048, (void *)&LedTaskBlinkTime, 1, NULL);

    xTaskCreate(Task_sensor, "Task_sensor", 2048 * 2, NULL, 5, NULL);

    xTaskCreate(Task_Hlw8032, "Task_Hlw8032", 2048 * 2, NULL, 2, NULL);

    // xTaskCreate(Task_WS2812, "Task_WS2812", 2048, NULL, 3, NULL);

    xTaskCreate(Task_key, "Task_key", 2048, NULL, 1, NULL);

    xTaskCreate(Task_Cloud, "Task_Cloud", 2048 * 2, NULL, 6, NULL);
}