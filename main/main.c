#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "BH1750.h"
#include "SHTC3.h"

void app_main(void)
{
    uint8_t shtc3_id_reg[3] = {0};
    uint8_t Humidity[3];
    uint8_t Temperature[3];
    uint16_t hum, temp;
    float temp_f = 0;
    float hum_f = 0;

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }

    printf("%luMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());

    i2c_master_init();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    shtc3_wakeup();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    shtc3_read_out_id(shtc3_id_reg);
    printf("shtc3_id_reg is %#x%x\tCRC is %#x\n", shtc3_id_reg[0], shtc3_id_reg[1], shtc3_id_reg[2]);

    while (1)
    {
        shtc3_measure_normal_rh_en_clocks(Humidity, Temperature);

        hum = (Humidity[0] << 8) + Humidity[1];
        temp = (Temperature[0] << 8) + Temperature[1];

        temp_f = temp / 65536.0f * 175.0f - 45.0f;
        hum_f = hum / 65536.0f * 100.0f;

        printf("Temperature = %.2f℃", temp_f);
        if (shtc3_crc_check(Humidity, 2, Humidity[2]))
        {
            printf("\nHumidity data is error!\n");
        }
        else
        {
            printf(" ✔\t");
        }

        printf("Humidity = %.2f%%", hum_f);
        if (shtc3_crc_check(Temperature, 2, Temperature[2]))
        {
            printf("\nTemperature data is error!\n");
        }
        else
        {
            printf("✔\n");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    // for (int i = 10; i >= 0; i--)
    // {
    //     // printf("Restarting in %d seconds...\n", i);
    //     func();
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}
