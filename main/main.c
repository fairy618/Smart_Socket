#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "BH1750.h"
#include "SHTC3.h"

#define EXAMPLE_ESP_WIFI_SSID "llx"
#define EXAMPLE_ESP_WIFI_PASS "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL 6
#define EXAMPLE_MAX_STA_CONN 4




static const char *TAG = "wifi softAP";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init_softap(void);

void app_main(void)
{
    uint8_t shtc3_id_reg[3] = {0};
    uint8_t Humidity[3];
    uint8_t Temperature[3];
    uint16_t hum, temp;
    float temp_f = 0;
    float hum_f = 0;
    uint16_t light;

    uint32_t io_s = 1;

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

    bh1750_power_cmd(BH1750_INS_POWER_ON);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bh1750_power_cmd(BH1750_INS_RESET);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bh1750_cnt_meas(BH1750_INS_CNT_H1_MOD);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    gpio_reset_pin(GPIO_NUM_HLW8032_TX);

    gpio_set_direction(GPIO_NUM_HLW8032_TX, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_MODE_OUTPUT, io_s);

    while (1)
    {
        // shtc3_measure_normal_rh_en_clocks(Humidity, Temperature);

        // hum = (Humidity[0] << 8) + Humidity[1];
        // temp = (Temperature[0] << 8) + Temperature[1];

        // temp_f = temp / 65536.0f * 175.0f - 45.0f;
        // hum_f = hum / 65536.0f * 100.0f;

        // printf("Temperature = %.2f ℃", temp_f);
        // if (shtc3_crc_check(Humidity, 2, Humidity[2]))
        // {
        //     printf("\nHumidity data is error!\n");
        // }
        // else
        // {
        //     printf(" ✔\t");
        // }

        // printf("Humidity = %.2f %%", hum_f);
        // if (shtc3_crc_check(Temperature, 2, Temperature[2]))
        // {
        //     printf("\nTemperature data is error!\n");
        // }
        // else
        // {
        //     printf("✔\n");
        // }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // bh1750_cnt_meas(BH1750_INS_CNT_H1_MOD);
        // vTaskDelay(200 / portTICK_PERIOD_MS);
        // bh1750_read_data(&light);
        // printf("ligth = %d lm\t", light);
        // vTaskDelay(700 / portTICK_PERIOD_MS);
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

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// void app_main(void)
// {
//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
//     wifi_init_softap();
// }
