#include <stdio.h>
#include "cloud.h"

#include "SHTC3.h"
#include "BasicDrive.h"
#include "HLW032.h"

static esp_err_t get_wifi_config(wifi_config_t *wifi_cfg, uint32_t wait_ms);
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t cloud_set_param(const char *id, const esp_qcloud_param_val_t *val);
static esp_err_t cloud_get_param(const char *id, esp_qcloud_param_val_t *val);

void Task_Cloud(void *pvParameters)
{
    /**
     * @brief Add debug function, you can use serial command and remote debugging.
     */
    esp_qcloud_log_config_t log_config = {
        .log_level_uart = ESP_LOG_INFO,
    };
    ESP_ERROR_CHECK(esp_qcloud_log_init(&log_config));
    /**
     * @brief Set log level
     * @note  This function can not raise log level above the level set using
     * CONFIG_LOG_DEFAULT_LEVEL setting in menuconfig.
     */
    esp_log_level_set("*", ESP_LOG_VERBOSE);

#ifdef CONFIG_LIGHT_DEBUG
    ESP_ERROR_CHECK(esp_qcloud_console_init());
    esp_qcloud_print_system_info(10000);
#endif /**< CONFIG_LIGHT_DEBUG */

    // /**
    //  * @brief Initialize Application specific hardware drivers and set initial state.
    //  */
    // ESP_ERROR_CHECK(example_driver_init());

    /**< Continuous power off and restart more than five times to reset the device */
    if (esp_qcloud_reboot_unbroken_count() >= REBOOT_UNBROKEN_COUNT_RESET)
    {
        ESP_LOGW("cloud", "Erase information saved in flash");
        esp_qcloud_storage_erase(CONFIG_QCLOUD_NVS_NAMESPACE);
    }

    /*
     * @breif Create a device through the server and obtain configuration parameters
     *        server: https://console.cloud.tencent.com/iotexplorer
     */
    /**< Create and configure device authentication information */
    ESP_ERROR_CHECK(esp_qcloud_create_device());
    /**< Configure the version of the device, and use this information to determine whether to OTA */
    ESP_ERROR_CHECK(esp_qcloud_device_add_fw_version("0.0.1"));
    /**< Register the properties of the device */
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("power_switch", QCLOUD_VAL_TYPE_BOOLEAN));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("voltage", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("current", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("EnvRH", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("EnvTemp", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("ChipTemp", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("Light", QCLOUD_VAL_TYPE_INTEGER));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("active_power", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("apparent_power", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("power_factor", QCLOUD_VAL_TYPE_FLOAT));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property("Electricity_consumption", QCLOUD_VAL_TYPE_FLOAT));
    /**< The processing function of the communication between the device and the server */
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property_cb(light_get_param, light_set_param));
    ESP_ERROR_CHECK(esp_qcloud_device_add_property_cb(cloud_get_param, cloud_set_param));

    /**
     * @brief Initialize Wi-Fi.
     */
    ESP_ERROR_CHECK(esp_qcloud_wifi_init());
    ESP_ERROR_CHECK(esp_event_handler_register(QCLOUD_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /**
     * @brief Get the router configuration
     */
    wifi_config_t wifi_cfg = {0};
    ESP_ERROR_CHECK(get_wifi_config(&wifi_cfg, portMAX_DELAY));

    /**
     * @brief Connect to router
     */
    ESP_ERROR_CHECK(esp_qcloud_wifi_start(&wifi_cfg));
    ESP_ERROR_CHECK(esp_qcloud_timesync_start());

    /**
     * @brief Connect to Tencent Cloud Iothub
     */
    ESP_ERROR_CHECK(esp_qcloud_iothub_init());
    ESP_ERROR_CHECK(esp_qcloud_iothub_start());
    ESP_ERROR_CHECK(esp_qcloud_iothub_ota_enable());

    while (1)
    {
        if (EnvSensorDataRefreshFlag)
        {
            EnvSensorDataRefreshFlag = false;
            esp_qcloud_iothub_report_all_property();
        }
        if (EleSensorDataRefreshFlag)
        {
            EleSensorDataRefreshFlag = false;
            esp_qcloud_iothub_report_all_property();
        }
        if (RelayStatusRefreshFlag)
        {
            RelayStatusRefreshFlag = false;
            esp_qcloud_iothub_report_all_property();
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

/* Callback to handle commands received from the QCloud cloud */
static esp_err_t cloud_get_param(const char *id, esp_qcloud_param_val_t *val)
{

    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("power_switch", QCLOUD_VAL_TYPE_BOOLEAN));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("TimeCountDown", QCLOUD_VAL_TYPE_BOOLEAN));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("voltage", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("current", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("EnvRH", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("EnvTemp", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("ChipTemp", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("Light", QCLOUD_VAL_TYPE_INTEGER));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("active_power", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("reactive_power", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("power_factor", QCLOUD_VAL_TYPE_FLOAT));
    // ESP_ERROR_CHECK(esp_qcloud_device_add_property("Electricity_consumption", QCLOUD_VAL_TYPE_FLOAT));

    // env sensor
    if (!strcmp(id, "EnvTemp"))
    {
        val->f = sensor_get_env_temp();
    }
    else if (!strcmp(id, "EnvRH"))
    {
        val->f = sensor_get_env_rh();
    }
    else if (!strcmp(id, "ChipTemp"))
    {
        val->f = sensor_get_chip_temp();
    }
    else if (!strcmp(id, "Light"))
    {
        val->i = sensor_get_env_light();
    }
    // relay
    else if (!strcmp(id, "power_switch"))
    {
        val->b = relay_get_switch();
    }
    // ele sensor
    else if (!strcmp(id, "voltage"))
    {
        val->f = hlw8032_get_voltage();
    }
    else if (!strcmp(id, "current"))
    {
        val->f = hlw8032_get_current();
    }
    else if (!strcmp(id, "active_power"))
    {
        val->f = hlw8032_get_active_power();
    }
    else if (!strcmp(id, "apparent_power"))
    {
        val->f = hlw8032_get_apparent_power();
    }
    else if (!strcmp(id, "power_factor"))
    {
        val->f = hlw8032_get_power_factor();
    }
    else if (!strcmp(id, "Electricity_consumption"))
    {
        val->f = hlw8032_get_Electricity_consumption();
    }
    /*
    todo:
    定义结构体 把要传递的数据用结构体包裹起来

    队列邮箱

    调用函数 直接读
    */

    // if (!strcmp(id, "power_switch")) {
    //     val->b = lightbulb_get_switch();
    // } else if (!strcmp(id, "value")) {
    //     val->i = lightbulb_get_value();
    // } else if (!strcmp(id, "hue")) {
    //     val->i = lightbulb_get_hue();
    // } else if (!strcmp(id, "saturation")) {
    //     val->i = lightbulb_get_saturation();
    // }

    // ESP_LOGI("cloud", "Report id: %s, val: %d", id, val->i);

    return ESP_OK;
}

/* Callback to handle commands received from the QCloud cloud */
static esp_err_t cloud_set_param(const char *id, const esp_qcloud_param_val_t *val)
{
    esp_err_t err = ESP_FAIL;
    ESP_LOGI("cloud", "Received id: %s, val: %d", id, val->i);

    if (!strcmp(id, "power_switch"))
    {
        err = relay_set_switch(val->b);
    }

    //     err = lightbulb_set_switch(val->b);

    // if (!strcmp(id, "power_switch")) {
    //     err = lightbulb_set_switch(val->b);
    // } else if (!strcmp(id, "value")) {
    //     err = lightbulb_set_value(val->i);
    // } else if (!strcmp(id, "hue")) {
    //     err = lightbulb_set_hue(val->i);
    // } else if (!strcmp(id, "saturation")) {
    //     err = lightbulb_set_saturation(val->i);
    // } else {
    //     ESP_LOGW("cloud", "This parameter is not supported");
    // }

    // /* Report driver changes to the cloud side */
    esp_qcloud_iothub_report_all_property();
    return err;
}

/* Event handler for catching QCloud events */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case QCLOUD_EVENT_IOTHUB_INIT_DONE:
        esp_qcloud_iothub_report_device_info();
        ESP_LOGI("cloud", "QCloud Initialised");
        break;

    case QCLOUD_EVENT_IOTHUB_BOUND_DEVICE:

#ifdef CONFIG_LIGHT_PROVISIONING_SOFTAPCONFIG
        esp_qcloud_prov_softapconfig_stop();
#endif
        ESP_LOGI("cloud", "Device binding successful");
        break;

    case QCLOUD_EVENT_IOTHUB_UNBOUND_DEVICE:
        ESP_LOGW("cloud", "Device unbound with iothub");
        esp_qcloud_wifi_reset();
        esp_restart();
        break;

    case QCLOUD_EVENT_IOTHUB_BIND_EXCEPTION:
        ESP_LOGW("cloud", "Device bind fail");
        esp_qcloud_wifi_reset();
        esp_restart();
        break;

    case QCLOUD_EVENT_IOTHUB_RECEIVE_STATUS:
        ESP_LOGI("cloud", "receive status message: %s", (char *)event_data);
        break;

    default:
        ESP_LOGW("cloud", "Unhandled QCloud Event: %" PRIu32 "", event_id);
    }
}

static esp_err_t get_wifi_config(wifi_config_t *wifi_cfg, uint32_t wait_ms)
{
    ESP_QCLOUD_PARAM_CHECK(wifi_cfg);

    if (esp_qcloud_storage_get("wifi_config", wifi_cfg, sizeof(wifi_config_t)) == ESP_OK)
    {
        return ESP_OK;
    }

    /**< Reset wifi and restart wifi */
    esp_wifi_restore();
    esp_wifi_start();

    /**< Note: Smartconfig and softapconfig working at the same time will affect the configure network performance */
    char softap_ssid[32 + 1] = SOFTAPCONFIG_SSID;
    esp_qcloud_prov_softapconfig_start(SOFTAPCONFIG_TYPE_ESPRESSIF_TENCENT, softap_ssid, NULL);

    esp_qcloud_prov_print_wechat_qr();
    ESP_ERROR_CHECK(esp_qcloud_prov_wait(wifi_cfg, wait_ms));

    esp_qcloud_prov_softapconfig_stop();

    /**< Store the configure of the device */
    esp_qcloud_storage_set("wifi_config", wifi_cfg, sizeof(wifi_config_t));

    return ESP_OK;
}