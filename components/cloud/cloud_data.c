#include "SHTC3.h"
#include "cloud_data.h"

float sensor_get_env_temp(void)
{
    UserData_t UserData;
    if (xQueueReceive(MailBox, &UserData, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE("qcloud", " --- sensor_get_env_temp falied --- ");
        return 0;
    }
    else
    {
        ESP_LOGI("qcloud", " --- sensor_get_env_temp done --- %.2f ", UserData.EnvData.EnvironmentTemperature);
        return UserData.EnvData.EnvironmentTemperature;
    }

    return -1;
}