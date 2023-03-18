#include <stdio.h>
#include <string.h>
#include "aiot_dm_api.h"

/**
 * @brief 上报属性powerstate到云端
 * @param value 属性powerstate的值
 * @param value 属性powerstate的值，数据类型uint8_t
 */

int32_t pal_post_property_powerstate (void *dm_handle, uint8_t value);
/**
 * @brief 上报属性RMSCurrent到云端
 * @param value 属性RMSCurrent的值
 * @param value 属性RMSCurrent的值，数据类型double
 */

int32_t pal_post_property_RMSCurrent (void *dm_handle, double value);
/**
 * @brief 上报属性RMSVoltage到云端
 * @param value 属性RMSVoltage的值
 * @param value 属性RMSVoltage的值，数据类型double
 */

int32_t pal_post_property_RMSVoltage (void *dm_handle, double value);
/**
 * @brief 上报属性EnvHumidity到云端
 * @param value 属性EnvHumidity的值
 * @param value 属性EnvHumidity的值，数据类型float
 */

int32_t pal_post_property_EnvHumidity (void *dm_handle, float value);
/**
 * @brief 上报属性EnvTemperature到云端
 * @param value 属性EnvTemperature的值
 * @param value 属性EnvTemperature的值，数据类型float
 */

int32_t pal_post_property_EnvTemperature (void *dm_handle, float value);
/**
 * @brief 上报属性sleepOnOff到云端
 * @param value 属性sleepOnOff的值
 * @param value 属性sleepOnOff的值，数据类型uint8_t
 */

int32_t pal_post_property_sleepOnOff (void *dm_handle, uint8_t value);
/**
 * @brief 上报属性RealTimePower到云端
 * @param value 属性RealTimePower的值
 * @param value 属性RealTimePower的值，数据类型double
 */

int32_t pal_post_property_RealTimePower (void *dm_handle, double value);
/**
 * @brief 上报属性ChipTemperture到云端
 * @param value 属性ChipTemperture的值
 * @param value 属性ChipTemperture的值，数据类型float
 */

int32_t pal_post_property_ChipTemperture (void *dm_handle, float value);
/**
 * @brief 上报属性timingFunction到云端
 * @param value 属性timingFunction的值
 * @param value 属性timingFunction的值，数据类型uint8_t
 */

int32_t pal_post_property_timingFunction (void *dm_handle, uint8_t value);
/**
 * @brief 上报属性LightIntensity到云端
 * @param value 属性LightIntensity的值
 * @param value 属性LightIntensity的值，数据类型int32_t
 */

int32_t pal_post_property_LightIntensity (void *dm_handle, int32_t value);
