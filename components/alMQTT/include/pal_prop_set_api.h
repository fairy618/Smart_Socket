#include <stdio.h>
#include <string.h>
#include "aiot_dm_api.h"
/**
 * @brief 上报属性powerstate到云端
 * @param value 属性powerstate的值
 */

void pal_powerstate_property_set(char * params, int len);
/**
 * @brief 上报属性RMSCurrent到云端
 * @param value 属性RMSCurrent的值
 */

void pal_RMSCurrent_property_set(char * params, int len);
/**
 * @brief 上报属性RMSVoltage到云端
 * @param value 属性RMSVoltage的值
 */

void pal_RMSVoltage_property_set(char * params, int len);
/**
 * @brief 上报属性EnvHumidity到云端
 * @param value 属性EnvHumidity的值
 */

void pal_EnvHumidity_property_set(char * params, int len);
/**
 * @brief 上报属性DeviceType到云端
 * @param value 属性DeviceType的值
 */

void pal_DeviceType_property_set(char * params, int len);
/**
 * @brief 上报属性EnvTemperature到云端
 * @param value 属性EnvTemperature的值
 */

void pal_EnvTemperature_property_set(char * params, int len);
/**
 * @brief 上报属性sleepOnOff到云端
 * @param value 属性sleepOnOff的值
 */

void pal_sleepOnOff_property_set(char * params, int len);
/**
 * @brief 上报属性RealTimePower到云端
 * @param value 属性RealTimePower的值
 */

void pal_RealTimePower_property_set(char * params, int len);
/**
 * @brief 上报属性ChipTemperture到云端
 * @param value 属性ChipTemperture的值
 */

void pal_ChipTemperture_property_set(char * params, int len);
