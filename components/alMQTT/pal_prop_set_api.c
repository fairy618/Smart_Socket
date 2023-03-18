#include <stdio.h>
#include <string.h>
#include "aiot_dm_api.h"


/**
 * @brief 云端下发属性powerstate到设备
 * @param value 属性powerstate的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_powerstate_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性RMSCurrent到设备
 * @param value 属性RMSCurrent的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_RMSCurrent_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性RMSVoltage到设备
 * @param value 属性RMSVoltage的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_RMSVoltage_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性EnvHumidity到设备
 * @param value 属性EnvHumidity的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_EnvHumidity_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性DeviceType到设备
 * @param value 属性DeviceType的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_DeviceType_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性EnvTemperature到设备
 * @param value 属性EnvTemperature的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_EnvTemperature_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性sleepOnOff到设备
 * @param value 属性sleepOnOff的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_sleepOnOff_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性RealTimePower到设备
 * @param value 属性RealTimePower的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_RealTimePower_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
/**
 * @brief 云端下发属性ChipTemperture到设备
 * @param value 属性ChipTemperture的值
 * 注意: 不可以在这里调用耗时较长的阻塞函数
 */

void pal_ChipTemperture_property_set(char * params, int len) {
    /* TODO: 用户如果需要对该属性下发的消息做处理, 需要在此加入处理代码 */
    printf("pal property_set %.*s\n", len, params);
}
