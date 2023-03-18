#include <stdio.h>
#include <string.h>
#include "aiot_dm_api.h"

#include "pal_prop_post_api.h"
/**
 * @brief 上报属性powerstate到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_powerstate(void *dm_handle, uint8_t value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[64] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"powerstate\": %d}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性RMSCurrent到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_RMSCurrent(void *dm_handle, double value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[384] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"RMSCurrent\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性RMSVoltage到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_RMSVoltage(void *dm_handle, double value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[384] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"RMSVoltage\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性EnvHumidity到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_EnvHumidity(void *dm_handle, float value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[128] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"EnvHumidity\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性EnvTemperature到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_EnvTemperature(void *dm_handle, float value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[128] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"EnvTemperature\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性sleepOnOff到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_sleepOnOff(void *dm_handle, uint8_t value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[64] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"sleepOnOff\": %d}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性RealTimePower到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_RealTimePower(void *dm_handle, double value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[384] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"RealTimePower\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性ChipTemperture到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_ChipTemperture(void *dm_handle, float value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[128] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"ChipTemperture\": %f}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性timingFunction到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_timingFunction(void *dm_handle, uint8_t value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[64] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"timingFunction\": %d}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

/**
 * @brief 上报属性LightIntensity到云端
 * @param dm_handle，dm句柄, 数据类型void *
 * @return 消息id:(>=1), 上报失败: <0
 */

int32_t pal_post_property_LightIntensity(void *dm_handle, int32_t value)
{
    aiot_dm_msg_t msg;
    int32_t res;
    /* TODO: 用户可以在此加入业务逻辑处理代码 */

    char property_payload[64] = {0};

    res = snprintf(property_payload, sizeof(property_payload), "{\"LightIntensity\": %ld}", value);
    if (res < 0) {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params =  property_payload;
    
    return aiot_dm_send(dm_handle, &msg);
}

