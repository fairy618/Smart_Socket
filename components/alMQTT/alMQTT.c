#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "cJSON.h"
#include "alMQTT.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY_)
        {

            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
static void demo_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void demo_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID_,
            .password = WIFI_PASSWORD_},
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID_, WIFI_PASSWORD_);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID_, WIFI_PASSWORD_);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    // ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

/*
 * 这个例程适用于`Linux`这类支持pthread的POSIX设备, 它演示了用SDK配置MQTT参数并建立连接, 之后创建2个线程
 *
 * + 一个线程用于保活长连接
 * + 一个线程用于接收消息, 并在有消息到达时进入默认的数据回调, 在连接状态变化时进入事件回调
 *
 * 需要用户关注或修改的部分, 已经用 TODO 在注释中标明
 *
 */

/* TODO: 替换为自己设备的三元组 */
char *product_key = "a14U3nfkMWp";
char *device_name = "4JxiKytRv9OkNx1aB7yX";
char *device_secret = "49bd91481d65e73e81319bef3fbeb073";

/* 位于portfiles/aiot_port文件夹下的系统适配函数集合 */
extern aiot_sysdep_portfile_t g_aiot_sysdep_portfile;

/* 位于external/ali_ca_cert.c中的服务器证书 */
extern const char *ali_ca_cert;

static pthread_t g_mqtt_process_thread;
static pthread_t g_mqtt_recv_thread;
static uint8_t g_mqtt_process_thread_running = 0;
static uint8_t g_mqtt_recv_thread_running = 0;

/* TODO: 如果要关闭日志, 就把这个函数实现为空, 如果要减少日志, 可根据code选择不打印
 *
 * 例如: [1577589489.033][LK-0317] mqtt_basic_demo&a13FN5TplKq
 *
 * 上面这条日志的code就是0317(十六进制), code值的定义见core/aiot_state_api.h
 *
 */

/* 日志回调函数, SDK的日志会从这里输出 */
int32_t demo_state_logcb(int32_t code, char *message)
{
    // ESP_LOGD("LINK SDK", "%s", message);
    printf("%s", message);
    return 0;
}

/* MQTT事件回调函数, 当网络连接/重连/断开时被触发, 事件定义见core/aiot_mqtt_api.h */
void demo_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata)
{
    switch (event->type)
    {
    /* SDK因为用户调用了aiot_mqtt_connect()接口, 与mqtt服务器建立连接已成功 */
    case AIOT_MQTTEVT_CONNECT:
    {
        ESP_LOGI("LinkSDK", "AIOT_MQTTEVT_CONNECT. ");
        // printf("AIOT_MQTTEVT_CONNECT\n");
        /* TODO: 处理SDK建连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    /* SDK因为网络状况被动断连后, 自动发起重连已成功 */
    case AIOT_MQTTEVT_RECONNECT:
    {
        ESP_LOGI("LinkSDK", "AIOT_MQTTEVT_RECONNECT");
        // printf("AIOT_MQTTEVT_RECONNECT\n");
        /* TODO: 处理SDK重连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    /* SDK因为网络的状况而被动断开了连接, network是底层读写失败, heartbeat是没有按预期得到服务端心跳应答 */
    case AIOT_MQTTEVT_DISCONNECT:
    {
        char *cause = (event->data.disconnect == AIOT_MQTTDISCONNEVT_NETWORK_DISCONNECT) ? ("network disconnect") : ("heartbeat disconnect");
        ESP_LOGI("LinkSDK", "AIOT_MQTTEVT_DISCONNECT: %s. ", cause);
        // printf("AIOT_MQTTEVT_DISCONNECT: %s\n", cause);
        /* TODO: 处理SDK被动断连, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    default:
    {
    }
    }
}

/* MQTT默认消息处理回调, 当SDK从服务器收到MQTT消息时, 且无对应用户回调处理时被调用 */
void demo_mqtt_default_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata)
{
    switch (packet->type)
    {
    case AIOT_MQTTRECV_HEARTBEAT_RESPONSE:
    {
        ESP_LOGI("LinkSDK", "heartbeat response");
        // printf("heartbeat response\n");
        /* TODO: 处理服务器对心跳的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_SUB_ACK:
    {
        ESP_LOGI("LinkSDK", "suback, res: -0x%04lX, packet id: %d, max qos: %d", -packet->data.sub_ack.res, packet->data.sub_ack.packet_id, packet->data.sub_ack.max_qos);
        // printf("suback, res: -0x%04lX, packet id: %d, max qos: %d\n", -packet->data.sub_ack.res, packet->data.sub_ack.packet_id, packet->data.sub_ack.max_qos);
        /* TODO: 处理服务器对订阅请求的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_PUB:
    {
        ESP_LOGI("LinkSDK", "pub, qos: %d, topic: %.*s", packet->data.pub.qos, packet->data.pub.topic_len, packet->data.pub.topic);
        printf("\n\n*******************************************\npub, payload: %.*s\n", (int)(packet->data.pub.payload_len), packet->data.pub.payload);
        // printf("pub, payload: %.*s\n",  packet->data.pub.payload);
        /* TODO: 处理服务器下发的业务报文 */

        printf("\n\n###########################################\n%s\n\n", packet->data.pub.topic);

        //  printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        //         printf("DATA=%.*s\r\n", event->data_len, event->data);
        //         if (memcmp("/sys/a1gFEcNBZwC/aaaa/thing/service/property/set", event->topic, event->topic_len) == 0)
        //         {
        //             cJSON *json = cJSON_ParseWithLength(event->data, event->data_len);
        //             const cJSON *method;
        //             method = cJSON_GetObjectItem(json, "method");
        //             if (cJSON_IsString(method) && (method->valuestring != NULL))
        //             {
        //                 if (strcmp("thing.service.property.set", method->valuestring) == 0)
        //                 {
        //                     cJSON *params = cJSON_GetObjectItem(json, "params");
        //                     cJSON *result = cJSON_CreateObject();
        //                     //                        cJSON_AddItemToObject(result, "params", params);
        //                     cJSON_AddItemReferenceToObject(result, "params", params);
        //                     cJSON *powerstate = cJSON_GetObjectItem(params, "powerstate");
        //                     if (cJSON_IsNumber(powerstate))
        //                     {
        //                         if (powerstate->valueint == 0)
        //                         {
        //                             led_off();
        //                         }
        //                         else
        //                         {
        //                             led_on();
        //                         }
        //                     }

        //                     char *result_string = cJSON_PrintUnformatted(result);

        //                     cJSON_Delete(result);
        //                     esp_mqtt_client_publish(client, "/sys/a1gFEcNBZwC/aaaa/thing/event/property/post",
        //                                             result_string,
        //                                             strlen(result_string),
        //                                             0, 0);
        //                     printf("%s\n", result_string);
        //                     cJSON_free(result_string);
        //                 }
        //             }
        //             cJSON_Delete(json);
        //         }
    }
    break;

    case AIOT_MQTTRECV_PUB_ACK:
    {
        ESP_LOGI("LinkSDK", "puback, packet id: %d. ", packet->data.pub_ack.packet_id);
        // printf("puback, packet id: %d\n", packet->data.pub_ack.packet_id);
        /* TODO: 处理服务器对QoS1上报消息的回应, 一般不处理 */
    }
    break;

    default:
    {
    }
    }
}

/* 执行aiot_mqtt_process的线程, 包含心跳发送和QoS1消息重发 */
void *demo_mqtt_process_thread(void *args)
{
    int32_t res = STATE_SUCCESS;

    while (g_mqtt_process_thread_running)
    {
        res = aiot_mqtt_process(args);
        if (res == STATE_USER_INPUT_EXEC_DISABLED)
        {
            break;
        }
        sleep(1);
    }
    return NULL;
}

/* 执行aiot_mqtt_recv的线程, 包含网络自动重连和从服务器收取MQTT消息 */
void *demo_mqtt_recv_thread(void *args)
{
    int32_t res = STATE_SUCCESS;

    while (g_mqtt_recv_thread_running)
    {
        res = aiot_mqtt_recv(args);
        if (res < STATE_SUCCESS)
        {
            if (res == STATE_USER_INPUT_EXEC_DISABLED)
            {
                break;
            }
            sleep(1);
        }
    }
    return NULL;
}

void Task_ali_mqqt(void *pvParameters)
{
    int32_t res = STATE_SUCCESS;
    void *mqtt_handle = NULL;
    char *url = "iot-as-mqtt.cn-shanghai.aliyuncs.com"; /* 阿里云平台上海站点的域名后缀 */
    char host[100] = {0};                               /* 用这个数组拼接设备连接的云平台站点全地址, 规则是 ${productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com */
    uint16_t port = 443;                                /* 无论设备是否使用TLS连接阿里云平台, 目的端口都是443 */
    uint8_t post_reply = 1;
    aiot_sysdep_network_cred_t cred; /* 安全凭据结构体, 如果要用TLS, 这个结构体中配置CA证书等参数 */

    /* 配置SDK的底层依赖 */
    aiot_sysdep_set_portfile(&g_aiot_sysdep_portfile);
    /* 配置SDK的日志输出 */
    aiot_state_set_logcb(demo_state_logcb);

    /* 创建SDK的安全凭据, 用于建立TLS连接 */
    memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
    cred.option = AIOT_SYSDEP_NETWORK_CRED_SVRCERT_CA; /* 使用RSA证书校验MQTT服务端 */
    cred.max_tls_fragment = 16384;                     /* 最大的分片长度为16K, 其它可选值还有4K, 2K, 1K, 0.5K */
    cred.sni_enabled = 1;                              /* TLS建连时, 支持Server Name Indicator */
    cred.x509_server_cert = ali_ca_cert;               /* 用来验证MQTT服务端的RSA根证书 */
    cred.x509_server_cert_len = strlen(ali_ca_cert);   /* 用来验证MQTT服务端的RSA根证书长度 */

    /* 创建1个MQTT客户端实例并内部初始化默认参数 */
    mqtt_handle = aiot_mqtt_init();
    if (mqtt_handle == NULL)
    {
        ESP_LOGE("LinkSDK", "aiot_mqtt_init failed");
        // printf("aiot_mqtt_init failed\n");
        // return -1;
    }

    /* TODO: 如果以下代码不被注释, 则例程会用TCP而不是TLS连接云平台 */
    /*
    {
        memset(&cred, 0, sizeof(aiot_sysdep_network_cred_t));
        cred.option = AIOT_SYSDEP_NETWORK_CRED_NONE;
    }
    */

    snprintf(host, 100, "%s.%s", product_key, url);
    /* 配置MQTT服务器地址 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_HOST, (void *)host);
    /* 配置MQTT服务器端口 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PORT, (void *)&port);
    /* 配置设备productKey */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_PRODUCT_KEY, (void *)product_key);
    /* 配置设备deviceName */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_NAME, (void *)device_name);
    /* 配置设备deviceSecret */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_DEVICE_SECRET, (void *)device_secret);
    /* 配置网络连接的安全凭据, 上面已经创建好了 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_NETWORK_CRED, (void *)&cred);
    /* 配置MQTT默认消息接收回调函数 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_RECV_HANDLER, (void *)demo_mqtt_default_recv_handler);
    /* 配置MQTT事件回调函数 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_EVENT_HANDLER, (void *)demo_mqtt_event_handler);

    void *dm_handle = NULL;
    {

        /* 创建DATA-MODEL实例 */
        dm_handle = aiot_dm_init();
        if (dm_handle == NULL)
        {
            ESP_LOGE("ALI MQTT", "aiot_dm_init failed");
            // return -1;
        }
        /* 配置MQTT实例句柄 */
        aiot_dm_setopt(dm_handle, AIOT_DMOPT_MQTT_HANDLE, mqtt_handle);
        /* 配置消息接收处理回调函数 */
        aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *)demo_dm_recv_handler);

        /* 配置是云端否需要回复post_reply给设备. 如果为1, 表示需要云端回复, 否则表示不回复 */
        aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *)&post_reply);
    }

    /* 与服务器建立MQTT连接 */
    res = aiot_mqtt_connect(mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        /* 尝试建立连接失败, 销毁MQTT实例, 回收资源 */
        aiot_mqtt_deinit(&mqtt_handle);
        ESP_LOGE("LinkSDK", "aiot_mqtt_connect failed: -0x%04lX. ", -res);
        // return -1;
    }

    /* MQTT 订阅topic功能示例, 请根据自己的业务需求进行使用 */
    {
        // char *sub_topic = "/sys/a13FN5TplKq/mqtt_basic_demo/thing/event/+/post_reply";
        char *sub_topic = "/ext/ntp/a14U3nfkMWp/4JxiKytRv9OkNx1aB7yX/response";

        aiot_mqtt_sub(mqtt_handle, "/sys/${YourProductKey}/${YourDeviceName}/thing/event/property/batch/post_reply", NULL, 1,
                      NULL);

        res = aiot_mqtt_sub(mqtt_handle, sub_topic, NULL, 1, NULL);
        if (res < 0)
        {
            ESP_LOGE("LinkSDK", "aiot_mqtt_sub failed, res: -0x%04lX. ", -res);
            // return -1;
        }
    }

    /* MQTT 发布消息功能示例, 请根据自己的业务需求进行使用 */
    {
        char *pub_topic = "/sys/a13FN5TplKq/mqtt_basic_demo/thing/event/property/post";
        char *pub_payload = "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"LightSwitch\":0}}";

        res = aiot_mqtt_pub(mqtt_handle, pub_topic, (uint8_t *)pub_payload, strlen(pub_payload), 0);
        if (res < 0)
        {
            ESP_LOGE("LinkSDK", "aiot_mqtt_sub failed, res: -0x%04lX. ", -res);
            // printf("aiot_mqtt_sub failed, res: -0x%04lX\n", -res);
            // return -1;
        }
    }

    /* 创建一个单独的线程, 专用于执行aiot_mqtt_process, 它会自动发送心跳保活, 以及重发QoS1的未应答报文 */
    g_mqtt_process_thread_running = 1;
    res = pthread_create(&g_mqtt_process_thread, NULL, demo_mqtt_process_thread, mqtt_handle);
    if (res < 0)
    {
        ESP_LOGE("LinkSDK", "pthread_create demo_mqtt_process_thread failed: %ld. ", res);
        // printf("pthread_create demo_mqtt_process_thread failed: %ld\n", res);
        // return -1;
    }

    /* 创建一个单独的线程用于执行aiot_mqtt_recv, 它会循环收取服务器下发的MQTT消息, 并在断线时自动重连 */
    g_mqtt_recv_thread_running = 1;
    res = pthread_create(&g_mqtt_recv_thread, NULL, demo_mqtt_recv_thread, mqtt_handle);
    if (res < 0)
    {
        ESP_LOGE("LinkSDK", "pthread_create demo_mqtt_recv_thread failed: %ld. ", res);
        // printf("pthread_create demo_mqtt_recv_thread failed: %ld\n", res);
        // return -1;
    }

    /* 主循环进入休眠 */
    while (1)
    {
        // demo_send_property_post(dm_handle, "{\"LightSwitch\": 0}");
        pal_post_property_EnvTemperature(dm_handle, 66.66);
        // sleep(1);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    /* 断开MQTT连接, 一般不会运行到这里 */
    res = aiot_mqtt_disconnect(mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        aiot_mqtt_deinit(&mqtt_handle);
        aiot_mqtt_deinit(&mqtt_handle);
        ESP_LOGE("LinkSDK", "aiot_mqtt_disconnect failed: -0x%04lX", -res);
        // printf("aiot_mqtt_disconnect failed: -0x%04lX\n", -res);
        // return -1;
    }

    /* 销毁MQTT实例, 一般不会运行到这里 */
    res = aiot_mqtt_deinit(&mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        ESP_LOGE("LinkSDK", "aiot_mqtt_deinit failed: -0x%04lX. ", -res);
        // printf("aiot_mqtt_deinit failed: -0x%04lX\n", -res);
        // return -1;
    }

    g_mqtt_process_thread_running = 0;
    g_mqtt_recv_thread_running = 0;
    pthread_join(g_mqtt_process_thread, NULL);
    pthread_join(g_mqtt_recv_thread, NULL);

    // return 0;
}

void WifiConnect(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // /* start linkkit mqtt */
    // ESP_LOGI(TAG, "Start linkkit mqtt");
    // linkkit_main();
}

/* 属性上报函数演示 */
int32_t demo_send_property_post(void *dm_handle, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 用户数据接收处理回调函数 */
static void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_handler, type = %d\r\n", recv->type);

    switch (recv->type)
    {

    /* 属性上报, 事件上报, 获取期望属性值或者删除期望属性值的应答 */
    case AIOT_DMRECV_GENERIC_REPLY:
    {
        // demo_dm_recv_generic_reply(dm_handle, recv, userdata);
    }
    break;

    /* 属性设置 */
    case AIOT_DMRECV_PROPERTY_SET:
    {
        // demo_dm_recv_property_set(dm_handle, recv, userdata);
    }
    break;

    /* 异步服务调用 */
    case AIOT_DMRECV_ASYNC_SERVICE_INVOKE:
    {
        // demo_dm_recv_async_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 同步服务调用 */
    case AIOT_DMRECV_SYNC_SERVICE_INVOKE:
    {
        // demo_dm_recv_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 下行二进制数据 */
    case AIOT_DMRECV_RAW_DATA:
    {
        // demo_dm_recv_raw_data(dm_handle, recv, userdata);
    }
    break;

    /* 二进制格式的同步服务调用, 比单纯的二进制数据消息多了个rrpc_id */
    case AIOT_DMRECV_RAW_SYNC_SERVICE_INVOKE:
    {
        // demo_dm_recv_raw_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 上行二进制数据后, 云端的回复报文 */
    case AIOT_DMRECV_RAW_DATA_REPLY:
    {
        // demo_dm_recv_raw_data_reply(dm_handle, recv, userdata);
    }
    break;

    default:
        break;
    }
}

int32_t demo_send_property_batch_post(void *dm_handle, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_BATCH_POST;
    msg.data.property_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 事件上报函数演示 */
int32_t demo_send_event_post(void *dm_handle, char *event_id, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_EVENT_POST;
    msg.data.event_post.event_id = event_id;
    msg.data.event_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 演示了获取属性LightSwitch的期望值, 用户可将此函数加入到main函数中运行演示 */
int32_t demo_send_get_desred_requset(void *dm_handle)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_GET_DESIRED;
    msg.data.get_desired.params = "[\"LightSwitch\"]";

    return aiot_dm_send(dm_handle, &msg);
}

/* 演示了删除属性LightSwitch的期望值, 用户可将此函数加入到main函数中运行演示 */
int32_t demo_send_delete_desred_requset(void *dm_handle)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_DELETE_DESIRED;
    msg.data.get_desired.params = "{\"LightSwitch\":{}}";

    return aiot_dm_send(dm_handle, &msg);
}

static void demo_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_data_reply receive reply for up_raw msg, data len = %ld\r\n", recv->data.raw_data.data_len);
    /* TODO: 用户处理下行的二进制数据, 位于recv->data.raw_data.data中 */
}

static void demo_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_sync_service_invoke raw sync service rrpc_id = %s, data_len = %ld\r\n",
           recv->data.raw_service_invoke.rrpc_id,
           recv->data.raw_service_invoke.data_len);
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
    if (res < 0)
    {
        return -1;
    }

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_POST;
    msg.data.property_post.params = property_payload;

    return aiot_dm_send(dm_handle, &msg);
}
