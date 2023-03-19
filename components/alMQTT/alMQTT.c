#include "lwip/err.h"

#include "alMQTT.h"

#include "pal_prop_post_api.h"
#include "pal_prop_set_api.h"

/*
 * @description: wifi connect
 */
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int s_retry_num = 0;

/*
 * @description: ali mqtt service
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

/*
 * @description: user
 */
bool ReceMqttFlag = 0;
alMQTT_data_t alMQTT_data;
bool RgbRecFlag = 0;
rgb_data_t RgbRecData;

/*
 * @description: static function
 */
static void al_dm_recv_property_set(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void al_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void al_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void demo_dm_recv_handler(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void demo_dm_recv_async_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void demo_dm_recv_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);
static void demo_dm_recv_raw_data(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata);

/*
 * @description: wifi connect callback function
 * @param {void} *arg
 * @param {esp_event_base_t} event_base
 * @param {int32_t} event_id
 * @param {void} *event_data
 * @return {*}
 */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
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

/*
 * @description: wifi initialization, station model
 * @return {*}
 */
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

void Task_ali_mqqt(void *pvParameters)
{
    QueueSetHandle_t xQueueSet = ((QueueSetHandle_t)pvParameters);
    QueueSetMemberHandle_t xActivatedMember;

    int32_t res = STATE_SUCCESS;
    void *mqtt_handle = NULL;
    char *url = "iot-as-mqtt.cn-shanghai.aliyuncs.com"; /* 阿里云平台上海站点的域名后缀 */
    char host[100] = {0};                               /* 用这个数组拼接设备连接的云平台站点全地址, 规则是 ${productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com */
    uint16_t port = 443;                                /* 无论设备是否使用TLS连接阿里云平台, 目的端口都是443 */
    uint8_t post_reply = 1;
    aiot_sysdep_network_cred_t cred; /* 安全凭据结构体, 如果要用TLS, 这个结构体中配置CA证书等参数 */

    ElectricalParameter_t ElectricalParameter;
    Sensor_data_t SensorData;

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
        ESP_LOGE("Task ali MQTT", "aiot_mqtt_init failed");
    }

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
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_RECV_HANDLER, (void *)al_mqtt_recv_handler);
    /* 配置MQTT事件回调函数 */
    aiot_mqtt_setopt(mqtt_handle, AIOT_MQTTOPT_EVENT_HANDLER, (void *)al_mqtt_event_handler);

    void *dm_handle = NULL;
    /* 创建DATA-MODEL实例 */
    dm_handle = aiot_dm_init();
    if (dm_handle == NULL)
    {
        ESP_LOGE("Task ali MQTT", "aiot_dm_init failed");
    }
    /* 配置MQTT实例句柄 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_MQTT_HANDLE, mqtt_handle);
    /* 配置消息接收处理回调函数 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_RECV_HANDLER, (void *)demo_dm_recv_handler);
    /* 配置是云端否需要回复post_reply给设备. 如果为1, 表示需要云端回复, 否则表示不回复 */
    aiot_dm_setopt(dm_handle, AIOT_DMOPT_POST_REPLY, (void *)&post_reply);

    /* 与服务器建立MQTT连接 */
    res = aiot_mqtt_connect(mqtt_handle);
    if (res < STATE_SUCCESS)
    {
        /* 尝试建立连接失败, 销毁MQTT实例, 回收资源 */
        aiot_mqtt_deinit(&mqtt_handle);
        ESP_LOGE("Task ali MQTT", "aiot_mqtt_connect failed: -0x%04lX. ", -res);
    }

    /* MQTT 订阅topic功能示例, 请根据自己的业务需求进行使用 */
    // {
    char *sub_topic = "/ext/ntp/a14U3nfkMWp/4JxiKytRv9OkNx1aB7yX/response";

    aiot_mqtt_sub(mqtt_handle, "/sys/${YourProductKey}/${YourDeviceName}/thing/event/property/batch/post_reply", NULL, 1, NULL);

    res = aiot_mqtt_sub(mqtt_handle, sub_topic, NULL, 1, NULL);
    if (res < 0)
    {
        ESP_LOGE("Task ali MQTT", "aiot_mqtt_sub failed, res: -0x%04lX. ", -res);
    }
    // }

    /* MQTT 发布消息功能示例, 请根据自己的业务需求进行使用 */
    // {
    char *pub_topic = "/sys/a13FN5TplKq/mqtt_basic_demo/thing/event/property/post";
    char *pub_payload = "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"LightSwitch\":0}}";

    res = aiot_mqtt_pub(mqtt_handle, pub_topic, (uint8_t *)pub_payload, strlen(pub_payload), 0);
    if (res < 0)
    {
        ESP_LOGE("LinkSDK", "aiot_mqtt_sub failed, res: -0x%04lX. ", -res);
    }
    // }

    /* 创建一个单独的线程, 专用于执行aiot_mqtt_process, 它会自动发送心跳保活, 以及重发QoS1的未应答报文 */
    g_mqtt_process_thread_running = 1;
    res = pthread_create(&g_mqtt_process_thread, NULL, demo_mqtt_process_thread, mqtt_handle);
    if (res < 0)
    {
        ESP_LOGE("LinkSDK", "pthread_create demo_mqtt_process_thread failed: %ld. ", res);
    }

    /* 创建一个单独的线程用于执行aiot_mqtt_recv, 它会循环收取服务器下发的MQTT消息, 并在断线时自动重连 */
    g_mqtt_recv_thread_running = 1;
    res = pthread_create(&g_mqtt_recv_thread, NULL, demo_mqtt_recv_thread, mqtt_handle);
    if (res < 0)
    {
        ESP_LOGE("LinkSDK", "pthread_create demo_mqtt_recv_thread failed: %ld. ", res);
    }

    /* 主循环进入休眠 */
    while (1)
    {
        xActivatedMember = xQueueSelectFromSet(xQueueSet, pdMS_TO_TICKS(200));

        if (xActivatedMember == xQueueSensor)
        {
            if (xQueueReceive(xActivatedMember, &SensorData, 0) == pdPASS)
            {
                ESP_LOGI("TASK AL MQTT", "Rec Data: EnvTemperature: %.2f(℃), EnvHumidity: %.2f(%%), ChipTemperature: %.2f(℃), LightIntensity: %d(lm). ", SensorData.EnvironmentTemperature, SensorData.EnvHumidity, SensorData.ChipTemperature, SensorData.LightIntensity);
                pal_post_property_EnvTemperature(dm_handle, SensorData.EnvironmentTemperature);
                pal_post_property_EnvHumidity(dm_handle, SensorData.EnvHumidity);
                pal_post_property_ChipTemperture(dm_handle, SensorData.ChipTemperature);
                pal_post_property_LightIntensity(dm_handle, SensorData.LightIntensity);
            }
            else
            {
                ESP_LOGE("TASK AL MQTT", "Send SensorData failed. ");
            }
        }
        else if (xActivatedMember == xQueueElectric)
        {
            if (xQueueReceive(xActivatedMember, &ElectricalParameter, 0) == pdPASS)
            {
                ESP_LOGI("TASK AL MQTT", "Rec Data: VoltageRMS: %.2f (V). ", ElectricalParameter.VoltageRMS);
                ESP_LOGI("TASK AL MQTT", "Rec Data: CurrentRMS: %.2f (A). ", ElectricalParameter.CurrentRMS);
                ESP_LOGI("TASK AL MQTT", "Rec Data: ActivePower: %.2f (W). ", ElectricalParameter.ActivePower);
                ESP_LOGI("TASK AL MQTT", "Rec Data: ApparentPower: %.2f (W). ", ElectricalParameter.ApparentPower);
                ESP_LOGI("TASK AL MQTT", "Rec Data: PowerFactor: %.2f. ", ElectricalParameter.PowerFactor);
                ESP_LOGI("TASK AL MQTT", "Rec Data: PF_value: %lld. ", ElectricalParameter.PF_value);
                ESP_LOGI("TASK AL MQTT", "Rec Data: ElectricityConsumption: %.2f (kWh). ", ElectricalParameter.ElectricityConsumption);

                pal_post_property_RMSCurrent(dm_handle, ElectricalParameter.CurrentRMS);
                pal_post_property_RMSVoltage(dm_handle, ElectricalParameter.VoltageRMS);
            }
            else
            {
                ESP_LOGE("TASK AL MQTT", "Send ElectricalParameter failed. ");
            }
        }

        if (RgbRecFlag)
        {
            RgbRecFlag = 0;

            if (xQueueSend(xQueueRgb, (void *)&RgbRecData, 0) == pdPASS)
            {
                ESP_LOGI("ALMQTT", " --- Send RgbRecData to xQueue done! --- ");
            }
            else
            {
                ESP_LOGE("ALMQTT", " --- Send RgbRecData to xQueue failed! --- ");
            }
        }

        // if (ReceMqttFlag == 1)
        // {
        //     ReceMqttFlag = 0;
        // }

        // if (uxQueueMessagesWaiting(xQueueSensor) != 0) // Returns the number of items that are currently held in a queue
        // {

        // }

        // pal_post_property_EnvTemperature(dm_handle, 12.34);

        vTaskDelay(10 / portTICK_PERIOD_MS);
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
}

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
void al_mqtt_event_handler(void *handle, const aiot_mqtt_event_t *event, void *userdata)
{
    switch (event->type)
    {
    /* SDK因为用户调用了aiot_mqtt_connect()接口, 与mqtt服务器建立连接已成功 */
    case AIOT_MQTTEVT_CONNECT:
    {
        ESP_LOGI("al_mqtt_event_handler", "AIOT_MQTTEVT_CONNECT. ");
        // printf("AIOT_MQTTEVT_CONNECT\n");
        /* TODO: 处理SDK建连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    /* SDK因为网络状况被动断连后, 自动发起重连已成功 */
    case AIOT_MQTTEVT_RECONNECT:
    {
        ESP_LOGI("al_mqtt_event_handler", "AIOT_MQTTEVT_RECONNECT");
        // printf("AIOT_MQTTEVT_RECONNECT\n");
        /* TODO: 处理SDK重连成功, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    /* SDK因为网络的状况而被动断开了连接, network是底层读写失败, heartbeat是没有按预期得到服务端心跳应答 */
    case AIOT_MQTTEVT_DISCONNECT:
    {
        char *cause = (event->data.disconnect == AIOT_MQTTDISCONNEVT_NETWORK_DISCONNECT) ? ("network disconnect") : ("heartbeat disconnect");
        ESP_LOGI("al_mqtt_event_handler", "AIOT_MQTTEVT_DISCONNECT: %s. ", cause);
        // printf("AIOT_MQTTEVT_DISCONNECT: %s\n", cause);
        /* TODO: 处理SDK被动断连, 不可以在这里调用耗时较长的阻塞函数 */
    }
    break;

    default:
    {
        ESP_LOGE("al_mqtt_event_handler", "Unkown enent");
    }
    }
}

/* MQTT默认消息处理回调, 当SDK从服务器收到MQTT消息时, 且无对应用户回调处理时被调用 */
void al_mqtt_recv_handler(void *handle, const aiot_mqtt_recv_t *packet, void *userdata)
{
    switch (packet->type)
    {
    case AIOT_MQTTRECV_HEARTBEAT_RESPONSE:
    {
        ESP_LOGI("al_mqtt_recv_handler", "heartbeat response");
        /* TODO: 处理服务器对心跳的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_SUB_ACK:
    {
        ESP_LOGI("al_mqtt_recv_handler", "suback, res: -0x%04lX, packet id: %d, max qos: %d", -packet->data.sub_ack.res, packet->data.sub_ack.packet_id, packet->data.sub_ack.max_qos);
        /* TODO: 处理服务器对订阅请求的回应, 一般不处理 */
    }
    break;

    case AIOT_MQTTRECV_PUB:
    {
        ESP_LOGI("al_mqtt_recv_handler", "pub, qos: %d, topic: %.*s", packet->data.pub.qos, packet->data.pub.topic_len, packet->data.pub.topic);
        /* TODO: 处理服务器下发的业务报文 */
    }
    break;

    case AIOT_MQTTRECV_PUB_ACK:
    {
        ESP_LOGI("al_mqtt_recv_handler", "puback, packet id: %d. ", packet->data.pub_ack.packet_id);
        /* TODO: 处理服务器对QoS1上报消息的回应, 一般不处理 */
    }
    break;

    default:
    {
        ESP_LOGE("al_mqtt_recv_handler", "Unknown recv data");
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

/*
 * @description: connect to wifi, think about if call "nvs_flash_init()"
 * @return {*}
 */
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
        al_dm_recv_property_set(dm_handle, recv, userdata);
    }
    break;

    /* 异步服务调用 */
    case AIOT_DMRECV_ASYNC_SERVICE_INVOKE:
    {
        demo_dm_recv_async_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 同步服务调用 */
    case AIOT_DMRECV_SYNC_SERVICE_INVOKE:
    {
        demo_dm_recv_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 下行二进制数据 */
    case AIOT_DMRECV_RAW_DATA:
    {
        demo_dm_recv_raw_data(dm_handle, recv, userdata);
    }
    break;

    /* 二进制格式的同步服务调用, 比单纯的二进制数据消息多了个rrpc_id */
    case AIOT_DMRECV_RAW_SYNC_SERVICE_INVOKE:
    {
        al_dm_recv_raw_sync_service_invoke(dm_handle, recv, userdata);
    }
    break;

    /* 上行二进制数据后, 云端的回复报文 */
    case AIOT_DMRECV_RAW_DATA_REPLY:
    {
        al_dm_recv_raw_data_reply(dm_handle, recv, userdata);
    }
    break;

    default:
        break;
    }
}

int32_t al_send_property_batch_post(void *dm_handle, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_PROPERTY_BATCH_POST;
    msg.data.property_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 事件上报函数演示 */
int32_t al_send_event_post(void *dm_handle, char *event_id, char *params)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_EVENT_POST;
    msg.data.event_post.event_id = event_id;
    msg.data.event_post.params = params;

    return aiot_dm_send(dm_handle, &msg);
}

/* 演示了获取属性LightSwitch的期望值, 用户可将此函数加入到main函数中运行演示 */
int32_t al_send_get_desred_requset(void *dm_handle)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_GET_DESIRED;
    msg.data.get_desired.params = "[\"LightSwitch\"]";

    return aiot_dm_send(dm_handle, &msg);
}

/* 演示了删除属性LightSwitch的期望值, 用户可将此函数加入到main函数中运行演示 */
int32_t al_send_delete_desred_requset(void *dm_handle)
{
    aiot_dm_msg_t msg;

    memset(&msg, 0, sizeof(aiot_dm_msg_t));
    msg.type = AIOT_DMMSG_DELETE_DESIRED;
    msg.data.get_desired.params = "{\"LightSwitch\":{}}";

    return aiot_dm_send(dm_handle, &msg);
}

/* 上行二进制数据后, 云端的回复报文 */
static void al_dm_recv_raw_data_reply(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("al_dm_recv_raw_data_reply receive reply for up_raw msg, data len = %ld\r\n", recv->data.raw_data.data_len);
    /* TODO: 用户处理下行的二进制数据, 位于recv->data.raw_data.data中 */
}

/* 二进制格式的同步服务调用, 比单纯的二进制数据消息多了个rrpc_id */
static void al_dm_recv_raw_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("al_dm_recv_raw_sync_service_invoke raw sync service rrpc_id = %s, data_len = %ld\r\n", recv->data.raw_service_invoke.rrpc_id, recv->data.raw_service_invoke.data_len);
}

/* 属性设置 */
static void al_dm_recv_property_set(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    ESP_LOGI("al_dm_recv_property_set", " msg_id = %ld, params = )%.*s(", (unsigned long)recv->data.property_set.msg_id, (int)(recv->data.property_set.params_len), recv->data.property_set.params);

    cJSON *json = cJSON_ParseWithLength(recv->data.property_set.params, (int)(recv->data.property_set.params_len));
    cJSON *RGBColor = cJSON_GetObjectItem(json, "RGBColor");
    if (cJSON_IsObject(RGBColor))
    {
        ESP_LOGI("cJSON TEST", "cJSON_IsObject");

        cJSON *Red = cJSON_GetObjectItem(RGBColor, "Red");
        cJSON *Blue = cJSON_GetObjectItem(RGBColor, "Blue");
        cJSON *Green = cJSON_GetObjectItem(RGBColor, "Green");

        if (cJSON_IsNumber(Red) && cJSON_IsNumber(Blue) && cJSON_IsNumber(Green))
        {
            RgbRecFlag = 1;
            RgbRecData.red = Red->valueint;
            RgbRecData.green = Green->valueint;
            RgbRecData.blue = Blue->valueint;

            ESP_LOGI("cJSON TEST", "RGB: %d-%d-%d", (uint8_t)RgbRecData.red, (uint8_t)RgbRecData.green, (uint8_t)RgbRecData.blue);
        }
    }

    cJSON *powerstate = cJSON_GetObjectItem(json, "powerstate");
    if (cJSON_IsNumber(powerstate))
    {
        alMQTT_data.powerstateFlag = 0;
        if (powerstate->valueint == 0)
        {
            alMQTT_data.powerstate = 0;
            ESP_LOGI("cJSON TEST", "powerstate = 0");
        }
        else
        {
            alMQTT_data.powerstate = 1;
            ESP_LOGI("cJSON TEST", "powerstate = 1");
        }
    }

    cJSON *Timer_Quantum = cJSON_GetObjectItem(json, "Timer_Quantum");
    if (cJSON_IsString(Timer_Quantum))
    {
        alMQTT_data.Timer_QuantumFlag = 0;
        alMQTT_data.Timer_Quantum = Timer_Quantum->valuestring;
        ESP_LOGI("cJSON TEST", "Timer_Quantum = %s", Timer_Quantum->valuestring);
    }

    cJSON *sleepOnOff = cJSON_GetObjectItem(json, "sleepOnOff");
    if (cJSON_IsNumber(sleepOnOff))
    {
        alMQTT_data.sleepOnOffFlag = 0;
        if (sleepOnOff->valueint == 0)
        {
            alMQTT_data.sleepOnOff = 0;
            ESP_LOGI("cJSON TEST", "sleepOnOff = 0");
        }
        else
        {
            alMQTT_data.sleepOnOff = 1;
            ESP_LOGI("cJSON TEST", "sleepOnOff = 1");
        }
    }

    cJSON *timingFunction = cJSON_GetObjectItem(json, "timingFunction");
    if (cJSON_IsNumber(timingFunction))
    {
        alMQTT_data.timingFunctionFlag = 0;
        if (timingFunction->valueint == 0)
        {
            alMQTT_data.timingFunction = 0;
            ESP_LOGI("cJSON TEST", "timingFunction = 0");
        }
        else
        {
            alMQTT_data.timingFunction = 1;
            ESP_LOGI("cJSON TEST", "timingFunction = 1");
        }
    }
    cJSON_Delete(json);
    ReceMqttFlag = 1;

    /* TODO: 以下代码演示如何对来自云平台的属性设置指令进行应答, 用户可取消注释查看演示效果 */
    // /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_PROPERTY_SET_REPLY;
        msg.data.property_set_reply.msg_id = recv->data.property_set.msg_id;
        msg.data.property_set_reply.code = 200;
        msg.data.property_set_reply.data = "{}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0)
        {
            printf("aiot_dm_send failed\r\n");
        }
    }
    // */
}

static void demo_dm_recv_async_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_async_service_invoke msg_id = %ld, service_id = %s, params = %.*s\r\n",
           (unsigned long)recv->data.async_service_invoke.msg_id,
           recv->data.async_service_invoke.service_id,
           (int)(recv->data.async_service_invoke.params_len),
           recv->data.async_service_invoke.params);

    /* TODO: 以下代码演示如何对来自云平台的异步服务调用进行应答, 用户可取消注释查看演示效果
     *
     * 注意: 如果用户在回调函数外进行应答, 需要自行保存msg_id, 因为回调函数入参在退出回调函数后将被SDK销毁, 不可以再访问到
     */

    /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_ASYNC_SERVICE_REPLY;
        msg.data.async_service_reply.msg_id = recv->data.async_service_invoke.msg_id;
        msg.data.async_service_reply.code = 200;
        msg.data.async_service_reply.service_id = "ToggleLightSwitch";
        msg.data.async_service_reply.data = "{\"dataA\": 20}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0) {
            printf("aiot_dm_send failed\r\n");
        }
    }
    */
}

static void demo_dm_recv_sync_service_invoke(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_sync_service_invoke msg_id = %ld, rrpc_id = %s, service_id = %s, params = %.*s\r\n",
           (unsigned long)recv->data.sync_service_invoke.msg_id,
           recv->data.sync_service_invoke.rrpc_id,
           recv->data.sync_service_invoke.service_id,
           (int)(recv->data.sync_service_invoke.params_len),
           recv->data.sync_service_invoke.params);

    /* TODO: 以下代码演示如何对来自云平台的同步服务调用进行应答, 用户可取消注释查看演示效果
     *
     * 注意: 如果用户在回调函数外进行应答, 需要自行保存msg_id和rrpc_id字符串, 因为回调函数入参在退出回调函数后将被SDK销毁, 不可以再访问到
     */

    /*
    {
        aiot_dm_msg_t msg;

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_SYNC_SERVICE_REPLY;
        msg.data.sync_service_reply.rrpc_id = recv->data.sync_service_invoke.rrpc_id;
        msg.data.sync_service_reply.msg_id = recv->data.sync_service_invoke.msg_id;
        msg.data.sync_service_reply.code = 200;
        msg.data.sync_service_reply.service_id = "SetLightSwitchTimer";
        msg.data.sync_service_reply.data = "{}";
        int32_t res = aiot_dm_send(dm_handle, &msg);
        if (res < 0) {
            printf("aiot_dm_send failed\r\n");
        }
    }
    */
}

static void demo_dm_recv_raw_data(void *dm_handle, const aiot_dm_recv_t *recv, void *userdata)
{
    printf("demo_dm_recv_raw_data raw data len = %d\r\n", (int)(recv->data.raw_data.data_len));
    /* TODO: 以下代码演示如何发送二进制格式数据, 若使用需要有相应的数据透传脚本部署在云端 */
    /*
    {
        aiot_dm_msg_t msg;
        uint8_t raw_data[] = {0x01, 0x02};

        memset(&msg, 0, sizeof(aiot_dm_msg_t));
        msg.type = AIOT_DMMSG_RAW_DATA;
        msg.data.raw_data.data = raw_data;
        msg.data.raw_data.data_len = sizeof(raw_data);
        aiot_dm_send(dm_handle, &msg);
    }
    */
}
