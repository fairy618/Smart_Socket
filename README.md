# 项目概述
基于 单片机的Wi-Fi智能插座控制系统，主控采用乐鑫的ESP32-C3，软件开发环境为win11+ESP-IDF v5.0.1

# 项目介绍
项目名称：单片机的Wi-Fi智能插座控制系统。硬件环境为ESP32-C3，软件为ESP-IDF v5.0.1。云通信采用的是腾讯云，可以直接用腾讯连连小程序或者APP控制。  
* 实现的功能  
  * 初次使用配网
  * 温湿度检测
  * 光照强度检测
  * 电能计量
  * 远程控制
  * 定时开启（关闭）
  * 可自定义场景

# 环境依赖
项目的SDK github仓库：[ESP-IDF v5.0](https://github.com/espressif/esp-idf/tree/release/v5.0)，  
windows 快速入门教程[ESP32-C3 快速入门](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/get-started/index.html#get-started-how-to-get-esp-idf)，  
项目使用了乐鑫的[esp-qcloud](https://github.com/espressif/esp-qcloud)仓库。

# 硬件


# 项目结构  
 
    |—— components          // 自定义组件
        |—— BasicDrive      // 按键、继电器等驱动
            |—— include
                |—— BasicDrive.h
                |—— led_strip_encoder.h
            |—— BasicDrive.c
            |—— led_strip_encoder.c
            |—— CMakeLists.txt
        |—— BH1750          // 光照传感器驱动
            |—— include
                |—— BH1750.h
            |—— BH1750
            |—— CMakeLists.txt
        |—— HLW032          // 电能计量芯片驱动
            |—— include
                |—— HLW032.h
            |—— HLW032.c
            |—— CMakeLists.txt
        |—— SHTC3           // 温湿度传感器驱动
            |—— include
                |—— SHTC3.h
            |—— SHTC3.c
            |—— CMakeLists.txt
        |—— cloud           // 云函数相关
            |—— include
                |—— cloud.h
            |—— cloud.c
            |—— CMakeLists.txt
        |—— qcloud          // esp-qcloud仓库
    |—— main                // 主函数
        |—— CMakeLists.txt
        |—— Kconfig.projbuild   // menuconfig配置项
        |—— main.c
    |—— CMakeLists.txt
    |—— LICENSE
    |—— partitions.csv      // 分区表
    |—— README.md           // 本文件

# 注意事项

* 需要在menuconfig配置选择自定义分区表

* 在腾讯云平台创建产品、设备之后需要在menuconfig配置，具体可查看[esp-qcloud 烧录认证信息](https://github.com/espressif/esp-qcloud/blob/master/README.md#3-%E6%9E%84%E5%BB%BA%E7%83%A7%E5%BD%95%E8%BF%90%E8%A1%8C%E5%B7%A5%E7%A8%8B)

# 变更记录
2023/04/15 初始版本