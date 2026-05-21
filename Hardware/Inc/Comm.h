/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Comm.h
  * @brief   OneNET MQTT 上报驱动头文件
  *
  * 功能说明：
  * - 使用 ESP8266 AT 指令完成 OneNET MQTT 登录与上报
  * - 将本地 RTC 时间、温湿度和天气信息上传到 OneNET 物模型
  * - 用户需要在本文件中填入自己的设备标识与 MQTT 密码
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __COMM_H__
#define __COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp8266.h"

/* ==================== OneNET 设备配置 ==================== */
#define ONENET_PRODUCT_ID        "D8MiDF905e"
#define ONENET_DEVICE_ID         "ESP_01S"
#define ONENET_CLIENT_ID         "ESP_01S"
#define ONENET_USERNAME          ONENET_PRODUCT_ID
#define ONENET_PASSWORD          "version=2018-10-31&res=products%2FD8MiDF905e%2Fdevices%2FESP_01S&et=1805693871&method=md5&sign=vBWBRC%2F02qLSaLRWssubLQ%3D%3D"

#define ONENET_MQTT_HOST         "mqtts.heclouds.com"
#define ONENET_MQTT_PORT         1883

#define ONENET_UPLOAD_INTERVAL_MS 5000U
#define ONENET_TIME_STR_SIZE      32
#define ONENET_PAYLOAD_SIZE       512

extern volatile uint8_t g_comm_ready_to_publish;
extern volatile uint8_t g_alarm_icon_visible;

/* ==================== OneNET 主题字符串 ==================== */
#define ONENET_TOPIC_FORMAT      "$sys/%s/%s/thing/property/post"
#define ONENET_TOPIC_POST        "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_ID "/thing/property/post"
#define ONENET_TOPIC_PROPERTY_POST_REPLY "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_ID "/thing/property/post/reply"
#define ONENET_TOPIC_PROPERTY_SET       "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_ID "/thing/property/set"
#define ONENET_TOPIC_PROPERTY_SET_REPLY "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_ID "/thing/property/set_reply"

/* ==================== Function declarations ==================== */
int Comm_Init(void);
int OneNET_MQTT_Init(void);
int OneNET_PublishDeviceData(const char *time_str, float temperature, float humidity, const char *weather);
void Comm_TaskLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H__ */
