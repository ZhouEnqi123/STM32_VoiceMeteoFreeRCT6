/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Comm.c
  * @brief   OneNET MQTT 上报驱动实现
  *
  * 实现说明：
  * - 使用 ESP8266 AT 指令进行 MQTT 用户配置与连接
  * - 将 RTC 时间、AHT20 温湿度、网络天气上报到 OneNET
  * - 数据上报遵循 OneNET 物模型：Device_Data.time/temperature/humidity/weather
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Comm.h"
#include "rtc.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

extern volatile int esp_init_done;
extern volatile uint8_t g_comm_ready_to_publish;
extern volatile uint8_t g_comm_upload_preparing;
extern volatile float g_latest_temperature;
extern volatile float g_latest_humidity;
extern osMessageQueueId_t SensorQueueHandle;

static uint8_t mqtt_ready = 0;
static char g_onenet_topic[128] = {0};
static char g_onenet_payload[ONENET_PAYLOAD_SIZE] = {0};
static char g_onenet_cmd[256] = {0};
static char g_onenet_rx[RX_BUFFER_SIZE] = {0};

static void Comm_JsonEscapeUtf8(const char *input, char *output, size_t output_len)
{
    if (output == NULL || output_len == 0) {
        return;
    }

    if (input == NULL) {
        output[0] = '\0';
        return;
    }

    size_t out = 0;
    for (size_t i = 0; input[i] != '\0' && out + 1 < output_len; ++i) {
        unsigned char c = (unsigned char)input[i];

        if (c == '"' || c == '\\') {
            if (out + 2 >= output_len) {
                break;
            }
            output[out++] = '\\';
            output[out++] = (char)c;
            continue;
        }

        if (c < 0x20) {
            output[out++] = ' ';
            continue;
        }

        output[out++] = (char)c;
    }

    output[out] = '\0';
}

static void Comm_FormatTime(char *buffer, size_t length)
{
    if (buffer == NULL || length == 0) {
        return;
    }

    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef rtc_date = {0};
    if (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) {
        snprintf(buffer, length, "0000/00/00 00:00:00");
        return;
    }

    snprintf(buffer, length, "%04d/%02d/%02d %02d:%02d:%02d",
             2000 + rtc_date.Year,
             rtc_date.Month,
             rtc_date.Date,
             rtc_time.Hours,
             rtc_time.Minutes,
             rtc_time.Seconds);
}

static void Comm_ClearTransportState(char *rx_response)
{
    if (rx_response == NULL) {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CIPMODE=0\r\n", strlen("AT+CIPMODE=0\r\n"), 500);
    vTaskDelay(pdMS_TO_TICKS(150));
    HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CIPCLOSE\r\n", strlen("AT+CIPCLOSE\r\n"), 500);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static int OneNET_SubscribeTopic(const char *topic, char *rx_response, size_t rx_size)
{
    char cmd[256] = {0};

    if (topic == NULL || rx_response == NULL || rx_size == 0) {
        return -1;
    }

    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",0\r\n", topic);
    DebugPrintf("[Comm] Subscribing topic: %s\r\n", topic);

    if (ESP8266_SendCmd(cmd, "OK", 6000, rx_response, (uint16_t)rx_size) != 0) {
        DebugPrintf("[Comm] Topic subscribe failed: %s\r\n", topic);
        return -1;
    }

    return 0;
}

static int OneNET_MQTT_IsConnectedEvent(const char *response)
{
    if (response == NULL) {
        return 0;
    }

    if (strstr(response, "+MQTTCONNECTED:0,1") != NULL ||
        strstr(response, "+MQTTCONN:0,0") != NULL ||
        strstr(response, "+MQTTCONN: 0,0") != NULL) {
        return 1;
    }
    return 0;
}

int OneNET_MQTT_Init(void)
{
    char cmd[256] = {0};
    char rx_response[RX_BUFFER_SIZE] = {0};

    if (!g_wifi_connected) {
        DebugPrintf("[Comm] WiFi not ready, cannot initialize OneNET\r\n");
        return -1;
    }

    DebugPrintf("[Comm] OneNET MQTT init start\r\n");

    Comm_ClearTransportState(rx_response);
    vTaskDelay(pdMS_TO_TICKS(500));

    snprintf(cmd, sizeof(cmd), "AT+CIPMUX=0\r\n");
    if (ESP8266_SendCmd(cmd, "OK", CMD_TIMEOUT_MS, rx_response, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[Comm] AT+CIPMUX=0 failed\r\n");
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
             ONENET_CLIENT_ID,
             ONENET_USERNAME,
             ONENET_PASSWORD);
    if (ESP8266_SendCmd(cmd, "OK", 6000, rx_response, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[Comm] MQTTUSERCFG failed, cleaning transport state and retrying once\r\n");

        Comm_ClearTransportState(rx_response);
        vTaskDelay(pdMS_TO_TICKS(1000));

        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
                 ONENET_CLIENT_ID,
                 ONENET_USERNAME,
                 ONENET_PASSWORD);
        if (ESP8266_SendCmd(cmd, "OK", 6000, rx_response, RX_BUFFER_SIZE) != 0) {
            DebugPrintf("[Comm] MQTTUSERCFG still failed, hard-reset ESP8266 then retry once\r\n");

            HAL_UART_Transmit(&huart2, (uint8_t *)"AT+RST\r\n", strlen("AT+RST\r\n"), 500);
            vTaskDelay(pdMS_TO_TICKS(8000));

            if (ESP8266_Init() != 0 || !g_wifi_connected) {
                DebugPrintf("[Comm] ESP8266 re-init failed before MQTTUSERCFG retry\r\n");
                return -1;
            }

            snprintf(cmd, sizeof(cmd), "AT+CIPMUX=0\r\n");
            if (ESP8266_SendCmd(cmd, "OK", CMD_TIMEOUT_MS, rx_response, RX_BUFFER_SIZE) != 0) {
                DebugPrintf("[Comm] AT+CIPMUX=0 failed after reset\r\n");
                return -1;
            }

            vTaskDelay(pdMS_TO_TICKS(500));

            snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
                     ONENET_CLIENT_ID,
                     ONENET_USERNAME,
                     ONENET_PASSWORD);
            if (ESP8266_SendCmd(cmd, "OK", 6000, rx_response, RX_BUFFER_SIZE) != 0) {
                DebugPrintf("[Comm] MQTTUSERCFG still failed after reset\r\n");
                return -1;
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,1\r\n",
             ONENET_MQTT_HOST,
             ONENET_MQTT_PORT);
    DebugPrintf("[Comm] Trying MQTT connect to %s:%d using cfg=0\r\n",
                ONENET_MQTT_HOST, ONENET_MQTT_PORT);
    if (ESP8266_SendCmd(cmd, "OK", 10000, rx_response, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[Comm] OneNET MQTT connect command failed\r\n");
        return -1;
    }

    if (!OneNET_MQTT_IsConnectedEvent(rx_response)) {
        if (ESP8266_WaitResponse("+MQTTCONNECTED:0,1", 5000, rx_response, RX_BUFFER_SIZE) != 0 &&
            ESP8266_WaitResponse("+MQTTCONN:0,0", 5000, rx_response, RX_BUFFER_SIZE) != 0 &&
            ESP8266_WaitResponse("+MQTTCONN: 0,0", 5000, rx_response, RX_BUFFER_SIZE) != 0) {
            DebugPrintf("[Comm] OneNET MQTT connect event not received\r\n");
            return -1;
        }
    }

    if (OneNET_SubscribeTopic(ONENET_TOPIC_PROPERTY_POST_REPLY, rx_response, RX_BUFFER_SIZE) != 0) {
        return -1;
    }

    if (OneNET_SubscribeTopic(ONENET_TOPIC_PROPERTY_SET, rx_response, RX_BUFFER_SIZE) != 0) {
        return -1;
    }

    DebugPrintf("[Comm] OneNET MQTT ready\r\n");
    return 0;
}

int OneNET_PublishDeviceData(const char *time_str, float temperature, float humidity, const char *weather)
{
    if (time_str == NULL || weather == NULL) {
        return -1;
    }

    char safe_time[ONENET_TIME_STR_SIZE * 2] = {0};
    char safe_weather[64] = {0};

    Comm_JsonEscapeUtf8(time_str, safe_time, sizeof(safe_time));
    Comm_JsonEscapeUtf8(weather, safe_weather, sizeof(safe_weather));
    if (safe_weather[0] == '\0') {
        strncpy(safe_weather, "unknown", sizeof(safe_weather) - 1);
    }

    snprintf(g_onenet_topic, sizeof(g_onenet_topic), ONENET_TOPIC_FORMAT, ONENET_PRODUCT_ID, ONENET_DEVICE_ID);
    snprintf(g_onenet_payload, sizeof(g_onenet_payload),
             "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"Device_Data\":{\"value\":{"
             "\"time\":\"%s\","
             "\"temperature\":%.1f,"
             "\"humidity\":%.1f,"
             "\"weather\":\"%s\"}}}}",
             safe_time, temperature, humidity, safe_weather);

    size_t payload_len = strlen(g_onenet_payload);
    if (payload_len == 0 || payload_len >= ONENET_PAYLOAD_SIZE) {
        DebugPrintf("[Comm] Payload size invalid: %u\r\n", (unsigned int)payload_len);
        return -1;
    }

    DebugPrintf("[Comm] OneNET payload: %s\r\n", g_onenet_payload);

    snprintf(g_onenet_cmd, sizeof(g_onenet_cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,0\r\n",
             g_onenet_topic, (unsigned int)payload_len);
    if (ESP8266_SendCmd(g_onenet_cmd, ">", 10000, g_onenet_rx, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[Comm] MQTTPUBRAW prompt not received\r\n");
        return -1;
    }

    DebugPrintf("[Comm] Publishing OneNET payload, len=%u\r\n", (unsigned int)payload_len);
    HAL_UART_Transmit(&huart2, (uint8_t *)g_onenet_payload, payload_len, 2000);

    if (ESP8266_WaitResponse("+MQTTPUB:OK", 10000, g_onenet_rx, RX_BUFFER_SIZE) != 0 &&
        ESP8266_WaitResponse("OK", 10000, g_onenet_rx, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[Comm] OneNET publish failed\r\n");
        return -1;
    }

    DebugPrintf("[Comm] OneNET publish succeeded\r\n");
    return 0;
}

int Comm_Init(void)
{
    if (mqtt_ready) {
        return 0;
    }

    if (!g_wifi_connected) {
        DebugPrintf("[Comm] WiFi disconnected, cannot init OneNET\r\n");
        return -1;
    }

    if (OneNET_MQTT_Init() != 0) {
        mqtt_ready = 0;
        return -1;
    }

    mqtt_ready = 1;
    return 0;
}

void Comm_TaskLoop(void)
{
    struct {
        float temperature;
        float humidity;
    } sensor_data = {0.0f, 0.0f};

    char time_string[ONENET_TIME_STR_SIZE] = {0};
    char weather_string[32] = {0};
    uint8_t waiting_for_sync_logged = 0;
    uint32_t waiting_for_sync_tick = 0;

    for (;;) {
        if (!esp_init_done || !g_wifi_connected) {
            waiting_for_sync_logged = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!g_comm_ready_to_publish) {
            if (g_comm_upload_preparing) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            uint32_t now_tick = xTaskGetTickCount();
            if (!waiting_for_sync_logged || (now_tick - waiting_for_sync_tick) >= pdMS_TO_TICKS(10000)) {
                DebugPrintf("[Comm] Waiting for weather/time sync before upload\r\n");
                waiting_for_sync_logged = 1;
                waiting_for_sync_tick = now_tick;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        waiting_for_sync_logged = 0;

        if (Comm_Init() != 0) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (osMessageQueueGet(SensorQueueHandle, &sensor_data, NULL, 0) == osOK) {
            DebugPrintf("[Comm] Sensor data refreshed: %.1fC %.1f%%\r\n",
                        sensor_data.temperature, sensor_data.humidity);
        } else {
            sensor_data.temperature = g_latest_temperature;
            sensor_data.humidity = g_latest_humidity;
        }

        Comm_FormatTime(time_string, sizeof(time_string));
        if (strlen(g_weather.weather_text) > 0) {
            snprintf(weather_string, sizeof(weather_string), "%s", g_weather.weather_text);
        } else {
            snprintf(weather_string, sizeof(weather_string), "%s", "unknown");
        }

        if (OneNET_PublishDeviceData(time_string, sensor_data.temperature,
                                     sensor_data.humidity, weather_string) != 0) {
            DebugPrintf("[Comm] Publish failed, will retry later\r\n");
            mqtt_ready = 0;
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(ONENET_UPLOAD_INTERVAL_MS));
    }
}
