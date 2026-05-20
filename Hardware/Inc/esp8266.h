/**
  ******************************************************************************
  * @file           : esp8266.h
  * @brief          : ESP8266 WiFi 模块驱动头文件
  * 
  * 功能概述：
  * - 双串口协同：UART2 与 ESP8266 通信，UART1 调试输出
  * - 低冗余高健壮：使用标准库 string.h 手动解析，禁止 cJSON
  * - 非阻塞设计：所有延时使用 FreeRTOS vTaskDelay，支持超时退出
  *
  * 硬件提醒：
  * - ESP8266 3.3V 供电电流需求：500mA+，请确保电源充足
  * - UART1 (PA9/PA10)：调试控制台 115200 波特率
  * - UART2 (PA2/PA3)：ESP8266 通信 115200 波特率
  ******************************************************************************
  */

#ifndef __ESP8266_H__
#define __ESP8266_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"  // 用于 DebugPrintf

/* ==================== 宏定义 - WiFi 参数 ==================== */
#define WIFI_SSID       "ZhouEnqi"
#define WIFI_PWD        "zeq123123"

/* ==================== 宏定义 - API 参数 ==================== */
#define SENIVERSE_KEY   "SVTaGHkZSNZIz6BVX"
#define CITY            "jinhua"

#define TIME_HOST       "api.pinduoduo.com"
#define TIME_PATH       "/api/server/_stm"

/* ==================== 宏定义 - 通信参数 ==================== */
#define RX_BUFFER_SIZE  1024        // 接收缓冲区大小（增加以容纳完整 HTTP 响应）
#define TX_BUFFER_SIZE  256         // 发送缓冲区大小
#define CMD_TIMEOUT_MS  2000        // 普通AT命令超时（毫秒）
#define CWJAP_TIMEOUT_MS 8000       // WiFi连接超时（毫秒），至少8秒
#define TCP_TIMEOUT_MS  5000        // TCP连接超时（毫秒）

/* ==================== 宏定义 - 文本搜索偏移 ==================== */
#define JSON_TEXT_OFFSET 8          // "text":"后的偏移量

/* ==================== 结构体定义 ==================== */

/**
 * @brief 天气数据结构体
 * 用于存储从 API 获取的天气信息
 */
typedef struct {
    char weather_text[32];          // 天气描述（如"晴"、"多云"等），保留更完整的中文结果
} NetWeather_t;

/* ==================== 全局变量声明 ==================== */
extern NetWeather_t g_weather;      // 全局天气数据，供其他任务访问
extern uint8_t g_wifi_connected;      // WiFi 连接状态：0=未连接, 1=已连接

extern volatile char g_uart2_rx_buffer[RX_BUFFER_SIZE];  // UART2接收缓冲区

/* ==================== 函数声明 ==================== */

/**
 * @brief ESP8266 模块初始化
 * 
 * 流程：
 * 1. AT+RESTORE (恢复出厂设置，等待2秒)
 * 2. AT+CWMODE=1 (配置WiFi模式为STA)
 * 3. AT+CWJAP="SSID","PWD" (连接WiFi，超时8秒)
 * 
 * @return int 返回 0 表示初始化成功，-1 表示失败
 * 
 * @note 调用此函数前，确保：
 *       - UART1/UART2 已初始化
 *       - FreeRTOS 已启动（使用 vTaskDelay）
 *       - ESP8266 已供电且可工作（3.3V 500mA+）
 */
int ESP8266_Init(void);
int ESP8266_ResetAndReinit(void);

/**
 * @brief 获取网络天气信息
 * 
 * 流程：
 * 1. AT+CIPSTART 建立 TCP 连接到 api.seniverse.com:80
 * 2. AT+CIPMODE=1 启用透传模式
 * 3. AT+CIPSEND 进入透传数据发送
 * 4. 发送 HTTP GET 请求（包含 API_KEY 和 CITY）
 * 5. 接收响应并手动解析 JSON：
 *    - 使用 strstr 查找 "text":"
 *    - 使用 strchr 定位结束引号
 *    - 提取天气单词到 g_weather.weather_text
 * 6. 发送 +++ 退出透传（前后各延时500ms，不带\r\n）
 * 7. AT+CIPMODE=0 关闭透传，AT+CIPCLOSE 关闭连接
 * 
 * @return int 返回 0 表示成功，-1 表示失败
 * 
 * @note 内存安全：
 *       - 所有接收数据存储在静态 rx_buffer[512] 中，禁止 malloc
 *       - strstr 结果必须 NULL 检查，防止 HardFault
 *       - strncpy 长度受限于 weather_text[16]，防止溢出
 * 
 * @note 后处理：
 *       - 如果获取失败，g_weather.weather_text 保持上次值或初始值
 *       - LinkQueue 中放置 flag=1 表示天气已更新，供 DisplayTask 使用
 */
int Get_Weather(void);
int ESP8266_GetTime(void);

/**
 * @brief 发送 AT 命令并接收响应
 * 
 * 功能：
 * - 将命令发送到 ESP8266 (UART2)
 * - 同时将发送内容回显到 UART1（调试输出）
 * - 接收 UART2 响应并即时转发到 UART1（透明传输监控）
 * - 等待特定响应 (e.g., "OK"、"ERROR"、"CONNECT")
 * - 若超时则退出并返回失败
 * 
 * @param cmd        AT 命令字符串 (e.g., "AT+CWMODE=1\r\n")
 * @param response   期望的响应字符串 (e.g., "OK")
 * @param timeout_ms 超时时间（毫秒）
 * @param rx_buf     用于存储响应数据的缓冲区指针
 * @param rx_size    缓冲区大小
 * 
 * @return int 返回 0 表示收到期望响应，-1 表示超时或错误
 * 
 * @note 关键设计：
 *       - UART2 接收使用 DMA 或中断处理，此函数轮询接收
 *       - UART1 用于实时监控和调试，便于排查 AT 命令问题
 *       - 接收数据存储在全局静态缓冲区，防止栈溢出
 */
int ESP8266_SendCmd(const char *cmd, const char *response, 
                    uint32_t timeout_ms, char *rx_buf, uint16_t rx_size);

int ESP8266_InitLock(void);
int ESP8266_WaitResponse(const char *wait_str, uint32_t timeout_ms, char *rx_buf, uint16_t rx_size);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H__ */
