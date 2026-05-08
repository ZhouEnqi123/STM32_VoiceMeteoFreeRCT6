/**
  ******************************************************************************
  * @file           : esp8266.c
  * @brief          : ESP8266 WiFi 模块驱动源文件
  * 
  * 实现说明：
  * - 双串口协同：UART2 与 ESP8266 通信，UART1 调试输出
  * - 手动 JSON 解析：使用 strstr、strchr 等标准库函数，禁止 cJSON
  * - 非阻塞延时：所有等待使用 FreeRTOS vTaskDelay，配合超时机制
  * - 内存安全：所有缓冲区静态分配，无 malloc
  * 
  * 硬件提醒：
  * - 确保 ESP8266 3.3V 供电电流 >= 500mA
  * - UART1 (115200)：调试监控
  * - UART2 (115200)：ESP8266 通信（带 DMA 接收）
  ******************************************************************************
  */

#include "esp8266.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ==================== 外部声明 ==================== */
extern DMA_HandleTypeDef hdma_usart2_rx;  // UART2 DMA 接收处理器

/* ==================== 全局变量 ==================== */

/** @brief 网络天气数据，供全局访问 */
NetWeather_t g_weather = {0};

/** @brief UART2 接收缓冲区（静态分配，1024字节） */
volatile char g_uart2_rx_buffer[RX_BUFFER_SIZE] = {0};


/** @brief ESP8266 命令与响应临时缓冲区，避免在任务堆栈上分配 1KB 缓冲 */
static char esp8266_rx_buffer[RX_BUFFER_SIZE] = {0};
static char esp8266_cmd_buffer[256] = {0};

/* ==================== 私有函数声明 ==================== */

/**
 * @brief 清空 UART2 接收缓冲区
 */
static void ClearRxBuffer(void);

/* ==================== UART2 缓冲区管理 ==================== */

/**
 * @brief 清空 UART2 接收缓冲区
 */
static uint16_t GetUart2DmaRxLen(void)
{
    return (uint16_t)(RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
}

static void ClearRxBuffer(void)
{
    taskENTER_CRITICAL();
    HAL_UART_DMAStop(&huart2);
    memset((void *)g_uart2_rx_buffer, 0, RX_BUFFER_SIZE);
    taskEXIT_CRITICAL();
}

/* ==================== 核心通信函数 ==================== */

/**
 * @brief 发送 AT 命令并等待响应
 * 
 * 工作流程：
 * 1. 清空接收缓冲区
 * 2. 将命令字符串发送到 UART2
 * 3. 同时将发送的命令回显到 UART1（调试监控）
 * 4. 轮询接收 UART2 的响应数据
 * 5. 查找期望的响应字符串
 * 6. 若找到则返回成功，否则超时后返回失败
 * 
 * @param cmd        AT 命令 (e.g., "AT+CWMODE=1\r\n")
 * @param response   期望响应 (e.g., "OK")
 * @param timeout_ms 超时时间（毫秒）
 * @param rx_buf     响应缓冲区指针
 * @param rx_size    缓冲区大小
 * 
 * @return 0 = 成功，-1 = 失败/超时
 * 
 * 注意：此实现使用简单的轮询和缓冲区累积
 *      实际应用中，应该使用中断或 DMA 完成回调来处理接收
 */
int ESP8266_SendCmd(const char *cmd, const char *response, 
                    uint32_t timeout_ms, char *rx_buf, uint16_t rx_size)
{
    if (cmd == NULL || response == NULL || rx_buf == NULL) {
        DebugPrintf("[ERR] ESP8266_SendCmd: Invalid parameter\r\n");
        return -1;
    }
    
    // 1. 清空接收缓冲区
    ClearRxBuffer();
    
    // 2. 打印发送的命令到 UART1（调试输出）
    DebugPrintf("[TX] %s", cmd);
    
    // 3. 启动 DMA 接收并发送命令到 ESP8266 (UART2)
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 DMA receive start failed\r\n");
        return -1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 1000);
    
    // 4. 等待接收数据
    uint32_t start_ticks = xTaskGetTickCount();
    uint32_t timeout_ticks = (timeout_ms / portTICK_PERIOD_MS);
    uint16_t last_rx_len = 0;
    uint32_t last_data_ticks = start_ticks;
    
    // 先等待100ms让 ESP8266 开始响应
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 然后进行多次轮询读取
    // 改进：只在新数据到达时进行一次检查，避免重复处理
    while (xTaskGetTickCount() - start_ticks < timeout_ticks) {
        vTaskDelay(pdMS_TO_TICKS(50));  // 延时50ms
        
        // 检查是否有新数据到达
        uint16_t current_len = GetUart2DmaRxLen();
        if (current_len > last_rx_len) {
            last_data_ticks = xTaskGetTickCount();
            
            taskENTER_CRITICAL();
            uint16_t copy_len = (current_len > rx_size - 1) ? (rx_size - 1) : current_len;
            memcpy(rx_buf, (const void *)g_uart2_rx_buffer, copy_len);
            taskEXIT_CRITICAL();
            rx_buf[copy_len] = '\0';
            last_rx_len = current_len;
            
            // 打印接收到的数据到 UART1（透明转发）
            DebugPrintf("[RX] %s\r\n", rx_buf);
            
            // 查找期望的响应字符串
            if (strstr(rx_buf, response) != NULL) {
                HAL_UART_DMAStop(&huart2);
                return 0;  // 成功找到响应
            }
            
            // 检查是否有错误响应
            if (strstr(rx_buf, "ERROR") != NULL && strcmp(response, "OK") == 0) {
                DebugPrintf("[ERR] Received ERROR response\r\n");
                HAL_UART_DMAStop(&huart2);
                return -1;
            }
            
            // 检查缓冲区是否快满
            if (current_len >= RX_BUFFER_SIZE - 1) {
                DebugPrintf("[WARN] RX buffer overflow\r\n");
                HAL_UART_DMAStop(&huart2);
                return -1;
            }
        }
        
        // 额外的超时机制：如果在固定时间内没有新数据到达，可能连接已断开
        if (xTaskGetTickCount() - last_data_ticks > pdMS_TO_TICKS(2000)) {
            if (last_rx_len == 0) {
                // 一直没有收到任何数据
                DebugPrintf("[TIMEOUT] No data received for 2s\r\n");
                HAL_UART_DMAStop(&huart2);
                return -1;
            }
        }
    }
    
    DebugPrintf("[TIMEOUT] Command timeout after %lu ms\r\n", timeout_ms);
    HAL_UART_DMAStop(&huart2);
    return -1;  // 超时
}

/* ==================== ESP8266 初始化函数 ==================== */

/**
 * @brief ESP8266 模块初始化
 * 
 * 初始化流程（共3个关键指令）：
 * 1. AT+RESTORE    → 恢复出厂设置（等待2秒）
 * 2. AT+CWMODE=1   → 配置为 STA 模式（连接WiFi客户端）
 * 3. AT+CWJAP      → 连接到指定 WiFi（超时8秒）
 * 
 * @return 0 = 初始化成功，-1 = 初始化失败
 */
int ESP8266_Init(void)
{
    int ret = 0;
    
    DebugPrintf("\n========== ESP8266 Initialization ==========\r\n");
    
    // 步骤 1: 检查模块是否已经可用
    DebugPrintf("\n[Step 1] Checking ESP8266 responsiveness...\r\n");

    int at_tries = 3;
    int at_ok = 0;
    while (at_tries-- > 0) {
        ClearRxBuffer();
        if (ESP8266_SendCmd("AT\r\n", "OK", 2000, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
            at_ok = 1;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!at_ok) {
        DebugPrintf("[WARN] No AT response, soft resetting ESP8266...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)"AT+RST\r\n", strlen("AT+RST\r\n"), 500);
        vTaskDelay(pdMS_TO_TICKS(8000));
        ClearRxBuffer();
        at_tries = 5;
        while (at_tries-- > 0) {
            if (ESP8266_SendCmd("AT\r\n", "OK", 3000, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
                at_ok = 1;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (!at_ok) {
            DebugPrintf("[ERROR] ESP8266 did not respond after reset\r\n");
            return -1;
        }
    }
    DebugPrintf("[OK] ESP8266 responsive\r\n");

    ClearRxBuffer();
    if (ESP8266_SendCmd("ATE0\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
        DebugPrintf("[OK] Echo disabled\r\n");
    } else {
        DebugPrintf("[WARN] ATE0 failed, continuing with echo enabled\r\n");
    }

    // 步骤 2: AT+CWMODE=1 - 配置为 STA 模式
    DebugPrintf("\n[Step 2] Setting WiFi mode (STA)...\r\n");
    ret = ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[FAIL] AT+CWMODE=1 failed\r\n");
        return -1;
    }
    DebugPrintf("[OK] AT+CWMODE=1 succeeded\r\n");

    // 探测当前 WiFi 连接状态
    ClearRxBuffer();
    if (ESP8266_SendCmd("AT+CWJAP?\r\n", "OK", 5000, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
        if (strstr(esp8266_rx_buffer, "No AP") == NULL) {
            DebugPrintf("[OK] ESP8266 already connected to WiFi\r\n");
            return 0;
        }
        DebugPrintf("[INFO] ESP8266 not connected to AP, starting join...\r\n");
    } else {
        DebugPrintf("[WARN] AT+CWJAP? probe failed, proceeding to join...\r\n");
    }

    // 步骤 3: AT+CWJAP - 连接到 WiFi
    DebugPrintf("[Step 3] Connecting to WiFi (%s)...\r\n", WIFI_SSID);
    
    // 构造 WiFi 连接命令
    snprintf(esp8266_cmd_buffer, sizeof(esp8266_cmd_buffer), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    
    // CWJAP 需要较长的超时时间（至少8秒），因为 WiFi 连接可能较慢
    ret = ESP8266_SendCmd(esp8266_cmd_buffer, "OK", CWJAP_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[FAIL] AT+CWJAP failed\r\n");
        return -1;
    }
    DebugPrintf("[OK] WiFi connected successfully\r\n");
    
    DebugPrintf("\n========== ESP8266 Init Success ==========\r\n");
    return 0;
}

/* ==================== 获取天气函数 ==================== */

/**
 * @brief 获取网络天气信息
 * 
 * 工作流程：
 * 1. 建立 TCP 连接 (AT+CIPSTART)
 * 2. 启用透传模式 (AT+CIPMODE=1)
 * 3. 进入透传数据发送 (AT+CIPSEND)
 * 4. 发送 HTTP GET 请求
 * 5. 接收响应并手动解析 JSON
 * 6. 发送 +++ 退出透传（关键：无 \r\n，前后各延时500ms）
 * 7. 关闭透传和连接
 * 
 * @return 0 = 获取成功，-1 = 获取失败
 * 
 * @note 内存安全提醒：
 *       - rx_buffer 大小为 1024 字节，能容纳完整 HTTP 响应
 *       - strstr 结果必须 NULL 检查
 *       - strncpy 长度必须小于 weather_text[16]
 */
int Get_Weather(void)
{
    int ret = 0;
    
    DebugPrintf("\n========== Get Weather ==========\r\n");
    
    // 步骤 0: 清理任何现存的 TCP 连接，避免连接状态冲突
    // 先禁用透传模式（如果之前未正确退出），再关闭连接
    DebugPrintf("[Step 0] Cleaning up previous connections...\r\n");
    
    // 明确禁用透传模式
    ClearRxBuffer();
    ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    vTaskDelay(pdMS_TO_TICKS(200));  // 增加延迟
    
    // 尝试关闭任何打开的连接（可能失败，但没关系）
    ClearRxBuffer();
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    vTaskDelay(pdMS_TO_TICKS(200));  // 增加延迟
    
    // 额外清理：再次禁用透传模式，确保模块处于命令模式
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(500));  // 关键：等待足够时间让模块稳定
    DebugPrintf("[OK] Previous connections cleaned\r\n");
    
    // 步骤 1: AT+CIPSTART - 建立 TCP 连接
    DebugPrintf("[Step 1] Connecting to api.seniverse.com:80...\r\n");
    
    // 清理缓冲区，确保没有旧数据干扰
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // AT+CIPSTART 可能返回 "CONNECT" 或 "OK"，两种都表示成功
    // 使用 rx_buffer 复用接收空间，避免额外栈占用
    ret = ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n", 
                          "CONNECT", TCP_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        // 如果 CONNECT 超时，可能网络问题或响应不同
        DebugPrintf("[WARN] AT+CIPSTART did not return CONNECT, checking for OK...\r\n");
        
        // 清理缓冲区，重新尝试
        ClearRxBuffer();
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // 重试一次，这次等待 "OK"
        ret = ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n", 
                              "OK", TCP_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
        if (ret != 0) {
            DebugPrintf("[FAIL] TCP connection failed\r\n");
            return -1;
        }
    }
    DebugPrintf("[OK] TCP connected\r\n");
    
    // 步骤 2: AT+CIPMODE=1 - 启用透传模式
    DebugPrintf("[Step 2] Enabling transparent transmission (CIPMODE=1)...\r\n");
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[FAIL] CIPMODE=1 failed\r\n");
        return -1;
    }
    DebugPrintf("[OK] Transparent mode enabled\r\n");
    
    // 步骤 3: AT+CIPSEND - 进入发送模式
    DebugPrintf("[Step 3] Entering transparent send mode (CIPSEND)...\r\n");
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = ESP8266_SendCmd("AT+CIPSEND\r\n", ">", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[FAIL] CIPSEND failed\r\n");
        return -1;
    }
    DebugPrintf("[OK] Ready to send HTTP request\r\n");
    
    // 步骤 4: 构造并发送 HTTP GET 请求
    DebugPrintf("[Step 4] Sending HTTP GET request...\r\n");
    
    // 构造 HTTP 请求
    // 格式: GET /v3/weather/now.json?key=XXXX&location=XXXX&language=zh-Hans HTTP/1.1\r\n...
    snprintf(esp8266_cmd_buffer, sizeof(esp8266_cmd_buffer),
             "GET /v3/weather/now.json?key=%s&location=%s&language=zh-Hans HTTP/1.1\r\n"
             "Host: api.seniverse.com\r\n"
             "Connection: close\r\n"
             "\r\n",
             SENIVERSE_KEY, CITY);
    
    HAL_UART_Transmit(&huart2, (uint8_t *)esp8266_cmd_buffer, strlen(esp8266_cmd_buffer), 1000);
    DebugPrintf("[TX] HTTP Request sent\r\n");
    
    // 步骤 5: 接收并解析响应
    DebugPrintf("[Step 5] Receiving HTTP response...\r\n");
    
    ClearRxBuffer();
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 DMA receive start failed for HTTP response\r\n");
        return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));  // 等待 3 秒确保完整 HTTP 响应到达（含 header + JSON body）
    
    // 由于 DMA 已接收数据，直接从 RX 缓冲区读取
    
    uint16_t rx_len = GetUart2DmaRxLen();
    if (rx_len > 0) {
        HAL_UART_DMAStop(&huart2);
        memcpy(esp8266_rx_buffer, (const void *)g_uart2_rx_buffer, (rx_len > RX_BUFFER_SIZE) ? RX_BUFFER_SIZE : rx_len);
        DebugPrintf("[RX] Response received (%d bytes)\r\n", rx_len);
        
        // 手动解析 JSON，寻找 "text":"XX"
        DebugPrintf("[Step 6] Parsing weather data...\r\n");
        
        char *text_pos = strstr(esp8266_rx_buffer, "\"text\":\"");
        if (text_pos == NULL) {
            DebugPrintf("[WARN] Weather text not found in response\r\n");
            // 不返回失败，允许使用旧数据
        } else {
            // 指针偏移到 JSON_TEXT_OFFSET (8) 位置，跳过 "text":"
            text_pos += JSON_TEXT_OFFSET;
            
            // 寻找下一个双引号，表示天气字符串的结束
            char *end_quote = strchr(text_pos, '"');
            if (end_quote == NULL) {
                DebugPrintf("[ERR] Malformed JSON: missing end quote\r\n");
                return -1;
            }
            
            // 计算天气字符串长度
            uint16_t weather_len = (uint16_t)(end_quote - text_pos);
            
            // 安全检查：防止缓冲区溢出
            // g_weather.weather_text 大小为 16，最多复制 15 字符 + \0
            if (weather_len >= sizeof(g_weather.weather_text)) {
                weather_len = sizeof(g_weather.weather_text) - 1;
                DebugPrintf("[WARN] Weather text truncated (original: %d bytes)\r\n", 
                           (int)(end_quote - text_pos));
            }
            
            // 使用 strncpy 安全复制
            strncpy(g_weather.weather_text, text_pos, weather_len);
            g_weather.weather_text[weather_len] = '\0';  // 确保以 \0 结尾
            
            DebugPrintf("[OK] Weather: %s\r\n", g_weather.weather_text);
        }
    } else {
        DebugPrintf("[WARN] No response received\r\n");
    }
    
    // 步骤 6: 发送 +++ 退出透传
    // 关键：+++ 必须单独发送，前后各延时500ms，且不带 \r\n
    DebugPrintf("[Step 7] Exiting transparent mode (sending +++, no \\r\\n)...\r\n");
    
    vTaskDelay(pdMS_TO_TICKS(500));  // 前延时 500ms
    HAL_UART_Transmit(&huart2, (uint8_t *)"+++", 3, 100);  // 发送 +++，不带 \r\n
    DebugPrintf("[TX] +++ (no \\r\\n)\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));  // 后延时 500ms
    
    // 步骤 7: AT+CIPMODE=0 - 关闭透传模式
    DebugPrintf("[Step 8] Disabling transparent mode (CIPMODE=0)...\r\n");
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[WARN] CIPMODE=0 failed (may already exited)\r\n");
    }
    
    // 步骤 8: AT+CIPCLOSE - 关闭 TCP 连接
    DebugPrintf("[Step 9] Closing TCP connection...\r\n");
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[WARN] CIPCLOSE failed\r\n");
    }
    
    DebugPrintf("\n========== Get Weather Success ==========\r\n");
    return 0;
}

/**
 * @brief UART2 接收中断回调（需要在 HAL 中调用）
 * 
 * 此函数应该在 UART2 的 RX 中断或 DMA 完成回调中调用，
 * 用于累积接收的数据到 g_uart2_rx_buffer
 * 
 * @param data 接收到的数据字节
 */
