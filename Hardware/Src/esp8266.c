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
#include "rtc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ==================== 外部声明 ==================== */
extern DMA_HandleTypeDef hdma_usart2_rx;  // UART2 DMA 接收处理器

/* ==================== 全局变量 ==================== */

/** @brief 网络天气数据，供全局访问 */
NetWeather_t g_weather = {0};

/** @brief 当前 WiFi 连接状态，DisplayTask 可据此显示 NoWIFI 图标 */
uint8_t g_wifi_connected = 0;

/** @brief UART2 接收缓冲区（静态分配，1024字节） */
volatile char g_uart2_rx_buffer[RX_BUFFER_SIZE] = {0};

#define TIME_HOST       "api.pinduoduo.com"
#define TIME_PATH       "/api/server/_stm"
#define TIME_REQUEST_TEMPLATE \
    "GET " TIME_PATH " HTTP/1.1\r\n" \
    "Host: " TIME_HOST "\r\n" \
    "Connection: close\r\n" \
    "\r\n"
#define BEIJING_OFFSET_SECONDS 28800U

/** @brief ESP8266 命令与响应临时缓冲区，避免在任务堆栈上分配 1KB 缓冲 */
static char esp8266_rx_buffer[RX_BUFFER_SIZE] = {0};
static char esp8266_cmd_buffer[256] = {0};

static osMutexId_t esp8266_mutex = NULL;

/* ==================== 私有函数声明 ==================== */

/**
 * @brief 清空 UART2 接收缓冲区
 */
static void ClearRxBuffer(void);
static int ESP8266_ParseServerTime(const char *buf, uint64_t *server_time_ms);
static int ESP8266_SetRtcFromUnix(uint32_t utc_seconds);
static void ESP8266_ExitTransparentMode(void);
static void ESP8266_QuickTransportCleanup(void);
static int ESP8266_MutexLock(uint32_t timeout_ms);
static void ESP8266_MutexUnlock(void);
int ESP8266_StartRxMonitor(void);

/* ==================== UART2 缓冲区管理 ==================== */

/**
 * @brief 清空 UART2 接收缓冲区
 */
static uint16_t GetUart2DmaRxLen(void)
{
    return (uint16_t)(RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
}

static uint16_t g_uart2_last_dma_len = 0;

static void ClearRxBuffer(void)
{
    taskENTER_CRITICAL();
    HAL_UART_DMAStop(&huart2);
    memset((void *)g_uart2_rx_buffer, 0, RX_BUFFER_SIZE);
    g_uart2_last_dma_len = 0;
    taskEXIT_CRITICAL();
}

int ESP8266_StartRxMonitor(void)
{
    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
        return 0;
    }

    ClearRxBuffer();
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 RX monitor start failed\r\n");
        return -1;
    }

    return 0;
}

int ESP8266_GetRxDelta(char *buffer, uint16_t buffer_size, uint16_t *out_length)
{
    uint16_t current_len = 0;
    uint16_t delta_len = 0;

    if (buffer == NULL || out_length == NULL || buffer_size == 0) {
        return -1;
    }

    buffer[0] = '\0';
    *out_length = 0;

    current_len = GetUart2DmaRxLen();
    if (current_len == g_uart2_last_dma_len) {
        return 0;
    }

    if (current_len > g_uart2_last_dma_len) {
        delta_len = (uint16_t)(current_len - g_uart2_last_dma_len);
        if (delta_len >= buffer_size) {
            delta_len = (uint16_t)(buffer_size - 1);
        }
        memcpy(buffer, (const void *)&g_uart2_rx_buffer[g_uart2_last_dma_len], delta_len);
    } else {
        uint16_t tail_len = (uint16_t)(RX_BUFFER_SIZE - g_uart2_last_dma_len);
        uint16_t head_len = current_len;

        delta_len = (uint16_t)(tail_len + head_len);
        if (delta_len >= buffer_size) {
            delta_len = (uint16_t)(buffer_size - 1);
        }

        if (tail_len >= buffer_size) {
            tail_len = (uint16_t)(buffer_size - 1);
            head_len = 0;
        } else if ((uint16_t)(tail_len + head_len) >= buffer_size) {
            head_len = (uint16_t)(buffer_size - 1 - tail_len);
        }

        memcpy(buffer, (const void *)&g_uart2_rx_buffer[g_uart2_last_dma_len], tail_len);
        if (head_len > 0) {
            memcpy(buffer + tail_len, (const void *)g_uart2_rx_buffer, head_len);
        }
    }

    buffer[delta_len] = '\0';
    *out_length = delta_len;
    g_uart2_last_dma_len = current_len;
    return 0;
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

    if (ESP8266_MutexLock(2000) != 0) {
        DebugPrintf("[ERR] ESP8266_SendCmd: failed to acquire mutex\r\n");
        return -1;
    }
    
    // 1. 清空接收缓冲区
    ClearRxBuffer();
    
    // 2. 打印发送的命令到 UART1（调试输出）
    DebugPrintf("[TX] %s", cmd);
    
    // 3. 启动 DMA 接收并发送命令到 ESP8266 (UART2)
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 DMA receive start failed\r\n");
        ESP8266_MutexUnlock();
        return -1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 1000);
    
    // 4. 等待接收数据
    uint32_t start_ticks = xTaskGetTickCount();
    uint32_t timeout_ticks = (timeout_ms / portTICK_PERIOD_MS);
    uint16_t last_rx_len = 0;
    uint8_t seen_error = 0;
    
    // 先等待100ms让 ESP8266 开始响应
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 然后进行多次轮询读取
    // 改进：只在新数据到达时进行一次检查，避免重复处理
    while (xTaskGetTickCount() - start_ticks < timeout_ticks) {
        vTaskDelay(pdMS_TO_TICKS(50));  // 延时50ms
        
        // 检查是否有新数据到达
        uint16_t current_len = GetUart2DmaRxLen();
        if (current_len > last_rx_len) {
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
                ESP8266_MutexUnlock();
                return 0;  // 成功找到响应
            }
            
            // ERROR 可能是前序命令残留，先记录并继续等待目标响应，避免误判
            if (strstr(rx_buf, "ERROR") != NULL && strcmp(response, "OK") == 0) {
                seen_error = 1;
            }
            
            // 检查缓冲区是否快满
            if (current_len >= RX_BUFFER_SIZE - 1) {
                DebugPrintf("[WARN] RX buffer overflow\r\n");
                HAL_UART_DMAStop(&huart2);
                ESP8266_MutexUnlock();
                return -1;
            }
        }
        
        // 不做固定 2s 的早超时，完整等待 timeout_ms，避免慢响应命令被误判失败
    }
    
    if (seen_error) {
        DebugPrintf("[ERR] Received ERROR response\r\n");
    }
    DebugPrintf("[TIMEOUT] Command timeout after %lu ms\r\n", timeout_ms);
    HAL_UART_DMAStop(&huart2);
    ESP8266_MutexUnlock();
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
            char *ssid_start = strstr(esp8266_rx_buffer, "+CWJAP:\"");
            if (ssid_start != NULL) {
                ssid_start += strlen("+CWJAP:\"");
                char *ssid_end = strchr(ssid_start, '"');
                if (ssid_end != NULL) {
                    size_t ssid_len = ssid_end - ssid_start;
                    if (ssid_len == strlen(WIFI_SSID) && strncmp(ssid_start, WIFI_SSID, ssid_len) == 0) {
                        g_wifi_connected = 1;
                        DebugPrintf("[OK] ESP8266 already connected to configured WiFi '%s'\r\n", WIFI_SSID);
                        return 0;
                    }
                }
            }
            g_wifi_connected = 0;
            DebugPrintf("[INFO] ESP8266 connected to other AP or unknown SSID, starting join...\r\n");
        } else {
            g_wifi_connected = 0;
            DebugPrintf("[INFO] ESP8266 not connected to AP, starting join...\r\n");
        }
    } else {
        g_wifi_connected = 0;
        DebugPrintf("[WARN] AT+CWJAP? probe failed, proceeding to join...\r\n");
    }

    // 步骤 3: AT+CWJAP - 连接到 WiFi
    DebugPrintf("[Step 3] Connecting to WiFi (%s)...\r\n", WIFI_SSID);
    
    // 构造 WiFi 连接命令
    snprintf(esp8266_cmd_buffer, sizeof(esp8266_cmd_buffer), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    
    // CWJAP 需要较长的超时时间（至少8秒），因为 WiFi 连接可能较慢
    ret = ESP8266_SendCmd(esp8266_cmd_buffer, "OK", CWJAP_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        g_wifi_connected = 0;
        DebugPrintf("[FAIL] AT+CWJAP failed\r\n");
        return -1;
    }
    g_wifi_connected = 1;
    DebugPrintf("[OK] WiFi connected successfully\r\n");
    
    DebugPrintf("\n========== ESP8266 Init Success ==========\r\n");
    return 0;
}

int ESP8266_ResetAndReinit(void)
{
    DebugPrintf("[ESP8266] Resetting module before cloud upload\r\n");

    ClearRxBuffer();
    HAL_UART_Transmit(&huart2, (uint8_t *)"AT+RST\r\n", strlen("AT+RST\r\n"), 500);
    vTaskDelay(pdMS_TO_TICKS(8000));

    if (ESP8266_Init() != 0) {
        DebugPrintf("[ESP8266] Reset and reinit failed\r\n");
        return -1;
    }

    DebugPrintf("[ESP8266] Reset and reinit success\r\n");
    return 0;
}

static void ESP8266_QuickTransportCleanup(void)
{
    ClearRxBuffer();
    HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CIPMODE=0\r\n", strlen("AT+CIPMODE=0\r\n"), 500);
    vTaskDelay(pdMS_TO_TICKS(120));
    ClearRxBuffer();
    HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CIPCLOSE\r\n", strlen("AT+CIPCLOSE\r\n"), 500);
    vTaskDelay(pdMS_TO_TICKS(120));
    ClearRxBuffer();
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
    int ret = -1;
    
    DebugPrintf("\n========== Get Weather ==========\r\n");

    // 先检查当前 WiFi 状态，如果已断开则直接返回失败，显示 NoWIFI 图标
    ClearRxBuffer();
    if (ESP8266_SendCmd("AT+CWJAP?\r\n", "OK", 5000, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
        if (strstr(esp8266_rx_buffer, "No AP") != NULL) {
            g_wifi_connected = 0;
            DebugPrintf("[WARN] WiFi disconnected (No AP)\r\n");
            goto cleanup;
        }
        if (strstr(esp8266_rx_buffer, "+CWJAP:") == NULL) {
            g_wifi_connected = 0;
            DebugPrintf("[WARN] WiFi probe returned OK but no active AP\r\n");
            goto cleanup;
        }
        g_wifi_connected = 1;
    } else {
        g_wifi_connected = 0;
        DebugPrintf("[WARN] WiFi state probe failed\r\n");
        goto cleanup;
    }
    
    // 步骤 0: 清理任何现存的 TCP 连接，避免连接状态冲突
    // 先禁用透传模式（如果之前未正确退出），再关闭连接
    DebugPrintf("[Step 0] Cleaning up previous connections...\r\n");
    
    // 明确禁用透传模式
    ClearRxBuffer();
    ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    vTaskDelay(pdMS_TO_TICKS(200));  // 增加延迟
    
    // 尝试关闭任何打开的连接（可能失败，但没关系）
    ClearRxBuffer();
    if (ESP8266_SendCmd("AT+CIPCLOSE\r\n", "CLOSED", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE) != 0) {
        ESP8266_SendCmd("AT+CIPCLOSE\r\n", "ERROR", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    }
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
            goto cleanup;
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
        goto cleanup;
    }
    DebugPrintf("[OK] Transparent mode enabled\r\n");
    
    // 步骤 3: AT+CIPSEND - 进入发送模式
    DebugPrintf("[Step 3] Entering transparent send mode (CIPSEND)...\r\n");
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = ESP8266_SendCmd("AT+CIPSEND\r\n", ">", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[FAIL] CIPSEND failed\r\n");
        goto cleanup;
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
        goto cleanup;
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
                goto cleanup;
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
    ret = 0;
    goto done;

cleanup:
    ESP8266_ExitTransparentMode();

    ClearRxBuffer();
    ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
    ClearRxBuffer();
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);

done:
    return ret;
}

static int ESP8266_ParseServerTime(const char *buf, uint64_t *server_time_ms)
{
    const char *pos = NULL;
    if (buf == NULL || server_time_ms == NULL) {
        return -1;
    }

    pos = strstr(buf, "\"server_time\"");
    if (pos == NULL) {
        return -1;
    }

    pos = strchr(pos, ':');
    if (pos == NULL) {
        return -1;
    }
    pos++;

    while (*pos == ' ' || *pos == '\"' || *pos == '\'' || *pos == '\t') {
        pos++;
    }

    uint64_t value = 0;
    uint32_t digits = 0;
    while (*pos >= '0' && *pos <= '9' && digits < 18) {
        value = value * 10 + (uint64_t)(*pos - '0');
        pos++;
        digits++;
    }

    if (digits == 0) {
        return -1;
    }

    *server_time_ms = value;
    return 0;
}

static int ESP8266_SetRtcFromUnix(uint32_t utc_seconds)
{
    uint32_t seconds = utc_seconds + BEIJING_OFFSET_SECONDS;
    uint32_t days = seconds / 86400U;
    uint32_t sec_of_day = seconds % 86400U;

    uint32_t hour = sec_of_day / 3600U;
    uint32_t minute = (sec_of_day % 3600U) / 60U;
    uint32_t second = sec_of_day % 60U;

    int64_t z = (int64_t)days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int64_t doe = z - era * 146097;
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t y = yoe + era * 400;
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int64_t mp = (5 * doy + 2) / 153;
    int64_t d = doy - (153 * mp + 2) / 5 + 1;
    int64_t m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);

    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    time.Hours = (uint8_t)hour;
    time.Minutes = (uint8_t)minute;
    time.Seconds = (uint8_t)second;

    date.Year = (uint8_t)(y - 2000);
    date.Month = (uint8_t)m;
    date.Date = (uint8_t)d;
    date.WeekDay = (uint8_t)(((days + 4) % 7 + 1));

    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) {
        DebugPrintf("[ERR] HAL_RTC_SetTime failed\r\n");
        return -1;
    }
    if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
        DebugPrintf("[ERR] HAL_RTC_SetDate failed\r\n");
        return -1;
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, date.Year);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, date.Month);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, date.Date);

    return 0;
}

static void ESP8266_ExitTransparentMode(void)
{
    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(1000));
    HAL_UART_Transmit(&huart2, (uint8_t *)"+++", 3, 1000);
    DebugPrintf("[TX] +++\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ClearRxBuffer();
    ESP8266_SendCmd("AT\r\n", "OK", CMD_TIMEOUT_MS, esp8266_rx_buffer, RX_BUFFER_SIZE);
}

int ESP8266_InitLock(void)
{
    if (esp8266_mutex != NULL) {
        return 0;
    }

    const osMutexAttr_t attr = {
        .name = "ESP8266_Mutex"
    };
    esp8266_mutex = osMutexNew(&attr);
    if (esp8266_mutex == NULL) {
        DebugPrintf("[ERR] ESP8266 InitLock failed\r\n");
        return -1;
    }
    return 0;
}

static int ESP8266_MutexLock(uint32_t timeout_ms)
{
    if (esp8266_mutex == NULL) {
        return -1;
    }
    return (osMutexAcquire(esp8266_mutex, pdMS_TO_TICKS(timeout_ms)) == osOK) ? 0 : -1;
}

static void ESP8266_MutexUnlock(void)
{
    if (esp8266_mutex != NULL) {
        osMutexRelease(esp8266_mutex);
    }
}

int ESP8266_WaitResponse(const char *wait_str, uint32_t timeout_ms, char *rx_buf, uint16_t rx_size)
{
    if (wait_str == NULL || rx_buf == NULL) {
        DebugPrintf("[ERR] ESP8266_WaitResponse: invalid parameters\r\n");
        return -1;
    }

    if (ESP8266_MutexLock(2000) != 0) {
        DebugPrintf("[ERR] ESP8266_WaitResponse: failed to acquire mutex\r\n");
        return -1;
    }

    ClearRxBuffer();
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 DMA receive start failed\r\n");
        ESP8266_MutexUnlock();
        return -1;
    }

    uint32_t start_ticks = xTaskGetTickCount();
    uint32_t timeout_ticks = (timeout_ms / portTICK_PERIOD_MS);
    uint16_t last_rx_len = 0;

    while (xTaskGetTickCount() - start_ticks < timeout_ticks) {
        vTaskDelay(pdMS_TO_TICKS(50));

        uint16_t current_len = GetUart2DmaRxLen();
        if (current_len > last_rx_len) {
            taskENTER_CRITICAL();
            uint16_t copy_len = (current_len > rx_size - 1) ? (rx_size - 1) : current_len;
            memcpy(rx_buf, (const void *)g_uart2_rx_buffer, copy_len);
            taskEXIT_CRITICAL();
            rx_buf[copy_len] = '\0';
            last_rx_len = current_len;

            DebugPrintf("[RX-WAIT] %s\r\n", rx_buf);
            if (strstr(rx_buf, wait_str) != NULL) {
                HAL_UART_DMAStop(&huart2);
                ESP8266_MutexUnlock();
                return 0;
            }
        }
    }

    HAL_UART_DMAStop(&huart2);
    ESP8266_MutexUnlock();
    return -1;
}

int ESP8266_GetTime(void)
{
    int ret = -1;
    uint64_t server_time_ms = 0;

    DebugPrintf("\n========== Get Network Time ==========\r\n");

    ClearRxBuffer();
    if (ESP8266_SendCmd("AT+CWJAP?\r\n", "OK", 5000, esp8266_rx_buffer, RX_BUFFER_SIZE) == 0) {
        if (strstr(esp8266_rx_buffer, "No AP") != NULL || strstr(esp8266_rx_buffer, "+CWJAP:") == NULL) {
            g_wifi_connected = 0;
            DebugPrintf("[WARN] WiFi disconnected before time sync\r\n");
            goto cleanup_time;
        }
        g_wifi_connected = 1;
    } else {
        g_wifi_connected = 0;
        DebugPrintf("[WARN] WiFi probe failed before time sync\r\n");
        goto cleanup_time;
    }

    ESP8266_QuickTransportCleanup();

    DebugPrintf("[Step 1] Connecting to %s:80...\r\n", TIME_HOST);
    ret = ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"" TIME_HOST "\",80\r\n", "CONNECT", 6000, esp8266_rx_buffer, RX_BUFFER_SIZE);
    if (ret != 0) {
        DebugPrintf("[WARN] AT+CIPSTART did not return CONNECT, checking for OK...\r\n");
        ClearRxBuffer();
        vTaskDelay(pdMS_TO_TICKS(200));
        ret = ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"" TIME_HOST "\",80\r\n", "OK", 6000, esp8266_rx_buffer, RX_BUFFER_SIZE);
        if (ret != 0) {
            DebugPrintf("[FAIL] TCP connection to time server failed\r\n");
            goto cleanup_time;
        }
    }
    DebugPrintf("[OK] TCP connected\r\n");

    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", 10000, esp8266_rx_buffer, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[FAIL] CIPMODE=1 failed\r\n");
        goto cleanup_time;
    }
    DebugPrintf("[OK] Transparent mode enabled\r\n");

    ClearRxBuffer();
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ESP8266_SendCmd("AT+CIPSEND\r\n", ">", 10000, esp8266_rx_buffer, RX_BUFFER_SIZE) != 0) {
        DebugPrintf("[FAIL] CIPSEND failed\r\n");
        goto cleanup_time;
    }
    DebugPrintf("[OK] Ready to send network time request\r\n");

    HAL_UART_Transmit(&huart2, (uint8_t *)TIME_REQUEST_TEMPLATE, strlen(TIME_REQUEST_TEMPLATE), 1000);
    DebugPrintf("[TX] Time HTTP request sent\r\n");

    ClearRxBuffer();
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_uart2_rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        DebugPrintf("[ERR] UART2 DMA receive start failed for time response\r\n");
        goto cleanup_time;
    }

    uint32_t receive_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - receive_start < pdMS_TO_TICKS(10000)) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (strstr((const char *)g_uart2_rx_buffer, "server_time") != NULL) {
            break;
        }
    }

    uint16_t rx_len = GetUart2DmaRxLen();
    HAL_UART_DMAStop(&huart2);

    if (rx_len == 0) {
        DebugPrintf("[WARN] No time response received\r\n");
        goto cleanup_time;
    }
    if (rx_len >= RX_BUFFER_SIZE) {
        rx_len = RX_BUFFER_SIZE - 1;
    }
    memcpy(esp8266_rx_buffer, (const void *)g_uart2_rx_buffer, rx_len);
    esp8266_rx_buffer[rx_len] = '\0';
    DebugPrintf("[RX] Time response received (%d bytes)\r\n", rx_len);

    if (ESP8266_ParseServerTime(esp8266_rx_buffer, &server_time_ms) != 0) {
        DebugPrintf("[FAIL] server_time parse failed\r\n");
        goto cleanup_time;
    }

    uint32_t utc_seconds = (uint32_t)(server_time_ms / 1000ULL);
    if (ESP8266_SetRtcFromUnix(utc_seconds) != 0) {
        DebugPrintf("[FAIL] RTC update failed\r\n");
        goto cleanup_time;
    }

    DebugPrintf("[OK] Network time synchronized: %lu s + %lu ms\r\n",
                (unsigned long)(server_time_ms / 1000ULL),
                (unsigned long)(server_time_ms % 1000ULL));
    ret = 0;

cleanup_time:
    ESP8266_ExitTransparentMode();
    ESP8266_QuickTransportCleanup();

    return ret;
}

/**
 * @brief UART2 接收中断回调（需要在 HAL 中调用）
 * 
 * 此函数应该在 UART2 的 RX 中断或 DMA 完成回调中调用，
 * 用于累积接收的数据到 g_uart2_rx_buffer
 * 
 * @param data 接收到的数据字节
 */
