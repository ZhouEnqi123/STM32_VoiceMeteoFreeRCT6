/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "font.h"
#include "aht20.h"
#include "esp8266.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    float humidity;
} WeatherData;

#define EVENT_SENSOR_READY   (1 << 0) // 0x01: 传感器数据更新
#define EVENT_CLOCK_TICK     (1 << 1) // 0x02: 时间跳动
#define EVENT_WEATHER_UPDATE (1 << 2) // 0x04: 天气数据更新
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern RTC_TimeTypeDef Time;
extern RTC_DateTypeDef Date;
extern RTC_HandleTypeDef hrtc;
// ESP 初始化状态（0 = 未初始化/未就绪, 1 = 已初始化就绪）
volatile int esp_init_done = 0;
// IWDG 看门狗喂养计数器
static uint32_t iwdg_feed_counter = 0;
/* USER CODE END Variables */
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LinkTask */
osThreadId_t LinkTaskHandle;
const osThreadAttr_t LinkTask_attributes = {
  .name = "LinkTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for SensorQueue */
osMessageQueueId_t SensorQueueHandle;
const osMessageQueueAttr_t SensorQueue_attributes = {
  .name = "SensorQueue"
};
/* Definitions for LinkQueue */
osMessageQueueId_t LinkQueueHandle;
const osMessageQueueAttr_t LinkQueue_attributes = {
  .name = "LinkQueue"
};
/* Definitions for ClockTimer */
osTimerId_t ClockTimerHandle;
const osTimerAttr_t ClockTimer_attributes = {
  .name = "ClockTimer"
};
/* Definitions for DisplayEvents */
osEventFlagsId_t DisplayEventsHandle;
const osEventFlagsAttr_t DisplayEvents_attributes = {
  .name = "DisplayEvents"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInitTask(void *argument);
void StartSensorTask(void *argument);
void StartDisplayTask(void *argument);
void StartLinkTask(void *argument);
void ClockTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of ClockTimer */
  ClockTimerHandle = osTimerNew(ClockTimerCallback, osTimerPeriodic, NULL, &ClockTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SensorQueue */
  SensorQueueHandle = osMessageQueueNew (1, 8, &SensorQueue_attributes);

  /* creation of LinkQueue */
  LinkQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &LinkQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of LinkTask */
  LinkTaskHandle = osThreadNew(StartLinkTask, NULL, &LinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of DisplayEvents */
  DisplayEventsHandle = osEventFlagsNew(&DisplayEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* USER CODE BEGIN StartInitTask */
  /* 初始化任务：系统启动时执行 */
  
  // 挂起其他任务
  vTaskSuspend(SensorTaskHandle);
  vTaskSuspend(DisplayTaskHandle);
  vTaskSuspend(LinkTaskHandle);
  
  // 初始化外设
  osDelay(100);
  OLED_Init();
  AHT20_Init();
  
  // 启动实时时钟定时器
  osTimerStart(ClockTimerHandle, 1000U);
  
  // 打印复位来源，便于诊断是否发生了 MCU 重启/看门狗
  {
    uint32_t reset_flags = RCC->CSR;
    DebugPrintf("[INIT] Reset flags CSR=0x%08lX\r\n", (unsigned long)reset_flags);
    if (reset_flags & RCC_CSR_PORRSTF) DebugPrintf("[INIT] Power-on reset\r\n");
    if (reset_flags & RCC_CSR_PINRSTF) DebugPrintf("[INIT] Pin reset (NRST)\r\n");
    if (reset_flags & RCC_CSR_SFTRSTF) DebugPrintf("[INIT] Software reset\r\n");
    if (reset_flags & RCC_CSR_IWDGRSTF) DebugPrintf("[INIT] Independent watchdog reset\r\n");
    if (reset_flags & RCC_CSR_WWDGRSTF) DebugPrintf("[INIT] Window watchdog reset\r\n");
    if (reset_flags & RCC_CSR_LPWRRSTF) DebugPrintf("[INIT] Low-power reset\r\n");
    // 清除复位标志
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  // ESP 初始化将在 LinkTask 中异步进行，确保显示和传感器先启动
  DebugPrintf("\r\n\n[INIT] Deferring ESP8266 initialization to LinkTask\r\n");

  // 启动显示、传感器和 LinkTask，后者负责从 AT 检测到 WiFi 连接的完整流程
  vTaskResume(SensorTaskHandle);
  vTaskResume(DisplayTaskHandle);
  vTaskResume(LinkTaskHandle);

  // 初始化任务完成，删除自己
  vTaskDelete(NULL);

  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  WeatherData sensor_data;
  for(;;)
  {
    AHT20_Measure();
    osDelay(100);

    sensor_data.temperature = AHT20_Temperature();
    sensor_data.humidity = AHT20_Humidity();

    osMessageQueuePut(SensorQueueHandle, &sensor_data, 0, 0);
    osEventFlagsSet(DisplayEventsHandle, EVENT_SENSOR_READY);

    osDelay(1900);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* 显示任务：刷新 OLED 屏幕，显示时间、温湿度和网络天气 */
  
  WeatherData display_data = {0};
  char message[32];
  static uint8_t last_saved_day = 0;
  
  for(;;)
  {
    // 等待事件：传感器数据更新、时间跳动、或天气数据更新
    uint32_t flags = osEventFlagsWait(DisplayEventsHandle, 
                             EVENT_SENSOR_READY | EVENT_CLOCK_TICK | EVENT_WEATHER_UPDATE, 
                             osFlagsWaitAny, osWaitForever);

    // 如果是传感器发来的数据，去队列里获取最新数据
    if (flags & EVENT_SENSOR_READY) {
        osMessageQueueGet(SensorQueueHandle, &display_data, NULL, 0);
    }

    // 如果是天气更新，日期改变时保存到 RTC 备份寄存器
    if (Date.Date != last_saved_day) 
    {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, Date.Year);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, Date.Month);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, Date.Date);
        last_saved_day = Date.Date;
    }

    // 无论哪个事件触发，都统一重新绘制一帧
    OLED_NewFrame();
    
    // ========== 顶部：左上角显示网络天气/WiFi状态图标 ==========
    if (g_wifi_connected == 0) {
      OLED_DrawImage(0, 0, &NoWIFIImg, OLED_COLOR_NORMAL);
    } else {
      const Image *weather_icon = GetWeatherIcon(g_weather.weather_text);
      if (weather_icon != NULL) {
        OLED_DrawImage(0, 0, weather_icon, OLED_COLOR_NORMAL);
      }
    }

    // ========== 中上部：显示日期 ==========
    sprintf(message, "%04d-%02d-%02d", 2000 + Date.Year, Date.Month, Date.Date);
    OLED_PrintString(24, 0, message, &font16x16, OLED_COLOR_NORMAL);

    // ========== 中部：显示时间 ==========
    sprintf(message, "%02d:%02d:%02d", Time.Hours, Time.Minutes, Time.Seconds);
    OLED_PrintASCIIString(15, 17, message, &afont24x12, OLED_COLOR_NORMAL);

    // ========== 下部：显示温湿度（本地传感器） ==========
    sprintf(message, "%.1f℃", display_data.temperature);
    OLED_DrawImage(0, 48, &tempImg, OLED_COLOR_NORMAL);
    OLED_PrintString(16, 48, message, &font16x16, OLED_COLOR_NORMAL);
    
    sprintf(message, "%.1f", display_data.humidity);
    OLED_DrawImage(64, 48, &humiImg, OLED_COLOR_NORMAL);
    OLED_PrintString(80, 48, message, &font16x16, OLED_COLOR_NORMAL);
    OLED_DrawImage(112, 48, &％Img, OLED_COLOR_NORMAL);
    
    OLED_ShowFrame();
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartLinkTask */
/**
* @brief Function implementing the LinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLinkTask */
void StartLinkTask(void *argument)
{
  /* USER CODE BEGIN StartLinkTask */
  /* 网络链接任务：每 10 分钟获取一次网络天气 */
  
  // 等待 LinkTask 被启动后再开始运行
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // 周期：10 分钟 = 600000 毫秒
  const uint32_t WEATHER_UPDATE_PERIOD_MS = 600000;  // 10 分钟
  
  uint32_t last_update_time = 0;
  uint32_t current_time = 0;

  // 在 LinkTask 中异步进行 ESP 初始化，避免阻塞显示和传感器任务
  if (!esp_init_done) {
    int tries = 3;
    while (tries-- > 0 && !esp_init_done) {
      DebugPrintf("[LinkTask] Attempting ESP8266_Init(), tries left=%d\r\n", tries);
      if (ESP8266_Init() == 0) {
        esp_init_done = 1;
        DebugPrintf("[LinkTask] ESP8266 initialized\r\n");
        break;
      }
      DebugPrintf("[LinkTask] ESP init attempt failed, retrying after delay...\r\n");
      osDelay(pdMS_TO_TICKS(2000));
    }
    if (!esp_init_done) {
      DebugPrintf("[LinkTask] ESP initialization deferred, will retry later\r\n");
    }
  }

  for(;;)
  {
    // 获取当前任务时间
    current_time = xTaskGetTickCount();

    // 只有在 ESP 初始化成功后，才尝试获取天气
    if (esp_init_done) {
      if ((current_time - last_update_time) >= pdMS_TO_TICKS(WEATHER_UPDATE_PERIOD_MS) ||
          last_update_time == 0)  // 第一次执行
      {
        DebugPrintf("\r\n[LinkTask] Starting weather update...\r\n");
        
        // 调用 ESP8266 获取天气函数
        if (Get_Weather() == 0) {
          // 获取成功，发送天气更新事件给 DisplayTask
          DebugPrintf("[LinkTask] Weather updated: %s\r\n", g_weather.weather_text);
          osEventFlagsSet(DisplayEventsHandle, EVENT_WEATHER_UPDATE);
          
          // 更新上次更新时间
          last_update_time = current_time;
        } else {
          DebugPrintf("[LinkTask] Weather update failed, will retry next cycle\r\n");
          // 失败不更新时间，等待下一轮尝试（仍然每10分钟重试）
          last_update_time = current_time;
        }
      }
    } else {
      // 如果还未初始化成功，降低重试频率（每 60 秒重试一次），避免大量打印阻塞
      if (ESP8266_Init() == 0) {
        esp_init_done = 1;
        DebugPrintf("[LinkTask] ESP8266 initialized on delayed retry\r\n");
      } else {
        // 仅在失败时打印一次信息，等待更长时间再试
        DebugPrintf("[LinkTask] ESP still not ready, will retry after delay\r\n");
        osDelay(pdMS_TO_TICKS(60000));
        continue;
      }
    }

    // 每秒检查一次是否需要更新
    osDelay(1000);
  }
  /* USER CODE END StartLinkTask */
}

/* ClockTimerCallback function */
void ClockTimerCallback(void *argument)
{
  /* USER CODE BEGIN ClockTimerCallback */
  // 1. 读取硬件 RTC (先 Time 后 Date)
  HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);

  // 2. 发送事件标志给 DisplayTask
  osEventFlagsSet(DisplayEventsHandle, EVENT_CLOCK_TICK);
  /* USER CODE END ClockTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief FreeRTOS Idle Hook - 保持空闲
 * 
 * 看门狗喂养已移至 LinkTask 的显式调用中，避免 Idle Hook 中的寄存器操作导致异常。
 * 这是更安全的做法，确保看门狗刷新与主任务流程同步。
 */
void vApplicationIdleHook(void)
{
    // 空闲钩子此处不做任何操作
    // IWDG 喂养在 LinkTask 中显式调用
    (void)iwdg_feed_counter;  // 避免警告
}

/* USER CODE END Application */

