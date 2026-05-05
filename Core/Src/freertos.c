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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    float humidity;
} WeatherData;

#define EVENT_SENSOR_READY  (1 << 0) // 0x01: 传感器数据更新
#define EVENT_CLOCK_TICK    (1 << 1) // 0x02: 时间跳动
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
  .stack_size = 512 * 4,
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
  /* Infinite loop */
  vTaskSuspend(SensorTaskHandle);
  vTaskSuspend(DisplayTaskHandle);

  osDelay(100);
  OLED_Init();
  AHT20_Init();

  osTimerStart(ClockTimerHandle, 1000U);
  
  vTaskResume(SensorTaskHandle);
  vTaskResume(DisplayTaskHandle);

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
  /* Infinite loop */
  WeatherData display_data = {0};
  char message[32];

  static uint8_t last_saved_day = 0;
  for(;;)
  {
    uint32_t flags = osEventFlagsWait(DisplayEventsHandle, 
                             EVENT_SENSOR_READY | EVENT_CLOCK_TICK, 
                             osFlagsWaitAny, osWaitForever);

    // 如果是传感器发来的，去队列里拿一下最新数据
    if (flags & EVENT_SENSOR_READY) {
        osMessageQueueGet(SensorQueueHandle, &display_data, NULL, 0);
    }

    if (Date.Date != last_saved_day) 
    {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, Date.Year);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, Date.Month);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, Date.Date);
        last_saved_day = Date.Date; // 更新记录
    }

    // 无论哪个事件触发，都统一重新绘制一帧
    OLED_NewFrame();
    
    // --- 顶部：显示时间 ---
    sprintf(message, "%04d-%02d-%02d", 2000 + Date.Year, Date.Month, Date.Date);
    OLED_PrintString(24, 0, message, &font16x16, OLED_COLOR_NORMAL);
    sprintf(message, "%02d:%02d:%02d", Time.Hours, Time.Minutes, Time.Seconds);
    OLED_PrintASCIIString(15, 17, message, &afont24x12, OLED_COLOR_NORMAL);


    // --- 下部：显示温湿度 ---
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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

/* USER CODE END Application */

