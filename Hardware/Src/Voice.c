/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Voice.c
  * @brief   Voice driver for SU03T receive and MY1680 playback
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Voice.h"
#include "usart.h"
#include "rtc.h"
#include <string.h>

/* Voice audio indexes (对应 MY1680 存储音频顺序) */
#define V_STARTUP      0x0001
#define V_PREFIX_TEMP  0x0002
#define V_PREFIX_HUMI  0x0003
#define V_PREFIX_TIME   0x001B
#define V_UNIT_MIN      0x001C
#define V_ALARM         0x001D
#define V_UNIT_TEN     0x0004
#define V_TEN_BASE     0x0005
#define V_NUM_BASE     0x000D
#define V_NUM_0        0x0016
#define V_UNIT_CEL     0x0017
#define V_UNIT_PER     0x0018
#define V_REPLY        0x0019
#define V_POINT        0x001A

#define VOICE_DEFAULT_VOLUME 8U

/* Receiver buffers */
static uint8_t aRxBuffer5 = 0;
static uint8_t aRxBuffer4 = 0;
static uint8_t fb_buf[4] = {0};

/* Playback and command state */
static volatile bool voice_is_playing = false;
static volatile uint32_t voice_play_end_tick = 0;
static volatile uint32_t voice_play_start_tick = 0;
static osMessageQueueId_t voice_queue = NULL;

static float voice_temperature = 25.6f;
static float voice_humidity = 72.3f;
static uint8_t voice_volume_level = VOICE_DEFAULT_VOLUME;

static void Voice_Send_Index(uint16_t index)
{
    uint8_t cmd[7];
    cmd[0] = 0x7E;
    cmd[1] = 0x05;
    cmd[2] = 0x41;
    cmd[3] = (uint8_t)((index >> 8) & 0xFF);
    cmd[4] = (uint8_t)(index & 0xFF);
    cmd[5] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4];
    cmd[6] = 0xEF;
    HAL_UART_Transmit(&huart4, cmd, sizeof(cmd), 100);
}

static void Voice_Play_Number(int num)
{
    if (num < 0) {
        num = 0;
    }

    if (num < 10) {
        if (num == 0) {
            Voice_Send_Index(V_NUM_0);
        } else {
            Voice_Send_Index(V_NUM_BASE + (num - 1));
        }
        osDelay(800);
    } else {
        int tens = num / 10;
        int units = num % 10;
        if (tens == 1) {
            Voice_Send_Index(V_UNIT_TEN);
        } else {
            Voice_Send_Index(V_TEN_BASE + (tens - 2));
        }
        osDelay(800);
        if (units != 0) {
            Voice_Send_Index(V_NUM_BASE + (units - 1));
            osDelay(800);
        }
    }
}

static void Voice_Report_Temp(void)
{
    int integer_part = (int)voice_temperature;
    int decimal_part = (int)((voice_temperature - (float)integer_part) * 10.0f + 0.5f);
    if (decimal_part >= 10) {
        integer_part += 1;
        decimal_part = 0;
    }

    Voice_Send_Index(V_PREFIX_TEMP);
    osDelay(1200);
    Voice_Play_Number(integer_part);
    Voice_Send_Index(V_POINT);
    osDelay(800);
    Voice_Play_Number(decimal_part);
    Voice_Send_Index(V_UNIT_CEL);
    osDelay(800);
}

static void Voice_Report_Humi(void)
{
    int integer_part = (int)voice_humidity;
    int decimal_part = (int)((voice_humidity - (float)integer_part) * 10.0f + 0.5f);
    if (decimal_part >= 10) {
        integer_part += 1;
        decimal_part = 0;
    }

    Voice_Send_Index(V_PREFIX_HUMI);
    osDelay(1200);
    Voice_Send_Index(V_UNIT_PER);
    osDelay(1000);
    Voice_Play_Number(integer_part);
    Voice_Send_Index(V_POINT);
    osDelay(800);
    Voice_Play_Number(decimal_part);
}

void Voice_SetVolumeLevel(uint8_t volume)
{
    uint8_t cmd[6] = {0x7E, 0x04, 0x31, 0x0F, 0x3A, 0xEF};
    if (volume <= 15) {
        voice_volume_level = volume;
        cmd[3] = 0x30 | (volume & 0x0F);
        cmd[4] = cmd[1] ^ cmd[2] ^ cmd[3];
    }
    HAL_UART_Transmit(&huart4, cmd, sizeof(cmd), 100);
}

void Voice_PlayAlarm(void)
{
    if (voice_queue != NULL) {
        uint8_t cmd = VOICE_CMD_ALARM;
        osMessageQueuePut(voice_queue, &cmd, 0, 0);
    } else {
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Send_Index(V_ALARM);
    }
}

static void Voice_Report_Time(void)
{
    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef rtc_date = {0};

    if (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) {
        return;
    }

    Voice_Send_Index(V_PREFIX_TIME);
    osDelay(1200);
    Voice_Play_Number(rtc_time.Hours);
    Voice_Send_Index(V_POINT);
    osDelay(800);
    if (rtc_time.Minutes < 10) {
        Voice_Send_Index(V_NUM_0);
        osDelay(800);
    }
    Voice_Play_Number(rtc_time.Minutes);
    Voice_Send_Index(V_UNIT_MIN);
    osDelay(800);
}

void Voice_UpdateSensorData(float temperature, float humidity)
{
    voice_temperature = temperature;
    voice_humidity = humidity;
}

bool Voice_IsPlaying(void)
{
    return voice_is_playing;
}

bool Voice_WaitForPlaybackComplete(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (voice_is_playing) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
        osDelay(20);
    }
    return true;
}

void Voice_PlayStartup(void)
{
    if (voice_queue != NULL) {
        uint8_t cmd = VOICE_CMD_STARTUP;
        osMessageQueuePut(voice_queue, &cmd, 0, 0);
    } else {
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Send_Index(V_STARTUP);
    }
}

void Voice_Init(void)
{
    if (voice_queue == NULL) {
        voice_queue = osMessageQueueNew(8, sizeof(uint8_t), NULL);
    }

    voice_is_playing = false;
    voice_play_end_tick = HAL_GetTick();
    voice_play_start_tick = 0;
    memset(fb_buf, 0, sizeof(fb_buf));

    Voice_SetVolumeLevel(VOICE_DEFAULT_VOLUME);
    osDelay(200);
    HAL_UART_Receive_IT(&huart5, &aRxBuffer5, 1);
    HAL_UART_Receive_IT(&huart4, &aRxBuffer4, 1);
}

void Voice_ProcessCommand(uint8_t cmd)
{
    if (voice_is_playing) {
        return;
    }

    switch (cmd) {
    case VOICE_CMD_WAKEUP:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Send_Index(V_REPLY);
        break;
    case VOICE_CMD_TEMP:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Report_Temp();
        break;
    case VOICE_CMD_HUMI:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Report_Humi();
        break;
    case VOICE_CMD_STARTUP:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Send_Index(V_STARTUP);
        break;
    case VOICE_CMD_TIME:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Report_Time();
        break;
    case VOICE_CMD_ALARM:
        voice_is_playing = true;
        voice_play_start_tick = HAL_GetTick();
        Voice_Send_Index(V_ALARM);
        break;
    default:
        break;
    }
}

void Voice_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        if (!voice_is_playing && (HAL_GetTick() - voice_play_end_tick > 1000U)) {
            if (voice_queue != NULL) {
                osMessageQueuePut(voice_queue, &aRxBuffer5, 0, 0);
            }
        }
        HAL_UART_Receive_IT(&huart5, &aRxBuffer5, 1);
    } else if (huart->Instance == UART4) {
        fb_buf[0] = fb_buf[1];
        fb_buf[1] = fb_buf[2];
        fb_buf[2] = fb_buf[3];
        fb_buf[3] = aRxBuffer4;

        if (fb_buf[0] == 'S' && fb_buf[1] == 'T' && fb_buf[2] == 'O' && fb_buf[3] == 'P') {
            voice_is_playing = false;
            voice_play_end_tick = HAL_GetTick();
        }
        HAL_UART_Receive_IT(&huart4, &aRxBuffer4, 1);
    }
}

void Voice_TaskLoop(void)
{
    while (voice_queue == NULL) {
        osDelay(1);
    }

    uint8_t cmd;
    for (;;) {
        if (osMessageQueueGet(voice_queue, &cmd, NULL, osWaitForever) == osOK) {
            Voice_ProcessCommand(cmd);
            uint32_t wait_start = HAL_GetTick();
            while (voice_is_playing && (HAL_GetTick() - wait_start < 15000U)) {
                osDelay(20);
            }
            voice_is_playing = false;
            voice_play_end_tick = HAL_GetTick();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Voice_IRQHandler(huart);
}
