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
#include <string.h>

/* Voice audio indexes (对应 MY1680 存储音频顺序) */
#define V_STARTUP      0x0001
#define V_PREFIX_TEMP  0x0002
#define V_PREFIX_HUMI  0x0003
#define V_UNIT_TEN     0x0004
#define V_TEN_BASE     0x0005
#define V_NUM_BASE     0x000D
#define V_NUM_0        0x0016
#define V_UNIT_CEL     0x0017
#define V_UNIT_PER     0x0018
#define V_REPLY        0x0019
#define V_POINT        0x001A

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
        HAL_Delay(800);
    } else {
        int tens = num / 10;
        int units = num % 10;
        if (tens == 1) {
            Voice_Send_Index(V_UNIT_TEN);
        } else {
            Voice_Send_Index(V_TEN_BASE + (tens - 2));
        }
        HAL_Delay(800);
        if (units != 0) {
            Voice_Send_Index(V_NUM_BASE + (units - 1));
            HAL_Delay(800);
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
    HAL_Delay(1200);
    Voice_Play_Number(integer_part);
    Voice_Send_Index(V_POINT);
    HAL_Delay(800);
    Voice_Play_Number(decimal_part);
    Voice_Send_Index(V_UNIT_CEL);
    HAL_Delay(800);
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
    HAL_Delay(1200);
    Voice_Send_Index(V_UNIT_PER);
    HAL_Delay(1000);
    Voice_Play_Number(integer_part);
    Voice_Send_Index(V_POINT);
    HAL_Delay(800);
    Voice_Play_Number(decimal_part);
}

static void Voice_SetVolume(uint8_t volume)
{
    uint8_t cmd[6] = {0x7E, 0x04, 0x31, 0x0F, 0x3A, 0xEF};
    if (volume <= 15) {
        cmd[3] = 0x30 | (volume & 0x0F);
        cmd[4] = cmd[1] ^ cmd[2] ^ cmd[3];
    }
    HAL_UART_Transmit(&huart4, cmd, sizeof(cmd), 100);
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
    voice_is_playing = true;
    voice_play_start_tick = HAL_GetTick();
    Voice_Send_Index(V_STARTUP);
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

    Voice_SetVolume(15);
    HAL_Delay(200);
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
