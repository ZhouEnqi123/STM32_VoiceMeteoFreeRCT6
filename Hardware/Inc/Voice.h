/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Voice.h
  * @brief   Voice module for SU03T command receive and MY1680 audio play
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __VOICE_H__
#define __VOICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

#define VOICE_CMD_WAKEUP  0x00
#define VOICE_CMD_TEMP    0x01
#define VOICE_CMD_HUMI    0x02

void Voice_Init(void);
void Voice_PlayStartup(void);
void Voice_UpdateSensorData(float temperature, float humidity);
bool Voice_WaitForPlaybackComplete(uint32_t timeout_ms);
bool Voice_IsPlaying(void);
void Voice_TaskLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOICE_H__ */
