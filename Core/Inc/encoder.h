/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   霍尔编码器测速模块
  *          左轮：PB6(A相) PB7(B相)，右轮：PB8(A相) PB9(B相)
  *          上升沿 + 下降沿双边计数，每圈 40 脉冲，减速比 1:20
  *          轮子每转一圈对应 800 个中断脉冲
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * Encoder_Init:
 *   配置 PB6/PB7/PB8/PB9 为双边沿外部中断输入，启动计数。
 *   必须在 MX_GPIO_Init 之后调用。
 *
 * Encoder_Update:
 *   在主循环每 20ms 调用一次，从中断计数器计算左右轮转速。
 *   结果单位：脉冲/20ms，上层可按需换算成 r/s 或 mm/s。
 *
 * Encoder_GetLeft / Encoder_GetRight:
 *   获取上次 Update 计算的左/右轮脉冲计数（带符号，正=前转，负=后转）。
 */
void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
