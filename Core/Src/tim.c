/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

/*
 * ===================== PWM 原理复习 =====================
 *
 * PWM（脉冲宽度调制）是用数字信号模拟模拟量的技术。
 * 定时器以固定频率产生方波，通过改变高电平占比（占空比）来控制等效电压：
 *   占空比 50% → 等效 VCC × 50%
 *   占空比 80% → 等效 VCC × 80%
 * TB6612 内部有 H 桥，PWM 占空比直接决定电机转速。
 *
 * ===================== 定时器计数原理 =====================
 *
 * STM32 定时器本质是一个计数器，每个时钟周期加 1，
 * 计到 Period（ARR 寄存器）后归零，产生一次溢出（Update Event）。
 *
 *   计数频率 = TIM 时钟 / (Prescaler + 1)
 *   PWM 频率 = 计数频率 / (Period + 1)
 *
 * ===================== 本项目参数计算 =====================
 *
 * TIM2 时钟来源：
 *   SYSCLK = 72MHz（HSE 8MHz × PLL）
 *   APB1   = SYSCLK / 2 = 36MHz
 *   TIM2 时钟 = APB1 × 2 = 72MHz
 *   （当 APB1 分频系数 ≠ 1 时，TIM 时钟自动 ×2，这是 STM32 的固定规则）
 *
 * Prescaler = 71：
 *   计数时钟 = 72MHz / (71 + 1) = 72MHz / 72 = 1MHz
 *   即每 1μs 计数器加 1
 *
 * Period = 999：
 *   PWM 周期 = (999 + 1) 个计数 = 1000μs = 1ms
 *   PWM 频率 = 1kHz
 *   1kHz 对 TB6612 来说是合适的频率，太低会听到电机嗡嗡声，太高会增加开关损耗
 *
 * 占空比控制（CCR 寄存器）：
 *   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value)
 *   当计数器 < value 时输出高电平，≥ value 时输出低电平（PWM1 模式）
 *   value = 0   → 占空比 0%，电机停
 *   value = 500 → 占空比 50%，半速
 *   value = 999 → 占空比 99.9%，全速
 *   value 范围 0~999，与 Period 对应
 *
 * ===================== 通道映射 =====================
 *
 *   TIM2_CH1 → PA0（AF1 复用）→ TB6612 PWMA → 左轮速度
 *   TIM2_CH2 → PA1（AF1 复用）→ TB6612 PWMB → 右轮速度
 *
 *   AF1 是 F411 PA0/PA1 的 TIM2 复用功能编号，
 *   必须在 GPIO 初始化时通过 .Alternate = GPIO_AF1_TIM2 显式指定，
 *   否则引脚不会输出 PWM 波形（F1 系列靠 AFIO 寄存器，F4 改成了每引脚独立 AF 选择）。
 */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;             // 72MHz / 72 = 1MHz 计数时钟
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;               // 1MHz / 1000 = 1kHz PWM 频率
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // PWM1 模式：计数器 < CCR 时输出高，≥ CCR 时输出低
  // Pulse = 0 表示初始占空比为 0（电机不转）
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 高电平有效
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)  // 左轮
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)  // 右轮
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  // MspPostInit 负责把 PA0/PA1 配置成 AF1 复用推挽输出，让引脚真正输出 PWM 波形
  HAL_TIM_MspPostInit(&htim2);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;  // F4 必须显式指定复用功能
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
