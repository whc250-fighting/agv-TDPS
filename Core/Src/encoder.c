/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   霍尔编码器测速
  *          PB6=左A（中断），PB7=左B（方向判断输入）
  *          PB8=右A（中断），PB9=右B（方向判断输入）
  *          A相双边沿触发中断，B相读电平判断方向
  *          每圈计数 = 20脉冲 × 双边沿 = 40次/圈（电机轴）
  ******************************************************************************
  */
/* USER CODE END Header */
#include "encoder.h"

/* 原始脉冲计数，在中断里累加，有符号 */
static volatile int32_t left_count  = 0;
static volatile int32_t right_count = 0;

/* Update 后的结果，供外部读取 */
static int32_t left_speed  = 0;
static int32_t right_speed = 0;

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6 PB8：A相，双边沿中断触发计数 */
    GPIO_InitStruct.Pin  = GPIO_PIN_6 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB7 PB9：B相，普通输入，仅用于方向判断 */
    GPIO_InitStruct.Pin  = GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Encoder_Update(void)
{
    /* 原子读取并清零计数器 */
    __disable_irq();
    left_speed  = left_count;
    right_speed = right_count;
    left_count  = 0;
    right_count = 0;
    __enable_irq();
}

int32_t Encoder_GetLeft(void)  { return left_speed;  }
int32_t Encoder_GetRight(void) { return right_speed; }

/*
 * EXTI9_5_IRQHandler：处理 PB6/PB7/PB8/PB9 的外部中断
 * 方向判断逻辑（以左轮为例）：
 *   A相触发中断时，读B相电平：
 *     A上升且B低 → 正转；A上升且B高 → 反转
 *     A下降且B高 → 正转；A下降且B低 → 反转
 *   B相触发中断时，读A相电平，逻辑相反
 * 这样单相 20 脉冲/圈 × 双边沿 × AB两相 = 80 计数/圈
 * 但为简化保持与规格书一致，这里只用A相单边沿，方向由B相判断
 */
void EXTI9_5_IRQHandler(void)
{
    /* 左轮 A 相（PB6）触发，读 B 相（PB7）判断方向 */
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6))
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
            left_count++;
        else
            left_count--;
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    }

    /* 右轮 A 相（PB8）触发，读 B 相（PB9）判断方向 */
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8))
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
            right_count--;
        else
            right_count++;
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    }
}
