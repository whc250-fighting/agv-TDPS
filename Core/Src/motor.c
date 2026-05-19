#include "motor.h"
#include "tim.h"

/**
 * @brief  电机初始化：启动 TIM2 的两路 PWM 输出
 *
 * MX_TIM2_Init() 只是配置了定时器的参数，并不会自动输出波形。
 * 必须调用 HAL_TIM_PWM_Start() 才能让对应引脚开始输出 PWM 信号。
 *
 * 通道映射（见 tim.c 引脚配置）：
 *   TIM2_CH1 → PA0 → 左轮 PWM（PWMA）
 *   TIM2_CH2 → PA1 → 右轮 PWM（PWMB）
 */
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 左轮 PWM 开始输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 右轮 PWM 开始输出
}

/**
 * @brief  设置左右轮速度
 * @param  left_speed   左轮速度，范围 -999 到 999
 * @param  right_speed  右轮速度，范围 -999 到 999
 *                      正值 = 前转，负值 = 后转，0 = 停止
 *
 * TB6612 控制逻辑（以左轮 Motor A 为例）：
 *   STBY = 1（使能芯片）
 *   AIN1=1, AIN2=0, PWMA = speed        → 正转
 *   AIN1=0, AIN2=1, PWMA = |speed|      → 反转（PWM 值必须为正，取绝对值）
 *   AIN1=0, AIN2=0, PWMA = 0            → 自由停止（惯性滑行）
 *
 * __HAL_TIM_SET_COMPARE 是直接写 CCR 寄存器的宏，
 * 定时器硬件会自动把 CCR 值转换成对应占空比的 PWM 波形，CPU 不参与后续输出。
 *
 * 引脚对应（见 main.h 宏定义）：
 *   PA2 = AIN1,  PA3 = AIN2（左轮方向）
 *   PA4 = BIN1,  PA5 = BIN2（右轮方向）
 *   PA6 = STBY（驱动芯片使能）
 */
void Motor_SetSpeed(int left_speed, int right_speed) {

    // STBY 拉高，TB6612 进入工作状态
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // -------- 左轮控制（Motor A） --------
    if (left_speed > 0) {
        // 正转：AIN1=1, AIN2=0
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
    }
    else if (left_speed < 0) {
        // 反转：AIN1=0, AIN2=1，PWM 值取绝对值（CCR 寄存器不能写负数）
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -left_speed);
    }
    else {
        // 停止：AIN1=0, AIN2=0，PWM=0，电机自由滑行
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

    // -------- 右轮控制（Motor B） --------
    if (right_speed > 0) {
        // 正转：BIN1=1, BIN2=0
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_speed);
    }
    else if (right_speed < 0) {
        // 反转：BIN1=0, BIN2=1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -right_speed);
    }
    else {
        // 停止
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

/**
 * @brief  紧急停车
 *
 * 将 STBY 引脚拉低，TB6612 进入低功耗待机状态，
 * 两路电机输出同时断开，比单纯把 PWM 设为 0 更彻底。
 * 适用于心跳超时、收到 S 指令等需要立即停车的场景。
 */
void Motor_Stop(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // STBY 拉低，芯片进入待机
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);      // 左轮 PWM 清零
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // 右轮 PWM 清零
}
