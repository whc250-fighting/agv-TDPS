#include "sensor.h"

// 记录上一次有效误差，用于丢线时保持转向惯性
static float last_valid_error = 0.0f;

/**
 * @brief  通过 CD4051 多路复用器轮询读取 8 路红外传感器
 *
 * 硬件原理：
 *   CD4051 是一个 8选1 开关芯片。
 *   3根地址线 AD0/AD1/AD2 的二进制组合（000~111）决定选中哪路传感器，
 *   被选中的传感器信号出现在 OUT 引脚上。
 *   因此必须循环 8 次，每次切换地址 → 等待稳定 → 读一次 OUT。
 *
 * 引脚对应：
 *   AD0 = PB12,  AD1 = PB13,  AD2 = PB14
 *   OUT = PB15
 *
 * 传感器逻辑：
 *   检测到黑线 → OUT 输出高电平 → 对应 bit 置 1
 *   检测到白底 → OUT 输出低电平 → 对应 bit 保持 0
 *
 * @return 8位无符号整数，bit0 对应传感器0，bit7 对应传感器7
 *         例如 0b00011000 = 传感器3和4亮，说明黑线在正中间
 */
uint8_t Read_8Way_Sensor(void) {
    uint8_t final_data = 0;

    for (int i = 0; i < 8; i++) {

        // 第一步：把循环变量 i（0~7）的三个二进制位分别写到地址线
        // i & 0x01 取 bit0 → AD0（权重1）
        // i & 0x02 取 bit1 → AD1（权重2）
        // i & 0x04 取 bit2 → AD2（权重4）
        // 三根线的组合就是 i 的二进制表示，选中第 i 路传感器
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (i & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (i & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // 第二步：硬件延时，等待 CD4051 内部模拟开关切换完成并稳定
        // 72MHz 下每次 NOP 约 14ns，循环 200 次 ≈ 2.8μs，足够 CD4051 稳定
        for (volatile int delay = 0; delay < 200; delay++) { __NOP(); }

        // 第三步：读取 OUT 引脚，若为高电平说明第 i 路传感器检测到黑线
        // 用移位把结果写入 final_data 的第 i 位
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) {
            final_data |= (1 << i);
        }
    }

    return final_data;
}

/**
 * @brief  计算黑线相对于小车中心的偏移误差（加权平均算法）
 *
 * 算法原理：
 *   8 个传感器横排在小车底部，从左到右编号 0~7，中心在 3.5 号位置。
 *   给每个传感器分配一个位置权重，中心为 0，左负右正：
 *
 *     传感器编号：  0     1     2     3     4     5     6     7
 *     位置权重：  -3.5  -2.5  -1.5  -0.5  +0.5  +1.5  +2.5  +3.5
 *
 *   误差 = 所有亮起传感器的权重之和 / 亮起传感器数量（加权平均）
 *
 *   结果含义：
 *     误差 = 0    → 黑线在正中间，直行
 *     误差 < 0    → 黑线偏左，需要左转（左轮减速或右轮加速）
 *     误差 > 0    → 黑线偏右，需要右转（右轮减速或左轮加速）
 *
 *   多个传感器同时亮（黑线压在两个传感器之间）时，
 *   加权平均会给出介于两者之间的平滑值，避免误差跳变。
 *
 * 丢线处理：
 *   若 8 路全灭（raw == 0），说明小车已经偏离黑线。
 *   此时返回上一次的有效误差，让小车保持最后的转向方向继续寻线，
 *   而不是误差归零导致小车走直线越跑越偏。
 *
 * @return 误差值，范围 -3.5 到 +3.5，0 为正中间
 */
float Get_Tracking_Error(void) {
    uint8_t raw = Read_8Way_Sensor();

    // 丢线保护：全灭时返回上次误差，维持转向惯性
    if (raw == 0x00) {
        return last_valid_error;
    }

    // 位置权重表，步长均匀为 1，中心对称，正中间两个传感器同时亮时均值恰好为 0
    float weights[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};

    float sum_weight  = 0.0f;
    int   active_count = 0;

    for (int i = 0; i < 8; i++) {
        if ((raw >> i) & 0x01) {       // 检查第 i 位是否为 1（该传感器是否亮起）
            sum_weight   += weights[i];
            active_count++;
        }
    }

    float current_error = sum_weight / active_count; // 加权平均
    last_valid_error = current_error;                // 更新丢线保护记忆

    return current_error;
}
