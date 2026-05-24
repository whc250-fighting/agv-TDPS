#ifndef __SENSOR_H
#define __SENSOR_H

/* main.h 包含了所有 HAL 头文件和引脚宏定义，这里只需引入它即可 */
#include "main.h"

/*
 * Read_8Way_Sensor:
 *   通过 CD4051 多路复用器轮询 8 路红外传感器，返回 8 位原始值。
 *   bit0 对应传感器0（最左），bit7 对应传感器7（最右）。
 *   检测到黑线 = 1，白底 = 0。
 *
 * Get_Tracking_Error:
 *   对原始传感器值做加权平均，计算黑线偏离中心的误差。
 *   返回值范围 -3.5（最左）到 +3.5（最右），0 为正中间。
 *   全灭（丢线）时返回上次有效误差，维持转向惯性。
 */
uint8_t Read_8Way_Sensor(void);
float   Get_Tracking_Error(void);

#endif
