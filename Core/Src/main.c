/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : TDPS 底盘控制核心 - 完整实现版
  * @author         : Yuxuan Shi
  * @description    : 实现基于 72MHz 主控的小车、8路传感器循线、TB6612 电机驱动，
  *                   以及基于串口的指令控制逻辑（对接香橙派上位机）。
  *
  * ===================== 通信协议说明 =====================
  * 【上位机 → STM32】帧格式：  $<CMD>[<PARAM>]#<XOR>\n
  *   - $        : 帧头
  *   - CMD      : 单字符指令（见下表）
  *   - PARAM    : 可选参数，整数字符串（仅 V 指令使用）
  *   - #        : 参数结束符
  *   - XOR      : 帧头到#之间所有字节的异或校验值（2位十六进制ASCII）
  *   - \n       : 帧尾
  *
  *   指令表：
  *     T  - 切换到自动循线模式
  *     S  - 停止并切回手动模式
  *     F  - 手动前进
  *     B  - 手动后退
  *     L  - 手动原地左转
  *     R  - 手动原地右转
  *     V<speed> - 设置基础速度，如 $V600#XX\n
  *     H  - 心跳包（上位机每隔一段时间发一次，防止超时停车）
  *
  * 【STM32 → 上位机】状态上报（每 200ms 一次）：
  *   格式：  $S<mode>,<lspd>,<rspd>,<sensor>,<error>#<XOR>\n
  *   示例：  $S1,450,-200,0b00011000,0.50#XX\n
  *     - mode   : 当前模式（0=手动，1=循线）
  *     - lspd   : 左轮当前速度
  *     - rspd   : 右轮当前速度
  *     - sensor : 传感器原始值（十六进制）
  *     - error  : 当前循线误差
  * ========================================================
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "sensor.h"
#include "encoder.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>  // atoi
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t rx_data;       // 串口中断接收缓冲，每次接收 1 字节
uint8_t work_mode = 0; // 工作模式：0=手动，1=自动循线

// PD 控制器状态
static float last_error = 0.0f;

// 当前速度记录（用于状态上报）
static int current_left_speed  = 0;
static int current_right_speed = 0;

// 基础巡线速度（可由上位机通过 V 指令动态修改）
static int base_speed = 450;

// -------- 帧解析状态机 --------
#define RX_BUF_SIZE 32
static uint8_t  rx_buf[RX_BUF_SIZE]; // 帧缓冲区
static uint8_t  rx_idx = 0;          // 当前写入位置
static uint8_t  in_frame = 0;        // 是否正在接收帧

// -------- 心跳超时保护 --------
// 上位机必须在 HEARTBEAT_TIMEOUT_MS 内发送任意指令（包括 H 心跳包），否则自动停车
#define HEARTBEAT_TIMEOUT_MS 500
static uint32_t last_rx_tick = 0;    // 上次收到有效帧的时间戳（HAL_GetTick）

// -------- 状态上报定时 --------
#define STATUS_REPORT_MS 200
static uint32_t last_report_tick = 0; // 上次上报时间戳
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static uint8_t  calc_xor(const uint8_t *data, uint8_t len);
static void     process_frame(const uint8_t *frame, uint8_t len);
static void     send_status(void);
/* USER CODE END PFP */

/* ---------------------------------------------------------------------------*/
/* 主程序入口 */
/* ---------------------------------------------------------------------------*/
int main(void)
{
  /* 硬件基础初始化 */
  HAL_Init();
  SystemClock_Config();

  /* 初始化所有外设 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  char clk_msg[128];

snprintf(clk_msg, sizeof(clk_msg),
    "SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\r\n",
     HAL_RCC_GetSysClockFreq(),
     HAL_RCC_GetHCLKFreq(),
     HAL_RCC_GetPCLK1Freq(),
     HAL_RCC_GetPCLK2Freq());

HAL_UART_Transmit(&huart1, (uint8_t*)clk_msg, strlen(clk_msg), 1000);

  // 记录启动时间，防止上电后立即触发超时停车
  last_rx_tick = HAL_GetTick();

  char *start_msg = ">>> TDPS AGV: System Ready (Manual Mode)\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)start_msg, strlen(start_msg), 100);

  // 开启串口中断接收
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END 2 */

  /* 主循环 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    // -------- 心跳超时检测 --------
    // 若超过 HEARTBEAT_TIMEOUT_MS 没收到任何帧，认为上位机断连，强制停车
    if ((now - last_rx_tick) > HEARTBEAT_TIMEOUT_MS)
    {
        if (work_mode != 0 || current_left_speed != 0 || current_right_speed != 0)
        {
            work_mode = 0;
            Motor_Stop();
            current_left_speed  = 0;
            current_right_speed = 0;
            HAL_UART_Transmit(&huart1, (uint8_t*)"!TIMEOUT\r\n", 10, 10);
        }
    }

    // -------- 自动循线模式 --------
    if (work_mode == 1)
    {
        float error   = Get_Tracking_Error();
        float Kp      = 120.0f;
        float Kd      = 80.0f;
        float d_error = error - last_error;
        last_error    = error;

        // 弯道自适应速度：误差越大说明弯越急，自动降速
        // base_speed 是香橙派设定的上限，实际速度在此基础上根据误差缩减
        int adaptive_speed;
        float abs_error = error < 0 ? -error : error;
        if (abs_error < 1.0f) {
            // 直线或微弯，全速
            adaptive_speed = base_speed;
        } else if (abs_error < 2.5f) {
            // 中等弯道，降到 70%
            adaptive_speed = (int)(base_speed * 0.7f);
        } else {
            // 急弯，降到 45%
            adaptive_speed = (int)(base_speed * 0.45f);
        }

        int correction = (int)(Kp * error + Kd * d_error);
        int lspd = adaptive_speed + correction;
        int rspd = adaptive_speed - correction;

        if(lspd >  999) lspd =  999;
        if(lspd < -999) lspd = -999;
        if(rspd >  999) rspd =  999;
        if(rspd < -999) rspd = -999;

        Motor_SetSpeed(lspd, rspd);
        current_left_speed  = lspd;
        current_right_speed = rspd;

        Encoder_Update();
        HAL_Delay(20);
    }
    // -------- 手动模式/空闲 --------
    else
    {
        Encoder_Update();
        HAL_Delay(20);
    }

    // -------- 定时状态上报 --------
    // 每 STATUS_REPORT_MS 向香橙派发送一次完整状态帧
    if ((HAL_GetTick() - last_report_tick) >= STATUS_REPORT_MS)
    {
        last_report_tick = HAL_GetTick();
        send_status();
    }
    /* USER CODE END WHILE */
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 计算异或校验值
 * @param data 数据起始指针
 * @param len  数据长度
 * @return 所有字节异或结果
 */
static uint8_t calc_xor(const uint8_t *data, uint8_t len)
{
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/**
 * @brief 解析并执行一帧完整指令
 * @param frame 帧内容（不含帧头$和帧尾\n，包含CMD、PARAM、#、XOR）
 * @param len   帧长度
 *
 * 帧格式示例（去掉$和\n后）：T#XX  或  V600#XX
 * 校验范围：从$到#（含#）之间的所有字节
 */
static void process_frame(const uint8_t *frame, uint8_t len)
{
    // 最短帧：CMD + '#' + 2位XOR = 4字节
    if (len < 4) return;

    // 找到 '#' 的位置
    int hash_pos = -1;
    for (int i = 0; i < len; i++) {
        if (frame[i] == '#') {
            hash_pos = i;
            break;
        }
    }
    if (hash_pos < 0 || hash_pos + 2 >= len) return;

    // 解析校验值（2位十六进制ASCII → uint8_t）
    char xor_str[3] = {frame[hash_pos + 1], frame[hash_pos + 2], '\0'};
    uint8_t recv_xor = (uint8_t)strtol(xor_str, NULL, 16);

    // 重新计算校验：范围是 '$' + frame[0..hash_pos]
    // 因为帧头$已经被状态机消耗，这里手动把$加进去
    uint8_t calc = '$';
    for (int i = 0; i <= hash_pos; i++) {
        calc ^= frame[i];
    }

    // 校验不通过，丢弃该帧
    if (calc != recv_xor) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"!CRCERR\r\n", 9, 10);
        return;
    }

    // 更新心跳时间戳（任何有效帧都刷新）
    last_rx_tick = HAL_GetTick();

    // 解析指令字符和可选参数
    uint8_t cmd = frame[0];
    int param = 0;
    if (hash_pos > 1) {
        // CMD 后面跟着参数字符串，转成整数
        char param_str[16] = {0};
        int plen = hash_pos - 1;
        if (plen >= (int)sizeof(param_str)) plen = sizeof(param_str) - 1;
        memcpy(param_str, &frame[1], plen);
        param = atoi(param_str);
    }

    // 执行指令
    switch (cmd)
    {
        case 'T': // 切换到自动循线模式
            work_mode  = 1;
            last_error = 0.0f;
            break;

        case 'S': // 停止并切回手动模式
            work_mode = 0;
            Motor_Stop();
            current_left_speed  = 0;
            current_right_speed = 0;
            break;

        case 'F': // 手动前进
            if (work_mode == 0) {
                Motor_SetSpeed(base_speed, base_speed);
                current_left_speed  = base_speed;
                current_right_speed = base_speed;
            }
            break;

        case 'B': // 手动后退
            if (work_mode == 0) {
                Motor_SetSpeed(-base_speed, -base_speed);
                current_left_speed  = -base_speed;
                current_right_speed = -base_speed;
            }
            break;

        case 'L': // 手动原地左转
            if (work_mode == 0) {
                Motor_SetSpeed(-base_speed, base_speed);
                current_left_speed  = -base_speed;
                current_right_speed =  base_speed;
            }
            break;

        case 'R': // 手动原地右转
            if (work_mode == 0) {
                Motor_SetSpeed(base_speed, -base_speed);
                current_left_speed  =  base_speed;
                current_right_speed = -base_speed;
            }
            break;

        case 'V': // 动态设置基础速度，参数范围 0-999
            if (param > 0 && param <= 999) {
                base_speed = param;
            }
            break;

        case 'H': // 心跳包，仅刷新时间戳，不做其他操作
            break;

        default:
            break;
    }

    // 回执 ACK
    HAL_UART_Transmit(&huart1, (uint8_t*)"ACK\r\n", 5, 10);
}

/**
 * @brief 向香橙派发送一帧完整状态信息
 *
 * 格式：$S<mode>,<lspd>,<rspd>,<sensor_hex>,<error>#<XOR>\r\n
 * 示例：$S1,450,-200,18,0.50#3A\r\n
 */
static void send_status(void)
{
    uint8_t  sensor_raw = Read_8Way_Sensor();
    float    error      = Get_Tracking_Error();
    int32_t  enc_l      = Encoder_GetLeft();
    int32_t  enc_r      = Encoder_GetRight();

    char body[64];
    int body_len = snprintf(body, sizeof(body),
        "S%d,%d,%d,%02X,%.2f,%ld,%ld#",
        work_mode,
        current_left_speed,
        current_right_speed,
        sensor_raw,
        error,
        enc_l,
        enc_r
    );

    // 计算校验：'$' 异或 body 所有字节（含末尾的#）
    uint8_t xor_val = '$';
    for (int i = 0; i < body_len; i++) {
        xor_val ^= (uint8_t)body[i];
    }

    // 拼接完整帧
    char frame[64];
    int frame_len = snprintf(frame, sizeof(frame), "$%s%02X\r\n", body, xor_val);

    HAL_UART_Transmit(&huart1, (uint8_t*)frame, frame_len, 20);
}

/**
 * @brief 串口接收完成回调（中断触发）
 * @note  每次收到 1 字节就进入这里，用状态机拼装完整帧
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t byte = rx_data;

    if (byte == '$') {
        // 收到帧头，重置状态机，开始接收新帧
        rx_idx   = 0;
        in_frame = 1;
    }
    else if (in_frame)
    {
        if (byte == '\n') {
            // 收到帧尾，交给 process_frame 处理
            in_frame = 0;
            process_frame(rx_buf, rx_idx);
            rx_idx = 0;
        }
        else if (byte == '\r') {
            // 忽略 \r，兼容 \r\n 结尾的串口助手
        }
        else {
            // 普通数据字节，写入缓冲区
            if (rx_idx < RX_BUF_SIZE - 1) {
                rx_buf[rx_idx++] = byte;
            } else {
                // 缓冲区溢出，丢弃当前帧
                in_frame = 0;
                rx_idx   = 0;
            }
        }
    }

    // 重新挂载中断，继续监听下一字节
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}
/* USER CODE END 4 */

/**
  * @brief 系统时钟配置，HSE 8MHz → PLL → 72MHz (F411CEU6)
  *
  * PLL 参数推导：
  *   VCO 输入 = HSE / PLLM = 8MHz / 8 = 1MHz
  *   VCO 输出 = VCO输入 × PLLN = 1MHz × 144 = 144MHz
  *   SYSCLK   = VCO输出 / PLLP = 144MHz / 2 = 72MHz
  *   USB时钟  = VCO输出 / PLLQ = 144MHz / 3 = 48MHz（USB 固定需要 48MHz）
  *
  * 总线时钟：
  *   AHB  (HCLK)  = SYSCLK / 1 = 72MHz
  *   APB1 (PCLK1) = HCLK   / 2 = 36MHz（APB1 最大 50MHz，满足）
  *   APB2 (PCLK2) = HCLK   / 1 = 72MHz
  *
  * Flash 延迟：72MHz 时需要 2 个等待周期（FLASH_LATENCY_2），
  * 等待周期不够会导致 CPU 读到错误指令，是常见的上电死机原因。
  *
  * 电压档位：SCALE2（1.2V 内核电压）支持最高 84MHz，72MHz 完全够用，
  * 比 SCALE1 省电。
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;              // HSE 8MHz / 8 = 1MHz VCO 输入（板载 8MHz 晶振）
    RCC_OscInitStruct.PLL.PLLN       = 144;             // 1MHz × 144 = 144MHz VCO
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;  // 144 / 2 = 72MHz SYSCLK
    RCC_OscInitStruct.PLL.PLLQ       = 3;              // 144 / 3 = 48MHz USB
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;  // HCLK  = 72MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // APB1  = 36MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // APB2  = 72MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief 错误处理
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* 死循环，可通过调试器查看状态 */ }
}
