/**
 ******************************************************************************
 * @file    app_state.c
 * @brief   应用状态机实现——IDLE/LEARNING/REPLAY + 陶晶驰串口屏帧解析。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#include "app_state.h"
#include "ui.h"
#include "dds_interface.h"
#include "adc_sampling.h"
#include "dac_output.h"
#include "iir_filter.h"
#include "sysid.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include <math.h>

typedef enum
{
    APP_IDLE = 0,
    APP_LEARNING,
    APP_REPLAY
} app_state_t;

static app_state_t app_state = APP_IDLE;

/* 辨识结果（建模后存，实时用） */
static float iir_b[3] = {1.0f, 0.0f, 0.0f};
static float iir_a[3] = {1.0f, 0.0f, 0.0f};

/* ---- 串口屏帧接收（9 字节固定长度帧） ---- */
/** 帧头标识：0x31=page1 基础(3,4)按H(s)反推，0x32=学习，0x33=复现切换，0x34=page0 基础(2)满幅输出 */
#define FRAME_LEN 9u
static uint8_t rx_byte;
static uint8_t rx_frame[FRAME_LEN];
static uint8_t rx_count = 0;

/** 待执行命令类型（中断置位，poll 清零执行） */
typedef enum
{
    CMD_NONE = 0,
    CMD_PAGE1_SET, /* page1 基础(3,4): 按H(s)反推DDS幅度使已知电路输出达设定Vpp */
    CMD_PAGE0_MAX, /* page0 基础(2): 设置频率，DDS满幅输出 */
    CMD_LEARN, /* 学习建模 */
    CMD_REPLAY_TOGGLE /* 复现/停止切换 */
} cmd_type_t;

static volatile cmd_type_t cmd_ready = CMD_NONE;
static uint32_t cmd_freq = 0; /* 频率（Hz） */
static uint32_t cmd_amp_div10 = 0; /* page1: 已知电路目标输出Vpp×10 */

/* 实时块处理缓冲 */
#define RT_BLOCK (ADC_RT_BUF_LEN / 2)
static float rt_in[RT_BLOCK];
static float rt_out[RT_BLOCK];

/**
 * @brief   实时块处理：ADC→去偏移归一化→IIR→DAC 格式化。
 *          去直流用硬编码 ADC_OFFSET_CODE（实时 256 点块均值含信号低频，
 *          不适合自适应；IIR 对直流偏移不敏感，硬编码足够）。
 * @param   adc_buf ADC 数据。
 * @param   len     样本数。
 * @param   dac_buf DAC 输出缓冲。
 * @retval  无。
 */
static void rt_process(uint16_t *adc_buf, uint32_t len, uint16_t *dac_buf)
{
    for (uint32_t i = 0; i < len; ++i)
    {
        rt_in[i] = ((float)adc_buf[i] - ADC_OFFSET_CODE) / 32768.0f;
    }
    iir_process_block(rt_in, rt_out, len);
    for (uint32_t i = 0; i < len; ++i)
    {
        float v = rt_out[i] * 2048.0f + 2048.0f;
        if (v < 0.0f)
            v = 0.0f;
        else if (v > 4095.0f)
            v = 4095.0f;
        dac_buf[i] = (uint16_t)v;
    }
}

/**
 * @brief   实时 half-transfer 回调。
 * @param   buf ADC 前半缓冲。
 * @param   len 样本数。
 * @retval  无。
 */
static void rt_ht_cb(uint16_t *buf, uint32_t len)
{
    rt_process(buf, len, &dac_rt_buf[0]);
}

/**
 * @brief   实时 transfer-complete 回调。
 * @param   buf ADC 后半缓冲。
 * @param   len 样本数。
 * @retval  无。
 */
static void rt_tc_cb(uint16_t *buf, uint32_t len)
{
    rt_process(buf, len, &dac_rt_buf[RT_BLOCK]);
}

/**
 * @brief   执行建模（若在 replay 先停止）。
 * @retval  无。
 */
static void app_exec_learn(void)
{
    if (app_state == APP_REPLAY)
    {
        adc_stop_rt();
        dac_rt_stop();
    }
    app_state = APP_LEARNING;
    ui_show_status("learning...");
    filter_type_t t = sysid_run(iir_b, iir_a);
    (void)t;
    app_state = APP_IDLE;
    ui_show_status("idle");
}

/**
 * @brief   执行实时复现。
 * @retval  无。
 */
static void app_exec_start(void)
{
    if (app_state != APP_IDLE)
    {
        ui_show_status("busy");
        return;
    }
    iir_set_coeffs(iir_b, iir_a);
    dac_rt_init();
    dac_rt_start();
    adc_rt_set_callbacks(rt_ht_cb, rt_tc_cb);
    adc_start_rt();
    app_state = APP_REPLAY;
    ui_show_status("replay");
}

/**
 * @brief   停止实时复现。
 * @retval  无。
 */
static void app_exec_stop(void)
{
    if (app_state != APP_REPLAY)
    {
        ui_show_status("idle");
        return;
    }
    adc_stop_rt();
    dac_rt_stop();
    app_state = APP_IDLE;
    ui_show_status("idle");
}

/**
 * @brief   已知模型电路 H(s)=5/(1e-8·s²+3e-4·s+1) 的幅频响应。
 * @param   freq_hz 频率（Hz）。
 * @retval  |H(jω)|，ω=2πf。
 * @note    |H(jω)| = 5/sqrt((1-1e-8·ω²)²+(3e-4·ω)²)。
 *          直流增益 5，过阻尼二阶低通（ζ=1.5, ωn=1e4 rad/s）。
 */
static float known_h_mag(float freq_hz)
{
    const float pi = 3.14159265358979f;
    float w = 2.0f * pi * freq_hz;
    float re = 1.0f - 1e-8f * w * w;
    float im = 3e-4f * w;
    return 5.0f / sqrtf(re * re + im * im);
}

/**
 * @brief   page1 基础(3,4): 按 H(s) 反推 DDS 幅度，使已知电路输出达设定 Vpp。
 * @param   freq_hz     频率（Hz）。
 * @param   amp_div10   已知电路目标输出峰峰值×10（Vpp = amp_div10/10）。
 * @retval  无。
 * @note    DDS_Vpp = target_vpp / |H(jω)|，再由 sysid_calibrate_dds 闭环校准 DDS 实际输出
 *          到该值（补偿 DDS 非平坦频响）。非 IDLE 状态拒绝执行；DDS 保持输出（不 stop）。
 */
static void app_exec_page1_set(uint32_t freq_hz, uint32_t amp_div10)
{
    if (app_state != APP_IDLE)
    {
        ui_show_status("busy");
        return;
    }
    float target_vpp = (float)amp_div10 / 10.0f;
    float h_mag = known_h_mag((float)freq_hz);
    float dds_vpp = target_vpp / h_mag;
    /* 闭环校准 DDS 实际输出到 dds_vpp（补偿 DDS 非平坦响应），DDS 保持输出 */
    sysid_calibrate_dds(freq_hz, dds_vpp);
}

/**
 * @brief   page0 基础(2): 设置频率，DDS 满幅输出（reg=1023）。
 * @param   freq_hz 频率（Hz）。
 * @retval  无。
 * @note    非 IDLE 状态拒绝执行；DDS 保持输出。
 */
static void app_exec_page0_max(uint32_t freq_hz)
{
    if (app_state != APP_IDLE)
    {
        ui_show_status("busy");
        return;
    }
    dds_set_freq(freq_hz);
    dds_set_amp_raw(1023u);
}

/**
 * @brief   串口接收回调（帧状态机：等帧头→收 8 字节体→置命令标志）。
 * @param   huart UART 句柄。
 * @retval  无。
 * @note    帧格式：0x31[freq 4B LE][目标Vpp×10 4B LE] / 0x34[freq 4B LE][4×忽略] / 0x32|0x33[8×'0']。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    if (rx_count == 0)
    {
        /* 等帧头 */
        if (rx_byte == 0x31u || rx_byte == 0x32u || rx_byte == 0x33u || rx_byte == 0x34u)
        {
            rx_frame[0] = rx_byte;
            rx_count = 1;
        }
    }
    else
    {
        rx_frame[rx_count++] = rx_byte;
        if (rx_count >= FRAME_LEN)
        {
            /* 完整帧，解析 */
            if (rx_frame[0] == 0x31u)
            {
                memcpy(&cmd_freq, &rx_frame[1], 4);
                memcpy(&cmd_amp_div10, &rx_frame[5], 4);
                cmd_ready = CMD_PAGE1_SET;
            }
            else if (rx_frame[0] == 0x34u)
            {
                memcpy(&cmd_freq, &rx_frame[1], 4);
                cmd_ready = CMD_PAGE0_MAX;
            }
            else if (rx_frame[0] == 0x32u)
            {
                cmd_ready = CMD_LEARN;
            }
            else if (rx_frame[0] == 0x33u)
            {
                cmd_ready = CMD_REPLAY_TOGGLE;
            }
            rx_count = 0;
        }
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/**
 * @brief   初始化应用（UI/DDS/DAC/ADC 校准/串口接收）。
 * @retval  无。
 */
void app_init(void)
{
    ui_init();
    dds_init();
    dac_rt_init();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    rx_count = 0;
    cmd_ready = CMD_NONE;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    app_state = APP_IDLE;
    ui_show_status("idle");
}

/**
 * @brief   主循环轮询（执行中断中排队的命令）。
 * @retval  无。
 */
void app_poll(void)
{
    cmd_type_t cmd = cmd_ready;
    if (cmd == CMD_NONE)
    {
        return;
    }
    cmd_ready = CMD_NONE;

    switch (cmd)
    {
    case CMD_PAGE1_SET:
        app_exec_page1_set(cmd_freq, cmd_amp_div10);
        break;
    case CMD_PAGE0_MAX:
        app_exec_page0_max(cmd_freq);
        break;
    case CMD_LEARN:
        app_exec_learn();
        break;
    case CMD_REPLAY_TOGGLE:
        if (app_state == APP_REPLAY)
        {
            app_exec_stop();
        }
        else
        {
            app_exec_start();
        }
        break;
    default:
        break;
    }
}
