/**
 ******************************************************************************
 * @file    dds_interface.c
 * @brief   DDS 抽象接口实现——封装 AD9959 CH3。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#include "dds_interface.h"
#include "AD9959.h"
#include "ui.h"

#define DDS_CHANNEL 3u

/**
 * @brief   初始化 DDS（调用 AD9959 Init）。
 * @retval  无。
 */
void dds_init(void)
{
    Init_AD9959();
    ui_log("[DDS] init\r\n");
}

/**
 * @brief   设置输出频率。
 * @param   freq_hz 频率（Hz，1~500MHz）。
 * @retval  无。
 */
void dds_set_freq(uint32_t freq_hz)
{
    Write_frequency(DDS_CHANNEL, freq_hz);
}

/**
 * @brief   设置幅度寄存器原始值。
 * @param   reg 幅度寄存器值（0~1023，对应约 0~4.9Vpp）。
 * @retval  无。
 */
void dds_set_amp_raw(uint16_t reg)
{
    if (reg > 1023u)
    {
        reg = 1023u;
    }
    Write_Amplitude(DDS_CHANNEL, reg);
}

/**
 * @brief   停止输出（幅度置 0）。
 * @retval  无。
 */
void dds_stop(void)
{
    Write_Amplitude(DDS_CHANNEL, 0);
    ui_log("[DDS] stop\r\n");
}
