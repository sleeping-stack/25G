/**
 ******************************************************************************
 * @file    dds_interface.h
 * @brief   DDS 抽象接口——封装 AD9959 CH3 频率/幅度/启停控制。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#ifndef __DDS_INTERFACE_H
#define __DDS_INTERFACE_H

#include <stdint.h>

/** DDS 幅度寄存器→Vpp 标定：Vpp ≈ 4.9*reg/1023 */
#define DDS_VPP_PER_REG (4.9f / 1023.0f)

/**
 * @brief   初始化 DDS（调用 AD9959 Init）。
 * @retval  无。
 */
void dds_init(void);

/**
 * @brief   设置输出频率。
 * @param   freq_hz 频率（Hz，1~500MHz）。
 * @retval  无。
 */
void dds_set_freq(uint32_t freq_hz);

/**
 * @brief   设置幅度寄存器原始值。
 * @param   reg 幅度寄存器值（0~1023，对应约 0~4.9Vpp）。
 * @retval  无。
 */
void dds_set_amp_raw(uint16_t reg);

/**
 * @brief   停止输出（幅度置 0）。
 * @retval  无。
 */
void dds_stop(void);

#endif /* __DDS_INTERFACE_H */
