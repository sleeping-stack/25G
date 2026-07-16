/**
  ******************************************************************************
  * @file    sysid.h
  * @brief   系统辨识——扫频 + FFT + SK 迭代 QR 求解 biquad + 滤波类型判定。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __SYSID_H
#define __SYSID_H

#include <stdint.h>

/** 滤波类型枚举 */
typedef enum
{
    FILTER_TYPE_UNKNOWN = 0,
    FILTER_TYPE_LOWPASS,
    FILTER_TYPE_HIGHPASS,
    FILTER_TYPE_BANDPASS,
    FILTER_TYPE_BANDSTOP
} filter_type_t;

/**
  * @brief   执行建模：扫频 → FFT 取 H(jω) → SK 迭代 QR 辨识 biquad → 判定类型。
  * @param   b_out 输出分子系数 [b0,b1,b2]。
  * @param   a_out 输出分母系数 [1,a1,a2]（a_out[0] 设为 1.0f）。
  * @retval  滤波类型。
  */
filter_type_t sysid_run(float b_out[3], float a_out[3]);

/**
  * @brief   获取类型字符串。
  * @param   t 滤波类型枚举。
  * @retval  类型字符串。
  */
const char *sysid_type_str(filter_type_t t);

/**
  * @brief   DDS 幅度闭环校准：采样 ADC1(DDS 输出) 并迭代调整 reg 使实际 Vpp 达目标。
  *          用 RMS 时域法测量（100Hz~3kHz 全覆盖精确，不依赖 FFT bin 对齐）。
  *          校准后 DDS 保持输出（不 stop），最终 reg 留在收敛值。
  * @param   freq_hz    频率（Hz）。
  * @param   target_vpp 目标 DDS 输出峰峰值（V）。
  * @retval  无。
  * @note    独立于 sysid_run，可在未建模时调用。内部用 adc_start_dual_oneshot 采样。
  */
void sysid_calibrate_dds(uint32_t freq_hz, float target_vpp);

#endif /* __SYSID_H */
