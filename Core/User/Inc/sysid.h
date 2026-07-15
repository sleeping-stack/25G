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

#endif /* __SYSID_H */
