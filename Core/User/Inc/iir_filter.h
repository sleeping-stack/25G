/**
  ******************************************************************************
  * @file    iir_filter.h
  * @brief   2 阶 IIR biquad 滤波器——封装 CMSIS-DSP arm_biquad_cascade_df2T_f32。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __IIR_FILTER_H
#define __IIR_FILTER_H

#include <stdint.h>

/**
  * @brief   装载 biquad 系数。
  * @param   b 分子系数 [b0, b1, b2]。
  * @param   a 分母系数 [1, a1, a2]（a[0] 需为 1.0f）。
  * @retval  无。
  * @note    传递函数 H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)。
  */
void iir_set_coeffs(const float b[3], const float a[3]);

/**
  * @brief   逐块处理（浮点）。
  * @param   in  输入缓冲。
  * @param   out 输出缓冲。
  * @param   n   样本数。
  * @retval  无。
  */
void iir_process_block(const float *in, float *out, uint32_t n);

#endif /* __IIR_FILTER_H */
