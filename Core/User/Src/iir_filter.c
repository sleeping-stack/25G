/**
  ******************************************************************************
  * @file    iir_filter.c
  * @brief   2 阶 IIR biquad 实现——封装 CMSIS-DSP arm_biquad_cascade_df2T_f32。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#include "iir_filter.h"
#include "arm_math.h"

static arm_biquad_cascade_df2T_instance_f32 iir_inst;
static float iir_coeffs[6];
static float iir_state[4];

/**
  * @brief   装载 biquad 系数。
  * @param   b 分子系数 [b0, b1, b2]。
  * @param   a 分母系数 [1, a1, a2]。
  * @retval  无。
  * @note    CMSIS-DSP DF2T 系数排列 {b0,b1,b2,-a1,-a2}，a0 归一化为 1。
  */
void iir_set_coeffs(const float b[3], const float a[3])
{
    iir_coeffs[0] = b[0];
    iir_coeffs[1] = b[1];
    iir_coeffs[2] = b[2];
    iir_coeffs[3] = -a[1];
    iir_coeffs[4] = -a[2];
    iir_coeffs[5] = 0.0f;
    arm_biquad_cascade_df2T_init_f32(&iir_inst, 1, iir_coeffs, iir_state);
}

/**
  * @brief   逐块处理（浮点）。
  * @param   in  输入缓冲。
  * @param   out 输出缓冲。
  * @param   n   样本数。
  * @retval  无。
  */
void iir_process_block(const float *in, float *out, uint32_t n)
{
    arm_biquad_cascade_df2T_f32(&iir_inst, in, out, n);
}
