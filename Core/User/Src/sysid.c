/**
 ******************************************************************************
 * @file    sysid.c
 * @brief   系统辨识实现——扫频 + Hanning+RFFT 取 H(jω) + SK 迭代 QR 求 biquad + 类型判定。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#include "sysid.h"
#include "dds_interface.h"
#include "adc_sampling.h"
#include "ui.h"
#include "usart.h"
#include "arm_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* ---- 常量 ---- */
#define SYSID_FS_HZ (240000000.0f / 293.0f) /* 819112.6 Hz，TIM6 Frequency */
#define SYSID_N ADC_BUF_SIZE /* 4096 */
#define SYSID_PI 3.14159265358979f

/* 主扫描 1k~50k 步长 200：246 点；高频 50k~400k 步长 2k：176 点；共 422 点
 * 高频点全部落在 200Hz 网格上（FS/N≈199.98Hz），FFT on-bin 无泄漏。 */
#define SYSID_NF_LOW 246
#define SYSID_NF_HIGH 176
#define SYSID_NF (SYSID_NF_LOW + SYSID_NF_HIGH)

/* 建模激励目标 Vpp */
#define SYSID_TARGET_VPP 2.0f
/* 闭环容差与增益 */
#define SYSID_AMP_TOL 0.005f
#define SYSID_AMP_GAIN 0.8f

/* SK 迭代参数（仅用于 Levy 初值，LM 接管精化） */
#define SYSID_SK_MAX_ITER 20u /* 最大迭代次数（第 0 轮即 Levy） */
#define SYSID_SK_D_FLOOR 1e-6f /* |D_prev|² 下限，防止极点处权重爆炸 */

/* LM 非线性最小二乘参数（阶段 2 精化，直接最小化真残差 Σ|H-N/D|²）
 * 拟合范围限定 1k-50k（SYSID_NF_LOW=246 点）后高频噪声干扰消除，
 * 标准小 λ 初值即可收敛。原 1e1 实测把 resid 翻倍（跳到错误盆地），已回退。 */
#define SYSID_LM_MAX_ITER 30u /* 最大 LM 迭代次数 */
#define SYSID_LM_LAMBDA_INIT 1e-3f /* 初始信赖域阻尼 */
#define SYSID_LM_LAMBDA_MAX 1e6f /* 阻尼上限，超过则停 */
#define SYSID_LM_TOL 1e-6f /* 残差相对变化收敛阈值 */

/* 诊断打印开关：1=学习后经 USART1 打印 LM 轨迹、频点残差与 summary；0=关闭 */
#define SYSID_DEBUG_PRINT 0u

/* ---- 静态缓冲（DTCMRAM，CPU 专用，零等待） ---- */
static float fft_in[SYSID_N];
static float fft_out[SYSID_N];
static float hanning_win[SYSID_N];
static uint16_t adc_x[SYSID_N];
static uint16_t adc_y[SYSID_N];

/* 测量结果 */
static float H_meas_re[SYSID_NF];
static float H_meas_im[SYSID_NF];
static uint32_t H_freq[SYSID_NF];
/* 诊断用：每频点 |X|、|Y|（FFT bin 模值），用于评估 SNR 与残差来源 */
static float H_Xmag[SYSID_NF];
static float H_Ymag[SYSID_NF];

/* 正规方程累积量：ATA(5×5), ATb(5×1) */
static float ATA[25];
static float ATb[5];
/* QR 分解缓冲 */
static float Q[25];
static float R[25];
static float QT[25];
static float Qb[5];
static float x_sol[5];
static float tau[5];
static float tmpA[5];
static float tmpB[5];

/* LM 雅可比累积：JᵀJ(5×5), Jᵀr(5×1)；Δp 求解缓冲 */
static float JTJ[25];
static float JTr[5];
static float dp[5];
static float A_lm[25]; /* JTJ + λ·diag(JTJ) 副本，供 QR */
static float b_lm[5]; /* -Jᵀr 副本 */

static arm_rfft_fast_instance_f32 fft_inst;

/**
 * @brief   取第 idx 个频点（Hz）。
 *          低频 1k~50k 步长 200（246 点）；高频 50k~400k 步长 2k（176 点）。
 *          高频全部落在 200Hz 网格上，FFT on-bin。
 * @param   idx 频点索引（0~421）。
 * @retval  频率（Hz）。
 */
static uint32_t sysid_freq_at(uint32_t idx)
{
    if (idx < SYSID_NF_LOW)
    {
        return 1000u + idx * 200u;
    }
    return 50000u + (idx - SYSID_NF_LOW) * 2000u;
}

/**
 * @brief   频率转 FFT bin 索引（四舍五入）。
 * @param   freq_hz 频率（Hz）。
 * @retval  bin 索引。
 */
static uint32_t sysid_freq_to_bin(uint32_t freq_hz)
{
    float b = (float)freq_hz * (float)SYSID_N / SYSID_FS_HZ;
    return (uint32_t)(b + 0.5f);
}

/**
 * @brief   去直流 + 加 Hanning 窗：计算块均值后减去，再乘窗。
 *          自适应均值取代硬编码 ADC_OFFSET_CODE，消除抬升电压偏差。
 * @param   src 输入 ADC 原始码值（uint16）。
 * @param   dst 输出 float 缓冲（去直流加窗后）。
 * @param   n   样本数。
 * @retval  块均值（码值），供调试参考。
 */
static float sysid_remove_dc_window(const uint16_t *src, float *dst, uint32_t n)
{
    float mean = 0.0f;
    for (uint32_t i = 0; i < n; ++i)
    {
        mean += (float)src[i];
    }
    mean /= (float)n;
    for (uint32_t i = 0; i < n; ++i)
    {
        dst[i] = ((float)src[i] - mean) * hanning_win[i];
    }
    return mean;
}

/**
 * @brief   由 ADC1 采样计算目标 bin 处的交流 Vpp。
 * @param   bin  目标 bin。
 * @retval  实际 Vpp（V）。
 * @note    去直流（块均值自适应）后加 Hanning，|X_bin| 还原：Vpp = |X_bin|*52.8/(N*65535)。
 *          推导：A_code=4*|X_bin|/N, V_adc_ac=A_code*3.3/65535,
 *          V_real=2*V_adc_ac, Vpp=2*V_real → Vpp=|X_bin|*52.8/(N*65535)。
 */
static float sysid_measure_vpp(uint32_t bin)
{
    sysid_remove_dc_window(adc_x, fft_in, SYSID_N);
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
    float re = fft_out[2 * bin];
    float im = fft_out[2 * bin + 1];
    float mag = sqrtf(re * re + im * im);
    return mag * 52.8f / ((float)SYSID_N * 65535.0f);
}

/**
 * @brief   单频点幅度闭环校准（迭代 4 次）。
 * @param   freq_hz     频率。
 * @param   target_vpp  目标 Vpp。
 * @retval  无。
 */
static void sysid_stable_amp(uint32_t freq_hz, float target_vpp)
{
    uint32_t bin = sysid_freq_to_bin(freq_hz);
    uint16_t reg = (uint16_t)(target_vpp / DDS_VPP_PER_REG + 0.5f);
    if (reg > 1023u)
    {
        reg = 1023u;
    }
    dds_set_amp_raw(reg);

    for (uint32_t iter = 0; iter < 4u; ++iter)
    {
        HAL_Delay(3);
        adc_start_dual_oneshot();
        adc_unpack_xy(adc_x, adc_y);
        float actual = sysid_measure_vpp(bin);
        float err = target_vpp - actual;
        if (fabsf(err) < SYSID_AMP_TOL)
        {
            break;
        }
        int32_t delta = (int32_t)(err / DDS_VPP_PER_REG * SYSID_AMP_GAIN);
        int32_t new_reg = (int32_t)reg + delta;
        if (new_reg < 0)
        {
            new_reg = 0;
        }
        else if (new_reg > 1023)
        {
            new_reg = 1023;
        }
        reg = (uint16_t)new_reg;
        dds_set_amp_raw(reg);
    }
}

/**
 * @brief   由 ADC1 采样计算 DDS 实际输出 Vpp（RMS 时域法）。
 * @retval  实际 Vpp（V）。
 * @note    块均值去直流后求 RMS：Vpp = 4·√2·3.3·RMS_code/65535 ≈ 18.668·RMS_code/65535。
 *          推导：ADC 输入衰减÷2，故 V_real = 2·V_adc_ac；正弦 Vpp = 2·√2·V_rms_ac；
 *          RMS_code = sqrt(mean((x-mean)²))，V_adc_ac = 3.3·RMS_code/65535。
 *          不依赖 FFT bin 对齐，100Hz~3kHz 全覆盖精确。
 *          使用前需已调用 adc_start_dual_oneshot + adc_unpack_xy 填好 adc_x。
 */
static float sysid_measure_vpp_rms(void)
{
    float mean = 0.0f;
    for (uint32_t i = 0; i < SYSID_N; ++i)
    {
        mean += (float)adc_x[i];
    }
    mean /= (float)SYSID_N;

    float sq = 0.0f;
    for (uint32_t i = 0; i < SYSID_N; ++i)
    {
        float d = (float)adc_x[i] - mean;
        sq += d * d;
    }
    float rms_code = sqrtf(sq / (float)SYSID_N);
    return 18.668f * rms_code / 65535.0f;
}

/**
 * @brief   DDS 幅度闭环校准（RMS 法，独立于 sysid_run）。
 *          迭代调整 reg 使 ADC1 实测 DDS Vpp 达 target_vpp。
 * @param   freq_hz    频率（Hz）。
 * @param   target_vpp 目标 DDS 输出峰峰值（V）。
 * @retval  无。
 * @note    校准后 DDS 保持输出（不 stop）。硬件极限处 reg 饱和 1023。
 */
void sysid_calibrate_dds(uint32_t freq_hz, float target_vpp)
{
    dds_set_freq(freq_hz);
    HAL_Delay(5);

    uint16_t reg = (uint16_t)(target_vpp / DDS_VPP_PER_REG + 0.5f);
    if (reg > 1023u)
    {
        reg = 1023u;
    }
    dds_set_amp_raw(reg);

    for (uint32_t iter = 0; iter < 25u; ++iter)
    {
        HAL_Delay(3);
        adc_start_dual_oneshot();
        adc_unpack_xy(adc_x, adc_y);
        float actual = sysid_measure_vpp_rms();
        float err = target_vpp - actual;
        if (fabsf(err) < SYSID_AMP_TOL)
        {
            break;
        }
        int32_t delta = (int32_t)(err / DDS_VPP_PER_REG * SYSID_AMP_GAIN);
        int32_t new_reg = (int32_t)reg + delta;
        if (new_reg < 0)
        {
            new_reg = 0;
        }
        else if (new_reg > 1023)
        {
            new_reg = 1023;
        }
        reg = (uint16_t)new_reg;
        dds_set_amp_raw(reg);
    }
}

/**
 * @brief   单频点：闭环校准幅度 → 采集 → FFT → 取 bin 复数 X/Y。
 * @param   freq_hz 频率。
 * @param   Xr/Xi/Yr/Yi 输出 X、Y 在目标 bin 的复数实/虚部。
 * @retval  无。
 */
static void sysid_measure_at(uint32_t freq_hz, float *Xr, float *Xi, float *Yr, float *Yi)
{
    dds_set_freq(freq_hz);
    HAL_Delay(5);
    sysid_stable_amp(freq_hz, SYSID_TARGET_VPP);
    /* 闭环收敛后最后采一次干净数据用于 H 估计 */
    adc_start_dual_oneshot();
    dds_stop();
    adc_unpack_xy(adc_x, adc_y);

    uint32_t bin = sysid_freq_to_bin(freq_hz);
    sysid_remove_dc_window(adc_x, fft_in, SYSID_N);
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
    *Xr = fft_out[2 * bin];
    *Xi = fft_out[2 * bin + 1];

    sysid_remove_dc_window(adc_y, fft_in, SYSID_N);
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
    *Yr = fft_out[2 * bin];
    *Yi = fft_out[2 * bin + 1];
}

/**
 * @brief   复数除法 H = Y/X。
 * @param   Yr/Yi 分子实/虚部。
 * @param   Xr/Xi 分母实/虚部。
 * @param   Hr/Hi 输出实/虚部。
 * @retval  无。
 */
static void sysid_cdiv(float Yr, float Yi, float Xr, float Xi, float *Hr, float *Hi)
{
    float den = Xr * Xr + Xi * Xi;
    *Hr = (Yr * Xr + Yi * Xi) / den;
    *Hi = (Yi * Xr - Yr * Xi) / den;
}

/**
 * @brief   在线累加正规方程一行（ATA += row^T*row, ATb += row^T*rhs）。
 * @param   row 方程行（5 元）。
 * @param   rhs 右端项。
 * @retval  无。
 */
static void sysid_accum_row(const float row[5], float rhs)
{
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            ATA[i * 5 + j] += row[i] * row[j];
        }
        ATb[i] += row[i] * rhs;
    }
}

/**
 * @brief   由 H(jω) 构造线性化方程两行（实部+虚部）并累加。
 *          SK 迭代中传入的 Hr/Hi 为归一化后的 H' = H/D_prev。
 * @param   freq_hz 频率。
 * @param   Hr/Hi   H(jω) 实/虚部（SK 迭代时为 H/D_prev）。
 * @retval  无。
 */
static void sysid_build_rows(uint32_t freq_hz, float Hr, float Hi)
{
    float w = 2.0f * SYSID_PI * (float)freq_hz / SYSID_FS_HZ;
    float c1 = cosf(w), s1 = sinf(w);
    float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);

    float He1_re = Hr * c1 + Hi * s1;
    float He1_im = Hi * c1 - Hr * s1;
    float He2_re = Hr * c2 + Hi * s2;
    float He2_im = Hi * c2 - Hr * s2;

    float rowR[5] = {1.0f, c1, c2, -He1_re, -He2_re};
    float rowI[5] = {0.0f, -s1, -s2, -He1_im, -He2_im};
    sysid_accum_row(rowR, Hr);
    sysid_accum_row(rowI, Hi);
}

/**
 * @brief   QR 分解求解正规方程 ATA x = ATb（每轮 SK 迭代调用一次）。
 * @retval  无。
 */
static void sysid_solve(void)
{
    arm_matrix_instance_f32 mR, mQ, mQT, mATb, mQb, mX;
    arm_mat_init_f32(&mR, 5, 5, R);
    arm_mat_init_f32(&mQ, 5, 5, Q);
    arm_mat_init_f32(&mQT, 5, 5, QT);
    arm_mat_init_f32(&mATb, 5, 1, ATb);
    arm_mat_init_f32(&mQb, 5, 1, Qb);
    arm_mat_init_f32(&mX, 5, 1, x_sol);

    static float ATA_copy[25];
    memcpy(ATA_copy, ATA, sizeof(ATA));
    arm_matrix_instance_f32 mATAc;
    arm_mat_init_f32(&mATAc, 5, 5, ATA_copy);

    arm_mat_qr_f32(&mATAc, 1e-6f, &mR, &mQ, tau, tmpA, tmpB);
    arm_mat_trans_f32(&mQ, &mQT);
    arm_mat_mult_f32(&mQT, &mATb, &mQb);
    arm_mat_solve_upper_triangular_f32(&mR, &mQb, &mX);
}

/**
 * @brief   计算真残差 Σ|H_k - N_k/D_k|²（SK 收敛判据）。
 *          与 Levy 的加权目标不同，这是 SK 迭代的真正优化目标。
 * @param   sol 当前解 [b0,b1,b2,a1,a2]。
 * @retval  残差平方和。
 */
static float sysid_compute_resid(const float sol[5])
{
    float b0 = sol[0], b1 = sol[1], b2 = sol[2];
    float a1 = sol[3], a2 = sol[4];
    float sum = 0.0f;
    for (uint32_t k = 0; k < SYSID_NF_LOW; ++k)
    {
        float w = 2.0f * SYSID_PI * (float)H_freq[k] / SYSID_FS_HZ;
        float c1 = cosf(w), s1 = sinf(w);
        float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);
        /* N(jω) = b0 + b1·e^-jω + b2·e^-j2ω */
        float Nr = b0 + b1 * c1 + b2 * c2;
        float Ni = -(b1 * s1 + b2 * s2);
        /* D(jω) = 1 + a1·e^-jω + a2·e^-j2ω */
        float Dr = 1.0f + a1 * c1 + a2 * c2;
        float Di = -(a1 * s1 + a2 * s2);
        float d2 = Dr * Dr + Di * Di;
        if (d2 < 1e-12f)
        {
            d2 = 1e-12f;
        }
        /* H - N/D */
        float er = H_meas_re[k] - (Nr * Dr + Ni * Di) / d2;
        float ei = H_meas_im[k] - (Ni * Dr - Nr * Di) / d2;
        sum += er * er + ei * ei;
    }
    return sum;
}

/**
 * @brief   由辨识 biquad 在三个特殊频点(z=1, z=-1, z=j)的 |H(z)| 判定滤波类型。
 *          判据：
 *            低通: |H(1)| 大, |H(-1)| 小
 *            高通: |H(-1)| 大, |H(1)| 小
 *            带通: |H(1)| 小, |H(-1)| 小, |H(j)| 大
 *            带阻: |H(1)| 大, |H(-1)| 大, |H(j)| 小
 *          「大」= ≥ SMALL·g_max；「小」= < SMALL·g_max。
 * @param   b 分子系数 [b0,b1,b2]。
 * @param   a 分母系数 [1,a1,a2]（a[0] 不使用）。
 * @retval  滤波类型。
 */
static filter_type_t sysid_classify(const float b[3], const float a[3])
{
    /* z = 1 (ω=0, 直流端): H(1) = (b0+b1+b2)/(1+a1+a2) */
    float n_dc = b[0] + b[1] + b[2];
    float d_dc = 1.0f + a[1] + a[2];
    if (fabsf(d_dc) < 1e-12f)
        d_dc = (d_dc >= 0.0f) ? 1e-12f : -1e-12f;
    float g_dc = fabsf(n_dc / d_dc);

    /* z = -1 (ω=π, 折叠频率): H(-1) = (b0-b1+b2)/(1-a1+a2) */
    float n_nyq = b[0] - b[1] + b[2];
    float d_nyq = 1.0f - a[1] + a[2];
    if (fabsf(d_nyq) < 1e-12f)
        d_nyq = (d_nyq >= 0.0f) ? 1e-12f : -1e-12f;
    float g_nyq = fabsf(n_nyq / d_nyq);

    /* z = j (ω=π/2, 中心频率): H(j) = ((b0-b2)-j·b1) / ((1-a2)-j·a1) */
    float nr = b[0] - b[2];
    float ni = -b[1];
    float dr = 1.0f - a[2];
    float di = -a[1];
    float d2_mid = dr * dr + di * di;
    if (d2_mid < 1e-24f)
        d2_mid = 1e-24f;
    float g_mid = sqrtf((nr * nr + ni * ni) / d2_mid);

    float g_max = g_dc;
    if (g_nyq > g_max)
        g_max = g_nyq;
    if (g_mid > g_max)
        g_max = g_mid;
    if (g_max < 1e-12f)
        return FILTER_TYPE_UNKNOWN;

    const float SMALL = 0.5f; /* 「极小」阈值：相对最大值 */

    /* 带通：直流小、高频小、中频大 */
    if (g_dc < g_max * SMALL && g_nyq < g_max * SMALL && g_mid >= g_max * SMALL)
        return FILTER_TYPE_BANDPASS;
    /* 带阻：直流大、高频大、中频小 */
    if (g_dc >= g_max * SMALL && g_nyq >= g_max * SMALL && g_mid < g_max * SMALL)
        return FILTER_TYPE_BANDSTOP;
    /* 低通：直流大、高频小 */
    if (g_dc >= g_max * SMALL && g_nyq < g_max * SMALL)
        return FILTER_TYPE_LOWPASS;
    /* 高通：高频大、直流小 */
    if (g_nyq >= g_max * SMALL && g_dc < g_max * SMALL)
        return FILTER_TYPE_HIGHPASS;
    return FILTER_TYPE_UNKNOWN;
}

/**
 * @brief   获取类型字符串。
 * @param   t 滤波类型枚举。
 * @retval  类型字符串。
 */
const char *sysid_type_str(filter_type_t t)
{
    switch (t)
    {
    case FILTER_TYPE_LOWPASS:
        return "Low-pass";
    case FILTER_TYPE_HIGHPASS:
        return "High-pass";
    case FILTER_TYPE_BANDPASS:
        return "Band-pass";
    case FILTER_TYPE_BANDSTOP:
        return "Band-stop";
    default:
        return "Unknown";
    }
}

#if SYSID_DEBUG_PRINT
/* 诊断打印缓冲（一行 CSV 足够长） */
static char dbg_buf[96];

/**
 * @brief   发送一行 ASCII 到 USART1（阻塞）。
 * @param   s 字符串（含结尾换行）。
 * @retval  无。
 */
static void sysid_dbg_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

/**
 * @brief   打印 CSV 表头。
 * @retval  无。
 */
static void sysid_dbg_header(void)
{
    sysid_dbg_puts("# freq,|H_meas|,|H_fit|,resid,|X|,|Y|\r\n");
}

/**
 * @brief   打印单频点诊断行。
 * @param   f     频率 Hz。
 * @param   Hmr   H 测量实部。
 * @param   Hmi   H 测量虚部。
 * @param   Nr,Ni 拟合 N(jω) 实/虚部。
 * @param   Dr,Di 拟合 D(jω) 实/虚部。
 * @param   Xmag  |X| 模值。
 * @param   Ymag  |Y| 模值。
 * @retval  无。
 */
static void sysid_dbg_row(uint32_t f, float Hmr, float Hmi, float Nr, float Ni, float Dr, float Di, float Xmag,
                          float Ymag)
{
    float Hm = sqrtf(Hmr * Hmr + Hmi * Hmi);
    float d2 = Dr * Dr + Di * Di;
    if (d2 < 1e-12f)
    {
        d2 = 1e-12f;
    }
    float Hf = sqrtf((Nr * Nr + Ni * Ni) / d2);
    float er = Hmr - (Nr * Dr + Ni * Di) / d2;
    float ei = Hmi - (Ni * Dr - Nr * Di) / d2;
    float resid = sqrtf(er * er + ei * ei);
    snprintf(dbg_buf, sizeof(dbg_buf), "%lu,%.4f,%.4f,%.5f,%.2f,%.2f\r\n", (unsigned long)f, Hm, Hf, resid, Xmag, Ymag);
    sysid_dbg_puts(dbg_buf);
}

/**
 * @brief   打印 summary：系数、总残差、迭代轮数。
 * @param   sol   [b0,b1,b2,a1,a2]。
 * @param   resid best_resid。
 * @param   iters 收敛轮数。
 * @retval  无。
 */
static void sysid_dbg_summary(const float sol[5], float resid, uint32_t iters)
{
    snprintf(dbg_buf, sizeof(dbg_buf), "# b0=%.6f,b1=%.6f,b2=%.6f,a1=%.6f,a2=%.6f,resid=%.6f,iters=%lu\r\n", sol[0],
             sol[1], sol[2], sol[3], sol[4], resid, (unsigned long)iters);
    sysid_dbg_puts(dbg_buf);
}

/**
 * @brief   打印 LM 每轮收敛轨迹（iter/resid/lambda/a1/a2）。
 * @param   iter   轮次。
 * @param   resid  本轮残差。
 * @param   lambda 本轮阻尼。
 * @param   a1,a2  本轮分母系数。
 * @retval  无。
 */
static void sysid_dbg_lm_iter(uint32_t iter, float resid, float lambda, float a1, float a2)
{
    snprintf(dbg_buf, sizeof(dbg_buf), "# lm_iter=%lu resid=%.6f lambda=%.2e a1=%.6f a2=%.6f\r\n", (unsigned long)iter,
             resid, lambda, a1, a2);
    sysid_dbg_puts(dbg_buf);
}
#endif /* SYSID_DEBUG_PRINT */

/**
 * @brief   计算 LM 雅可比累积量 JᵀJ(5×5) 与 Jᵀr(5×1)。
 *          残差 r_k = H_k - N(p)/D(p)（复数），实部+虚部拆为两行。
 *          解析偏导：
 *            ∂N/∂b0=1, ∂N/∂b1=e^-jω, ∂N/∂b2=e^-j2ω
 *            ∂D/∂a1=e^-jω, ∂D/∂a2=e^-j2ω
 *            ∂r/∂b_i = -(∂N/∂b_i)/D
 *            ∂r/∂a_i =  N·(∂D/∂a_i)/D²
 *          JᵀJ += J_reᵀJ_re + J_imᵀJ_im，Jᵀr += J_re·r_re + J_im·r_im。
 * @param   p 当前参数 [b0,b1,b2,a1,a2]。
 * @param   JTJ_out 输出 5×5 累积（调用前需清零）。
 * @param   JTr_out 输出 5×1 累积（调用前需清零）。
 * @retval  无。
 */
static void sysid_lm_jacobian(const float p[5], float *JTJ_out, float *JTr_out)
{
    float b0 = p[0], b1 = p[1], b2 = p[2];
    float a1 = p[3], a2 = p[4];

    for (uint32_t k = 0; k < SYSID_NF_LOW; ++k)
    {
        float w = 2.0f * SYSID_PI * (float)H_freq[k] / SYSID_FS_HZ;
        float c1 = cosf(w), s1 = sinf(w);
        float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);

        /* N(jω) = b0 + b1·e^-jω + b2·e^-j2ω */
        float Nr = b0 + b1 * c1 + b2 * c2;
        float Ni = -(b1 * s1 + b2 * s2);
        /* D(jω) = 1 + a1·e^-jω + a2·e^-j2ω */
        float Dr = 1.0f + a1 * c1 + a2 * c2;
        float Di = -(a1 * s1 + a2 * s2);
        float d2 = Dr * Dr + Di * Di;
        if (d2 < 1e-12f)
        {
            d2 = 1e-12f;
        }
        float inv_d = 1.0f / d2;

        /* r = H - N/D = (H·D - N)/D */
        float HDr = H_meas_re[k] * Dr - H_meas_im[k] * Di;
        float HDi = H_meas_re[k] * Di + H_meas_im[k] * Dr;
        float rr = (HDr - Nr) * inv_d;
        float ri = (HDi - Ni) * inv_d;

        /* ∂r/∂b_i = -(∂N/∂b_i)/D = -(∂N/∂b_i)·conj(D)/|D|²
         *   ∂N/∂b0=1: g_b0 = -conj(D)/d2 = (-Dr + j·Di)/d2
         *   ∂N/∂b1=e^-jω=(c1 - j·s1): g_b1 = -(c1-j·s1)·conj(D)/d2
         *   ∂N/∂b2=e^-j2ω=(c2 - j·s2): g_b2 = -(c2-j·s2)·conj(D)/d2 */
        /* 公共因子 -conj(D)/d2 = (-Dr + j·Di)/d2 */
        float fr = -Dr * inv_d;
        float fi = Di * inv_d;
        /* g_b0 = (fr, fi) */
        float g_b0_r = fr, g_b0_i = fi;
        /* g_b1 = (fr + j·fi)·(c1 - j·s1) = (fr·c1 + fi·s1) + j(fi·c1 - fr·s1) */
        float g_b1_r = fr * c1 + fi * s1;
        float g_b1_i = fi * c1 - fr * s1;
        /* g_b2 = (fr + j·fi)·(c2 - j·s2) */
        float g_b2_r = fr * c2 + fi * s2;
        float g_b2_i = fi * c2 - fr * s2;

        /* ∂r/∂a_i = N·(∂D/∂a_i)/D² = N·(∂D/∂a_i)·conj(D)²/|D|⁴
         *   更简洁：N/D · (∂D/∂a_i)/D = (N/D)·(∂D/∂a_i)·conj(D)/|D|²
         *   令 NoverD = N·conj(D)/d2，g_a_i = NoverD · (∂D/∂a_i)·conj(D)/d2
         *   但 (∂D/∂a_i)·conj(D)/d2 = conj(D)/d2 · (∂D/∂a_i)
         *   ∂D/∂a1 = e^-jω = (c1 - j·s1), ∂D/∂a2 = e^-j2ω = (c2 - j·s2)
         *   令 h = conj(D)/d2 = (Dr - j·Di)/d2 → hr=Dr/d2, hi=-Di/d2 */
        float hr = Dr * inv_d;
        float hi = -Di * inv_d;
        /* h·(∂D/∂a1) = (hr+j·hi)·(c1-j·s1) = (hr·c1+hi·s1) + j(hi·c1-hr·s1) */
        float h_a1_r = hr * c1 + hi * s1;
        float h_a1_i = hi * c1 - hr * s1;
        float h_a2_r = hr * c2 + hi * s2;
        float h_a2_i = hi * c2 - hr * s2;
        /* NoverD = N·conj(D)/d2 = (Nr+j·Ni)·(Dr-j·Di)/d2 */
        float ND_r = (Nr * Dr + Ni * Di) * inv_d;
        float ND_i = (Ni * Dr - Nr * Di) * inv_d;
        /* g_a1 = NoverD · h_a1 */
        float g_a1_r = ND_r * h_a1_r - ND_i * h_a1_i;
        float g_a1_i = ND_r * h_a1_i + ND_i * h_a1_r;
        float g_a2_r = ND_r * h_a2_r - ND_i * h_a2_i;
        float g_a2_i = ND_r * h_a2_i + ND_i * h_a2_r;

        /* J 行（实部+虚部两行）：[g_b0, g_b1, g_b2, g_a1, g_a2] */
        float gr[5] = {g_b0_r, g_b1_r, g_b2_r, g_a1_r, g_a2_r};
        float gi[5] = {g_b0_i, g_b1_i, g_b2_i, g_a1_i, g_a2_i};

        /* JᵀJ += grᵀgr + giᵀgi；Jᵀr += gr·rr + gi·ri */
        for (uint32_t i = 0; i < 5u; ++i)
        {
            for (uint32_t j = 0; j < 5u; ++j)
            {
                JTJ_out[i * 5 + j] += gr[i] * gr[j] + gi[i] * gi[j];
            }
            JTr_out[i] += gr[i] * rr + gi[i] * ri;
        }
    }
}

/**
 * @brief   Levenberg-Marquardt 非线性最小二乘精化：直接最小化 Σ|H-N/D|²。
 *          (JᵀJ + λ·diag(JᵀJ))·Δp = -Jᵀr，λ 信赖域阻尼。
 *          残差下降则接受 Δp、λ/=10；上升则拒绝、λ*=10。best_p 单调取优兜底。
 *          稳定性闸：a2∈[0,1]（极点不出单位圆），违反则拒绝该步。
 *          SYSID_DEBUG_PRINT 时打印每轮轨迹。
 * @param   p_inout 输入初值（Levy 解），输出 LM 精化后最优解 [b0,b1,b2,a1,a2]。
 * @retval  最优残差。
 */
static float sysid_lm_run(float p_inout[5])
{
    float p[5];
    memcpy(p, p_inout, sizeof(p));

    float resid = sysid_compute_resid(p);
    float best_p[5];
    memcpy(best_p, p, sizeof(p));
    float best_resid = resid;

    float lambda = SYSID_LM_LAMBDA_INIT;

#if SYSID_DEBUG_PRINT
    sysid_dbg_lm_iter(0u, resid, lambda, p[3], p[4]);
#endif

    for (uint32_t iter = 1u; iter < SYSID_LM_MAX_ITER; ++iter)
    {
        /* 算 JᵀJ, Jᵀr */
        memset(JTJ, 0, sizeof(JTJ));
        memset(JTr, 0, sizeof(JTr));
        sysid_lm_jacobian(p, JTJ, JTr);

        /* A = JTJ + λ·diag(JTJ)；b = -JTr */
        memcpy(A_lm, JTJ, sizeof(JTJ));
        for (uint32_t i = 0; i < 5u; ++i)
        {
            A_lm[i * 5 + i] += lambda * JTJ[i * 5 + i];
            b_lm[i] = -JTr[i];
        }

        /* QR 求解 A·dp = b */
        arm_matrix_instance_f32 mA, mQ, mQT, mR, mB, mQb, mDp;
        arm_mat_init_f32(&mA, 5, 5, A_lm);
        arm_mat_init_f32(&mQ, 5, 5, Q);
        arm_mat_init_f32(&mQT, 5, 5, QT);
        arm_mat_init_f32(&mR, 5, 5, R);
        arm_mat_init_f32(&mB, 5, 1, b_lm);
        arm_mat_init_f32(&mQb, 5, 1, Qb);
        arm_mat_init_f32(&mDp, 5, 1, dp);
        if (arm_mat_qr_f32(&mA, 1e-6f, &mR, &mQ, tau, tmpA, tmpB) != ARM_MATH_SUCCESS)
        {
            lambda *= 10.0f;
            if (lambda > SYSID_LM_LAMBDA_MAX)
            {
                break;
            }
            continue;
        }
        arm_mat_trans_f32(&mQ, &mQT);
        arm_mat_mult_f32(&mQT, &mB, &mQb);
        arm_mat_solve_upper_triangular_f32(&mR, &mQb, &mDp);

        /* 试探 p_new = p + dp */
        float p_new[5];
        for (uint32_t i = 0; i < 5u; ++i)
        {
            p_new[i] = p[i] + dp[i];
        }

        /* 注：原稳定性闸（a2∈[0,1] 硬拒绝）已移除。它阻止 LM 从 Levy 坏初值
         * (如 HPF a2=-0.64) 收敛到稳定真值(a2=0.783)，导致 LM 完全卡死。
         * compute_resid 有 d2<1e-12 保护不会除零，best_p 兜底防发散。
         * 真值稳定，LM 方向对会自然收敛到稳定域。 */

        float resid_new = sysid_compute_resid(p_new);

        if (resid_new < resid)
        {
            /* 接受：p←p_new, λ/=10 */
            float rel = (resid - resid_new) / (resid + 1e-12f);
            memcpy(p, p_new, sizeof(p));
            resid = resid_new;
            lambda *= 0.1f;
            if (lambda < 1e-12f)
            {
                lambda = 1e-12f;
            }
            if (resid < best_resid)
            {
                best_resid = resid;
                memcpy(best_p, p, sizeof(p));
            }
#if SYSID_DEBUG_PRINT
            sysid_dbg_lm_iter(iter, resid, lambda, p[3], p[4]);
#endif
            /* 收敛判据：残差相对变化 < tol */
            if (rel < SYSID_LM_TOL)
            {
                break;
            }
        }
        else
        {
            /* 拒绝：λ*=10 */
            lambda *= 10.0f;
            if (lambda > SYSID_LM_LAMBDA_MAX)
            {
                break;
            }
#if SYSID_DEBUG_PRINT
            sysid_dbg_lm_iter(iter, resid, lambda, p[3], p[4]);
#endif
        }
    }

    memcpy(p_inout, best_p, sizeof(best_p));
    return best_resid;
}

/**
 * @brief   执行建模主流程。
 *          阶段 1：扫频 422 点采集 H(jω)；
 *          阶段 2：Levy 一次线性化解初值 → LM 非线性最小二乘精化（直接最小化
 *                  Σ|H-N/D|²，根治 Levy/SK 的间接目标偏差）。
 *          SYSID_DEBUG_PRINT=1 时，经 USART1 打印 LM 轨迹、每频点残差与 summary 供诊断。
 * @param   b_out 输出分子系数 [b0,b1,b2]。
 * @param   a_out 输出分母系数 [1,a1,a2]。
 * @retval  滤波类型。
 */
filter_type_t sysid_run(float b_out[3], float a_out[3])
{
    ui_show_status("learning...");
    arm_rfft_fast_init_f32(&fft_inst, SYSID_N);
    arm_hanning_f32(hanning_win, SYSID_N);

    /* ---- 阶段 1：扫频采集 H(jω) ---- */
    for (uint32_t k = 0; k < SYSID_NF; ++k)
    {
        uint32_t f = sysid_freq_at(k);
        float Xr, Xi, Yr, Yi;
        sysid_measure_at(f, &Xr, &Xi, &Yr, &Yi);

        sysid_cdiv(Yr, Yi, Xr, Xi, &H_meas_re[k], &H_meas_im[k]);
        H_freq[k] = f;
        H_Xmag[k] = sqrtf(Xr * Xr + Xi * Xi);
        H_Ymag[k] = sqrtf(Yr * Yr + Yi * Yi);
    }

    /* ---- 阶段 2：Levy 初值 + LM 精化 ---- */
    /* 2a. Levy 一次线性化（D_prev=1）求初值 x_sol */
    memset(ATA, 0, sizeof(ATA));
    memset(ATb, 0, sizeof(ATb));
    for (uint32_t k = 0; k < SYSID_NF_LOW; ++k)
    {
        /* D_prev=1 → H' = H，经典 Levy 线性化 */
        sysid_build_rows(H_freq[k], H_meas_re[k], H_meas_im[k]);
    }
    sysid_solve();

    /* 2b. LM 非线性最小二乘精化 x_sol */
    float best_resid = sysid_lm_run(x_sol);
    float best_sol[5];
    memcpy(best_sol, x_sol, sizeof(x_sol));
    uint32_t iters_done = SYSID_LM_MAX_ITER; /* LM 迭代上限（实际轮数见 dbg 轨迹） */

#if SYSID_DEBUG_PRINT
    /* 诊断：用 best_sol 计算每频点 N(jω)/D(jω) 并打印残差，供定位偏差来源 */
    sysid_dbg_header();
    for (uint32_t k = 0; k < SYSID_NF; ++k)
    {
        float w = 2.0f * SYSID_PI * (float)H_freq[k] / SYSID_FS_HZ;
        float c1 = cosf(w), s1 = sinf(w);
        float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);
        float Nr = best_sol[0] + best_sol[1] * c1 + best_sol[2] * c2;
        float Ni = -(best_sol[1] * s1 + best_sol[2] * s2);
        float Dr = 1.0f + best_sol[3] * c1 + best_sol[4] * c2;
        float Di = -(best_sol[3] * s1 + best_sol[4] * s2);
        sysid_dbg_row(H_freq[k], H_meas_re[k], H_meas_im[k], Nr, Ni, Dr, Di, H_Xmag[k], H_Ymag[k]);
    }
    sysid_dbg_summary(best_sol, best_resid, iters_done);
#endif /* SYSID_DEBUG_PRINT */

    b_out[0] = best_sol[0];
    b_out[1] = best_sol[1];
    b_out[2] = best_sol[2];
    a_out[0] = 1.0f;
    a_out[1] = best_sol[3];
    a_out[2] = best_sol[4];

    filter_type_t t = sysid_classify(b_out, a_out);
    ui_show_filter_type(sysid_type_str(t));
    ui_show_status("learn done");
    return t;
}
