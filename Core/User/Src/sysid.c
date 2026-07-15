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
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* ---- 常量 ---- */
#define SYSID_FS_HZ (240000000.0f / 293.0f) /* 819112.6 Hz，TIM6 Frequency */
#define SYSID_N ADC_BUF_SIZE /* 4096 */
#define SYSID_PI 3.14159265358979f

/* 主扫描 1k~50k 步长 200：246 点；高频稀疏 7 点；共 253 点 */
#define SYSID_NF_LOW 246
#define SYSID_NF_HIGH 7
#define SYSID_NF (SYSID_NF_LOW + SYSID_NF_HIGH)

/* 建模激励目标 Vpp */
#define SYSID_TARGET_VPP 2.0f
/* 闭环容差与增益 */
#define SYSID_AMP_TOL 0.05f
#define SYSID_AMP_GAIN 0.8f

/* SK 迭代参数 */
#define SYSID_SK_MAX_ITER 5u     /* 最大迭代次数（第 0 轮即 Levy） */
#define SYSID_SK_D_FLOOR  1e-6f  /* |D_prev|² 下限，防止极点处权重爆炸 */

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

static arm_rfft_fast_instance_f32 fft_inst;

/**
 * @brief   取第 idx 个频点（Hz）。
 * @param   idx 频点索引（0~252）。
 * @retval  频率（Hz）。
 */
static uint32_t sysid_freq_at(uint32_t idx)
{
    if (idx < SYSID_NF_LOW)
    {
        return 1000u + idx * 200u;
    }
    static const uint32_t hi[SYSID_NF_HIGH] = {60000, 80000, 100000, 150000, 200000, 300000, 400000};
    return hi[idx - SYSID_NF_LOW];
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
 * @brief   单频点：闭环校准幅度 → 采集 → FFT → 取 bin 复数 X/Y。
 * @param   freq_hz 频率。
 * @param   Xr/Xi/Yr/Yi 输出 X、Y 在目标 bin 的复数实/虚部。
 * @retval  无。
 */
static void sysid_measure_at(uint32_t freq_hz, float *Xr, float *Xi, float *Yr, float *Yi)
{
    dds_set_freq(freq_hz);
    ui_log("[DDS] start f=%lu\r\n", (unsigned long)freq_hz);
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
    for (uint32_t k = 0; k < SYSID_NF; ++k)
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
 * @brief   由辨识 biquad 扫频响 0~π 判定滤波类型。
 * @param   b 分子系数。
 * @param   a 分母系数。
 * @retval  滤波类型。
 */
static filter_type_t sysid_classify(const float b[3], const float a[3])
{
    float gmax = 0.0f, gmin = 1e30f;
    float g0 = 0.0f, gN = 0.0f;
    for (uint32_t k = 0; k <= 256; ++k)
    {
        float w = SYSID_PI * (float)k / 256.0f;
        float c1 = cosf(w), s1 = sinf(w);
        float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);
        float nr = b[0] + b[1] * c1 + b[2] * c2;
        float ni = -(b[1] * s1 + b[2] * s2);
        float dr = 1.0f + a[1] * c1 + a[2] * c2;
        float di = -(a[1] * s1 + a[2] * s2);
        float gmag = sqrtf(nr * nr + ni * ni) / sqrtf(dr * dr + di * di);
        if (k == 0)
            g0 = gmag;
        if (k == 256)
            gN = gmag;
        if (gmag > gmax)
            gmax = gmag;
        if (gmag < gmin)
            gmin = gmag;
    }
    if (gmin < g0 * 0.3f && gmin < gN * 0.3f && g0 > gmin * 3.0f && gN > gmin * 3.0f)
    {
        return FILTER_TYPE_BANDSTOP;
    }
    if (gmax > g0 * 3.0f && gmax > gN * 3.0f)
    {
        return FILTER_TYPE_BANDPASS;
    }
    if (g0 > gN * 3.0f)
    {
        return FILTER_TYPE_LOWPASS;
    }
    if (gN > g0 * 3.0f)
    {
        return FILTER_TYPE_HIGHPASS;
    }
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

/**
 * @brief   执行建模主流程。
 *          阶段 1：扫频 253 点采集 H(jω)；
 *          阶段 2：SK 迭代（第 0 轮即 Levy，后续轮用 H/D_prev 归一化消除权重偏差）。
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

        if ((k & 0x1F) == 0)
        {
            ui_log("scan %lu/%lu f=%lu", (unsigned long)k, (unsigned long)SYSID_NF, (unsigned long)f);
        }
    }

    /* ---- 阶段 2：SK 迭代 ---- */
    float a1_prev = 0.0f, a2_prev = 0.0f; /* 第 0 轮 D_prev=1，即经典 Levy */
    float best_sol[5];
    float best_resid = 1e30f;

    for (uint32_t iter = 0; iter < SYSID_SK_MAX_ITER; ++iter)
    {
        memset(ATA, 0, sizeof(ATA));
        memset(ATb, 0, sizeof(ATb));

        for (uint32_t k = 0; k < SYSID_NF; ++k)
        {
            float w = 2.0f * SYSID_PI * (float)H_freq[k] / SYSID_FS_HZ;
            float c1 = cosf(w), s1 = sinf(w);
            float c2 = cosf(2.0f * w), s2 = sinf(2.0f * w);
            /* D_prev(jω) = 1 + a1_prev·e^-jω + a2_prev·e^-j2ω */
            float Dr = 1.0f + a1_prev * c1 + a2_prev * c2;
            float Di = -(a1_prev * s1 + a2_prev * s2);
            float d2 = Dr * Dr + Di * Di;
            if (d2 < SYSID_SK_D_FLOOR)
            {
                d2 = SYSID_SK_D_FLOOR;
            }
            /* H' = H / D_prev（SK 归一化，消除 Levy 的 |D|² 加权偏差） */
            float Hr = (H_meas_re[k] * Dr + H_meas_im[k] * Di) / d2;
            float Hi = (H_meas_im[k] * Dr - H_meas_re[k] * Di) / d2;
            sysid_build_rows(H_freq[k], Hr, Hi);
        }

        sysid_solve();
        float resid = sysid_compute_resid(x_sol);
        ui_log("SK iter %lu resid=%.6f", (unsigned long)iter, resid);

        if (resid < best_resid)
        {
            best_resid = resid;
            memcpy(best_sol, x_sol, sizeof(x_sol));
            a1_prev = x_sol[3];
            a2_prev = x_sol[4];
        }
        else
        {
            break; /* 残差反弹，停止迭代，回退 best_sol */
        }
    }

    b_out[0] = best_sol[0];
    b_out[1] = best_sol[1];
    b_out[2] = best_sol[2];
    a_out[0] = 1.0f;
    a_out[1] = best_sol[3];
    a_out[2] = best_sol[4];

    ui_log("b=[%.3f %.3f %.3f] a=[1 %.3f %.3f]", b_out[0], b_out[1], b_out[2], a_out[1], a_out[2]);

    filter_type_t t = sysid_classify(b_out, a_out);
    ui_show_filter_type(sysid_type_str(t));
    ui_show_status("learn done");
    return t;
}
