/**
 ******************************************************************************
 * @file    adc_sampling.h
 * @brief   ADC 双通道采样接口——建模 dual oneshot 与实时 circular 双缓冲。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#ifndef __ADC_SAMPLING_H
#define __ADC_SAMPLING_H

#include "adc.h"
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/** 建模：dual oneshot 缓冲（uint32 打包，低16=ADC1/X，高16=ADC2/Y） */
#define ADC_BUF_SIZE 4096
extern uint32_t adc_sampling_buffer[ADC_BUF_SIZE];
extern volatile uint8_t adc_sampling_done;

/** 实时：单通道 circular 双缓冲 */
#define ADC_RT_BUF_LEN 512
extern uint16_t adc_rt_buf[ADC_RT_BUF_LEN];

/** ADC 调理偏移码值：信号衰减一半后抬升约 1.2V
 *  仅实时复现（rt_process）使用此硬编码值；建模（sysid）已改为块均值自适应去直流。 */
#define ADC_OFFSET_CODE 24824.0f

typedef void (*adc_rt_callback_t)(uint16_t *buf, uint32_t len);

/**
 * @brief   启动 dual simultaneous oneshot 采样，阻塞等完成。
 * @retval  无。
 */
void adc_start_dual_oneshot(void);

/**
 * @brief   拆包 dual 数据到 X/Y 两个 uint16 数组。
 * @param   x 输出 ADC1 数据（CDR 低 16 位）。
 * @param   y 输出 ADC2 数据（CDR 高 16 位）。
 * @retval  无。
 */
void adc_unpack_xy(uint16_t *x, uint16_t *y);

/**
 * @brief   注册实时 half/complete 回调。
 * @param   ht half-transfer 回调。
 * @param   tc transfer-complete 回调。
 * @retval  无。
 */
void adc_rt_set_callbacks(adc_rt_callback_t ht, adc_rt_callback_t tc);

/**
 * @brief   启动实时 circular DMA 采样。
 * @retval  无。
 */
void adc_start_rt(void);

/**
 * @brief   停止实时采样并清除回调。
 * @retval  无。
 */
void adc_stop_rt(void);

#endif /* __ADC_SAMPLING_H */
