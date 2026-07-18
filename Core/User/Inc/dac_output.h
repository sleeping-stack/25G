/**
  ******************************************************************************
  * @file    dac_output.h
  * @brief   DAC1 实时输出——DMA circular 双缓冲，TIM7 触发（f_DAC = 2·fs）。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __DAC_OUTPUT_H
#define __DAC_OUTPUT_H

#include <stdint.h>

/** 实时 DAC 输出缓冲（双缓冲，half-transfer/complete 各填一半）。
 *  DAC 由 TIM7 触发 @ 1.644MHz = 2·fs，故缓冲长度 = ADC 块长 × 2 (L=2 内插)。
 *  half-transfer/complete 各填一半 = 一个 ADC 块对应的内插后点数 (256×2=512)。 */
#define DAC_RT_BUF_LEN 1024

extern uint16_t dac_rt_buf[DAC_RT_BUF_LEN];

/**
  * @brief   初始化 DAC 缓冲为中点（0V）。
  * @retval  无。
  */
void dac_rt_init(void);

/**
  * @brief   启动 DAC1 DMA circular 输出。
  * @retval  无。
  */
void dac_rt_start(void);

/**
  * @brief   停止 DAC1 DMA 输出。
  * @retval  无。
  */
void dac_rt_stop(void);

#endif /* __DAC_OUTPUT_H */
