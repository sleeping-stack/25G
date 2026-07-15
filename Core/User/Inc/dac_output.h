/**
  ******************************************************************************
  * @file    dac_output.h
  * @brief   DAC1 实时输出——DMA circular 双缓冲，TIM6 触发。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __DAC_OUTPUT_H
#define __DAC_OUTPUT_H

#include <stdint.h>

/** 实时 DAC 输出缓冲（双缓冲，half-transfer/complete 各填一半） */
#define DAC_RT_BUF_LEN 512

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
