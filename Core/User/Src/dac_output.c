/**
  ******************************************************************************
  * @file    dac_output.c
  * @brief   DAC1 实时输出实现——DMA circular，TIM7 触发（f_DAC = 2·fs = 1.644MHz）。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#include "dac_output.h"
#include "dac.h"
#include "tim.h"

/* DMA 缓冲，放 .DMA_BUF 段（RAM，DMA 可访问） */
uint16_t dac_rt_buf[DAC_RT_BUF_LEN] __attribute__((section(".DMA_BUF")));

/**
  * @brief   初始化 DAC 缓冲为中点（0V）。
  * @retval  无。
  */
void dac_rt_init(void)
{
    for (uint32_t i = 0; i < DAC_RT_BUF_LEN; ++i)
    {
        dac_rt_buf[i] = 2048;
    }
}

/**
  * @brief   启动 DAC1 DMA circular 输出（TIM7 触发）。
  * @retval  无。
  */
void dac_rt_start(void)
{
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_rt_buf, DAC_RT_BUF_LEN, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(&htim7);
}

/**
  * @brief   停止 DAC1 DMA 输出（含 TIM7）。
  * @retval  无。
  */
void dac_rt_stop(void)
{
    HAL_TIM_Base_Stop(&htim7);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}
