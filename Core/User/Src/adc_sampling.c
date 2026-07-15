/**
  ******************************************************************************
  * @file    adc_sampling.c
  * @brief   ADC 双通道采样实现——建模 dual oneshot 与实时 circular 双缓冲。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#include "adc_sampling.h"
#include "tim.h"
#include "string.h"

/* 建模 dual oneshot 缓冲，放 .DMA_BUF 段（RAM，DMA 可访问） */
uint32_t adc_sampling_buffer[ADC_BUF_SIZE] __attribute__((section(".DMA_BUF")));
volatile uint8_t adc_sampling_done = 0;

/* 实时单通道 circular 缓冲，放 .DMA_BUF 段（RAM，DMA 可访问） */
uint16_t adc_rt_buf[ADC_RT_BUF_LEN] __attribute__((section(".DMA_BUF")));

static adc_rt_callback_t rt_ht_cb = NULL;
static adc_rt_callback_t rt_tc_cb = NULL;

/**
  * @brief   ADC DMA 转换完成回调（建模与实时共用，通过 rt_tc_cb 区分）。
  * @param   hadc ADC 句柄。
  * @retval  无。
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }
    if (rt_tc_cb != NULL)
    {
        rt_tc_cb(&adc_rt_buf[ADC_RT_BUF_LEN / 2], ADC_RT_BUF_LEN / 2);
        return;
    }
    adc_sampling_done = 1;
    HAL_TIM_Base_Stop(&htim6);
}

/**
  * @brief   ADC DMA 半转换完成回调（实时模式）。
  * @param   hadc ADC 句柄。
  * @retval  无。
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && rt_ht_cb != NULL)
    {
        rt_ht_cb(adc_rt_buf, ADC_RT_BUF_LEN / 2);
    }
}

/**
  * @brief   启动 dual simultaneous oneshot 采样，阻塞等完成。
  * @retval  无。
  */
void adc_start_dual_oneshot(void)
{
    adc_sampling_done = 0;
    memset(adc_sampling_buffer, 0, sizeof(adc_sampling_buffer));

    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_sampling_buffer, ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim6);

    while (adc_sampling_done != 1)
    {
    }
    HAL_TIM_Base_Stop(&htim6);
}

/**
  * @brief   拆包 dual 数据到 X/Y 两个 uint16 数组。
  * @param   x 输出 ADC1 数据（CDR 低 16 位）。
  * @param   y 输出 ADC2 数据（CDR 高 16 位）。
  * @retval  无。
  */
void adc_unpack_xy(uint16_t *x, uint16_t *y)
{
    for (uint32_t i = 0; i < ADC_BUF_SIZE; ++i)
    {
        x[i] = (uint16_t)(adc_sampling_buffer[i] & 0xFFFFu);
        y[i] = (uint16_t)((adc_sampling_buffer[i] >> 16) & 0xFFFFu);
    }
}

/**
  * @brief   注册实时 half/complete 回调。
  * @param   ht half-transfer 回调。
  * @param   tc transfer-complete 回调。
  * @retval  无。
  */
void adc_rt_set_callbacks(adc_rt_callback_t ht, adc_rt_callback_t tc)
{
    rt_ht_cb = ht;
    rt_tc_cb = tc;
}

/**
  * @brief   启动实时 circular DMA 采样。
  * @retval  无。
  */
void adc_start_rt(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_rt_buf, ADC_RT_BUF_LEN);
    HAL_TIM_Base_Start(&htim6);
}

/**
  * @brief   停止实时采样并清除回调。
  * @retval  无。
  */
void adc_stop_rt(void)
{
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    rt_ht_cb = NULL;
    rt_tc_cb = NULL;
}
