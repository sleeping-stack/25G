#include "adc_sampling.h"
#include "tim.h"
#include "string.h"

uint16_t adc_sampling_buffer[ADC_BUF_SIZE] __attribute__((section(".ADC_SAMPLE_BUF"))); // adc dma 采集数据缓冲区
volatile uint8_t adc_sampling_done = 0;

/**
 * @brief  ADC DMA 转换完成回调
 * @param  hadc ADC 句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_sampling_done = 1;
        HAL_TIM_Base_Stop(&htim6); // 停止定时器
        HAL_ADC_Stop_DMA(hadc);
    }
}

void adc_start_sampling(ADC_HandleTypeDef *hadc)
{
    adc_sampling_done = 0;
    memset(adc_sampling_buffer, 0, sizeof(adc_sampling_buffer));
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_sampling_buffer, ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim6); // 启动定时器
    while (adc_sampling_done != 1)
    {
    }
}