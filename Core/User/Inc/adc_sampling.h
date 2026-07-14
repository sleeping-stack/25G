#ifndef __ADC_SAMPLING_H
#define __ADC_SAMPLING_H

#include "adc.h"

extern ADC_HandleTypeDef hadc1;

/** DMA 采样缓冲区长度 */
#define ADC_BUF_SIZE 4096

extern uint16_t adc_sampling_buf[ADC_BUF_SIZE];
extern volatile uint8_t adc_sampling_done;

/**
 * @brief  启动 ADC DMA 采样
 * @param  hadc ADC 句柄
 */
void adc_start_sampling(ADC_HandleTypeDef *hadc);

#endif /* __ADC_SAMPLING_H */
