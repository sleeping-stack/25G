/**
  ******************************************************************************
  * @file    app_state.h
  * @brief   应用状态机——IDLE/LEARNING/REPLAY + 串口屏命令解析。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __APP_STATE_H
#define __APP_STATE_H

/**
  * @brief   初始化应用（UI/DDS/DAC/ADC 校准/串口接收）。
  * @retval  无。
  */
void app_init(void);

/**
  * @brief   主循环轮询（处理串口屏命令）。
  * @retval  无。
  */
void app_poll(void);

#endif /* __APP_STATE_H */
