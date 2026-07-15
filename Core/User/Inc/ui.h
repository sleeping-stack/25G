/**
  ******************************************************************************
  * @file    ui.h
  * @brief   显示后端抽象——陶晶驰串口屏实现，接口预留升级。
  ******************************************************************************
  * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
  ******************************************************************************
  */
#ifndef __UI_H
#define __UI_H

#include <stdint.h>
#include <stdio.h>

/**
  * @brief   初始化 UI（串口屏开机问候）。
  * @retval  无。
  */
void ui_init(void);

/**
  * @brief   显示滤波类型（发串口屏 t6 控件）。
  * @param   type 类型字符串（如 "Low-pass"）。
  * @retval  无。
  */
void ui_show_filter_type(const char *type);

/**
  * @brief   显示状态信息（发串口屏 t1 控件，覆盖最新一条）。
  * @param   status 状态字符串。
  * @retval  无。
  */
void ui_show_status(const char *status);

/**
  * @brief   打印日志（发串口屏 t1 控件，覆盖最新一条）。
  * @param   fmt printf 风格格式串。
  * @retval  无。
  */
void ui_log(const char *fmt, ...);

#endif /* __UI_H */
