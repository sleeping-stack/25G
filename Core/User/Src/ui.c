/**
 ******************************************************************************
 * @file    ui.c
 * @brief   显示后端实现——陶晶驰串口屏，USART1 发送。
 ******************************************************************************
 * @attention 用户自有代码，CubeMX 重新生成不会覆盖。
 ******************************************************************************
 */
#include "ui.h"
#include "usart.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;

/* 陶晶驰指令结束符 */
static const uint8_t TJC_EOF[3] = {0xFFu, 0xFFu, 0xFFu};

static char ui_buf[128];

/**
 * @brief   发送陶晶驰文本赋值指令（obj.attr="val"+0xFF×3）。
 * @param   obj 控件名（如 "t1"、"t6"）。
 * @param   attr 属性名（如 "txt"）。
 * @param   val 属性值。
 * @retval  无。
 */
static void tjc_send_txt(const char *obj, const char *attr, const char *val)
{
    int n = snprintf(ui_buf, sizeof(ui_buf), "%s.%s=\"%s\"", obj, attr, val);
    if (n > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)ui_buf, (uint16_t)n, 100);
        HAL_UART_Transmit(&huart1, TJC_EOF, 3, 100);
    }
}

/**
 * @brief   初始化 UI（串口屏开机问候）。
 * @retval  无。
 */
void ui_init(void)
{
    tjc_send_txt("t1", "txt", "电赛25G ready");
}

/**
 * @brief   显示滤波类型（发串口屏 t6 控件）。
 * @param   type 类型字符串。
 * @retval  无。
 */
void ui_show_filter_type(const char *type)
{
    tjc_send_txt("t6", "txt", type);
}

/**
 * @brief   显示状态信息（发串口屏 t1 控件，覆盖最新一条）。
 * @param   status 状态字符串。
 * @retval  无。
 */
void ui_show_status(const char *status)
{
    tjc_send_txt("t1", "txt", status);
}

/**
 * @brief   显示学习用时（发串口屏 t5 控件，格式如 "12.3s"）。
 * @param   seconds 用时（秒）。
 * @retval  无。
 */
void ui_show_learning_time(float seconds)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fs", seconds);
    tjc_send_txt("t5", "txt", buf);
}
