#ifndef USER_SERIALPLOT_H
#define USER_SERIALPLOT_H
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED

/* 包含头文件 */
#include <stdarg.h>
#include "user_uart.h"
#include "../../Core/Inc/bsp_config.h"
#include "../../User_Architect/user_systick.h"

/* 宏定义 ----------------------------------------------------------------*/
#define SERIALPLOT_MAX_CH      (12)    /* 最多绘图通道数 */
#define SERIALPLOT_NAME_LEN    (20)    /* 变量名最大长度 */
#define SERIALPLOT_RX_LEN      (64)    /* 接收缓冲长度 */
#define SERIALPLOT_FRAME_HEAD  (0xAB)  /* 帧头 */

/* 类型定义 --------------------------------------------------------------*/

/* 一条可在线修改的变量 */
typedef struct {
    const char* name;         /* 上位机发送的名字 */
    float*      value;        /* 变量地址 */
    float       min;          /* 下限 */
    float       max;          /* 上限 */
    void      (*on_change)(void);  /* 改完后的回调，可为 NULL */
} SERIALPLOT_VAR;

typedef struct {
    UART_DRIVES* user_uart;
    uint8_t      checksum;                            /* 是否带校验和 */
    uint8_t      frame_header;
    const void*  data[SERIALPLOT_MAX_CH];             /* 待发送数据的地址 */
    uint8_t      data_num;
    const SERIALPLOT_VAR* vars;                       /* 在线可调变量表 */
    uint8_t      var_num;
    int8_t       var_index;                           /* 最近命中的变量号，未命中为 -1 */
    float        var_value;                           /* 最近解析出的值 */
    uint8_t      rx_buf[SERIALPLOT_RX_LEN];
    uint8_t      tx_buf[2 + SERIALPLOT_MAX_CH * 4];
    SysTick_Task task;
} SERIALPLOT_DRIVES;

/* 函数声明 --------------------------------------------------------------*/
void    Serialplot_Init(SERIALPLOT_DRIVES* sp, UART_DRIVES* user_uart,
                        const SERIALPLOT_VAR* vars, uint8_t var_num, uint32_t period_ms);

/* 指定本帧要发送的变量地址，全部按 float 发送 */
void    Serialplot_Set_Data(SERIALPLOT_DRIVES* sp, uint8_t number, ...);

int8_t  Serialplot_Get_Index(const SERIALPLOT_DRIVES* sp);
float   Serialplot_Get_Value(const SERIALPLOT_DRIVES* sp);

#endif /* HAL_UART_MODULE_ENABLED */
#endif // USER_SERIALPLOT_H
