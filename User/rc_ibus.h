#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_IBUS_CH_MAX      14
#define RC_IBUS_FRAME_LEN   32

typedef struct {
    uint16_t ch_raw[RC_IBUS_CH_MAX];   // 典型 1000~2000
    float    ch_norm[RC_IBUS_CH_MAX];  // -1~1（含死区）
    uint32_t last_ok_ms;               // 最近一次成功解帧的 tick
    uint8_t  frame_ok;                 // 最近一次是否解帧成功
} rc_ibus_t;

typedef struct {
    rc_ibus_t data;

    // 帧同步器
    uint8_t frame_buf[RC_IBUS_FRAME_LEN];
    uint8_t sync_idx;

    // 参数
    float deadzone;        // 0.05~0.08 推荐
    uint32_t timeout_ms;   // 200~500ms 推荐
} rc_ibus_handle_t;

/* 你现有的缓冲接口（由 UART_receive.c 提供） */
uint8_t Command_GetCommand(uint8_t *command);

/* 初始化 */
void RC_IBUS_Init(rc_ibus_handle_t *h, float deadzone, uint32_t timeout_ms);

/* 轮询：从 Command_GetCommand 拉取字节并解析（建议 1~10ms 调一次） */
void RC_IBUS_Poll(rc_ibus_handle_t *h);

/* 掉控判断 */
uint8_t RC_IBUS_IsLost(const rc_ibus_handle_t *h);

/* 读取通道 */
static inline uint16_t RC_IBUS_GetRaw(const rc_ibus_handle_t *h, uint8_t ch) {
    return (ch < RC_IBUS_CH_MAX) ? h->data.ch_raw[ch] : 1500;
}
static inline float RC_IBUS_GetNorm(const rc_ibus_handle_t *h, uint8_t ch) {
    return (ch < RC_IBUS_CH_MAX) ? h->data.ch_norm[ch] : 0.0f;
}

/* 两段/三段开关 */
uint8_t RC_IBUS_Switch2Pos(const rc_ibus_handle_t *h, uint8_t ch);
uint8_t RC_IBUS_Switch3Pos(const rc_ibus_handle_t *h, uint8_t ch);

#ifdef __cplusplus
}
#endif
