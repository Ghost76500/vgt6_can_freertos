#include "rc_ibus.h"
#include <string.h>
#include "UART_receive.h"

/* 常见 iBUS 帧头 */
#define IBUS_LEN_BYTE   0x20
#define IBUS_CMD_BYTE   0x40

static uint16_t ibus_checksum(const uint8_t *buf)
{
    // csum = 0xFFFF - sum(buf[0..29])
    uint16_t csum = 0xFFFF;
    for (int i = 0; i < RC_IBUS_FRAME_LEN - 2; i++) {
        csum -= buf[i];
    }
    return csum;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float norm_1000_2000(uint16_t v, float deadzone)
{
    // 1500 中位；500 半幅
    float x = ((float)v - 1500.0f) / 500.0f;
    x = clampf(x, -1.0f, 1.0f);
    if (x > -deadzone && x < deadzone) x = 0.0f;
    return x;
}

static uint8_t ibus_decode_frame(rc_ibus_handle_t *h, const uint8_t *buf)
{
    if (buf[0] != IBUS_LEN_BYTE) return 0;
    if (buf[1] != IBUS_CMD_BYTE) return 0;

    uint16_t rx = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
    uint16_t cal = ibus_checksum(buf);
    if (rx != cal) return 0;

    for (int i = 0; i < RC_IBUS_CH_MAX; i++) {
        int off = 2 + i * 2;
        uint16_t v = (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
        h->data.ch_raw[i]  = v;
        h->data.ch_norm[i] = norm_1000_2000(v, h->deadzone);
    }

    h->data.last_ok_ms = HAL_GetTick();
    h->data.frame_ok = 1;
    return 1;
}

/* 字节流同步器：自动处理断帧/粘包/错位 */
static void ibus_feed_byte(rc_ibus_handle_t *h, uint8_t b)
{
    if (h->sync_idx == 0) {
        if (b == IBUS_LEN_BYTE) {
            h->frame_buf[h->sync_idx++] = b;
        }
        return;
    }

    if (h->sync_idx == 1) {
        if (b == IBUS_CMD_BYTE) {
            h->frame_buf[h->sync_idx++] = b;
        } else {
            // 尝试重新同步：如果当前字节又是 0x20，则认为是新帧起始
            h->sync_idx = 0;
            if (b == IBUS_LEN_BYTE) {
                h->frame_buf[h->sync_idx++] = b;
            }
        }
        return;
    }

    // sync_idx >= 2
    h->frame_buf[h->sync_idx++] = b;

    if (h->sync_idx >= RC_IBUS_FRAME_LEN) {
        (void)ibus_decode_frame(h, h->frame_buf);
        h->sync_idx = 0;
    }
}

void RC_IBUS_Init(rc_ibus_handle_t *h, float deadzone, uint32_t timeout_ms)
{
    memset(h, 0, sizeof(*h));
    h->deadzone = (deadzone < 0.0f) ? 0.0f : deadzone;
    h->timeout_ms = timeout_ms;

    for (int i = 0; i < RC_IBUS_CH_MAX; i++) {
        h->data.ch_raw[i] = 1500;
        h->data.ch_norm[i] = 0.0f;
    }
    h->data.last_ok_ms = 0;
    h->data.frame_ok = 0;
    h->sync_idx = 0;
}

/*
 * 轮询读取：每次调用尽可能把缓冲区里的数据读空
 * 你的 Command_GetCommand() 是“单字节读出”风格：返回 1 表示读到一个字节，0 表示没数据
 */
void RC_IBUS_Poll(rc_ibus_handle_t *h)
{
    uint8_t b;
    while (Command_GetCommand(&b)) {
        ibus_feed_byte(h, b);
    }
}

uint8_t RC_IBUS_IsLost(const rc_ibus_handle_t *h)
{
    uint32_t now = HAL_GetTick();
    // 如果从未成功解帧，则认为 lost
    if (h->data.last_ok_ms == 0) return 1;
    return (now - h->data.last_ok_ms) > h->timeout_ms;
}

uint8_t RC_IBUS_Switch2Pos(const rc_ibus_handle_t *h, uint8_t ch)
{
    uint16_t v = RC_IBUS_GetRaw(h, ch);
    return (v >= 1500) ? 1 : 0;
}

uint8_t RC_IBUS_Switch3Pos(const rc_ibus_handle_t *h, uint8_t ch)
{
    uint16_t v = RC_IBUS_GetRaw(h, ch);
    if (v < 1300) return 0;
    if (v > 1700) return 2;
    return 1;
}
