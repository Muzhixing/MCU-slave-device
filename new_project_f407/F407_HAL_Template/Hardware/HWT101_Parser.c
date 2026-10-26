#include "HWT101_Parser.h"
#include <math.h>
#include <string.h>

float HWT101_AngleError(float target_deg, float current_deg)
{
    float error = fmodf(target_deg - current_deg + 180.0f, 360.0f);
    if (error < 0.0f) error += 360.0f;
    return error - 180.0f;
}

void HWT101_ParserInit(HWT101_Parser *p)
{
    memset(p, 0, sizeof(*p));
}

void HWT101_ParserBreak(HWT101_Parser *p)
{
    p->length = 0;
    p->broken = 1U;
    p->data.valid = 0;
    p->data.rate_valid = 0;
    p->data.continuous_valid = 0;
}

void HWT101_ParserExpire(HWT101_Parser *p, uint32_t now)
{
    if (p->data.valid && (uint32_t)(now - p->data.timestamp_ms) >= HWT101_TIMEOUT_MS)
        HWT101_ParserBreak(p);
}

uint8_t HWT101_ParserZero(HWT101_Parser *p, uint32_t now)
{
    HWT101_ParserExpire(p, now);
    if (!p->data.valid) return 0U;
    p->zero_deg = p->data.yaw_deg;
    p->data.heading_deg = 0.0f;
    p->data.continuous_deg = 0.0f;
    p->data.continuous_valid = 1U;
    p->broken = 0U;
    return 1U;
}

static void HWT101_AcceptAngle(HWT101_Parser *p, uint32_t ms)
{
    uint16_t word = (uint16_t)((uint16_t)p->frame[7] << 8) | p->frame[6];
    int32_t raw = (word < 32768U) ? (int32_t)word : (int32_t)word - 65536L;
    float yaw = (float)raw * (180.0f / 32768.0f);
    float delta;
    uint32_t dt = ms - p->data.timestamp_ms;
    p->data.rate_valid = 0U;
    p->data.yaw_rate_dps = 0.0f;
    if (!p->received)
    {
        p->zero_deg = yaw;
        p->data.continuous_valid = (uint8_t)!p->broken;
    }
    else
    {
        if (dt >= HWT101_TIMEOUT_MS) p->broken = 1U;
        if (p->broken) p->data.continuous_valid = 0U;
        if (!p->broken)
        {
            delta = HWT101_AngleError(yaw, p->data.yaw_deg);
            p->data.continuous_deg += delta;
            if (dt != 0U)
            {
                p->data.yaw_rate_dps = delta * 1000.0f / (float)dt;
                p->data.rate_valid = 1U;
            }
        }
    }
    p->received = 1U;
    p->data.yaw_deg = yaw;
    p->data.heading_deg = HWT101_AngleError(yaw, p->zero_deg);
    p->data.timestamp_ms = ms;
    p->data.sequence++;
    p->data.valid = 1U;
}

void HWT101_ParserFeed(HWT101_Parser *p, uint8_t byte, uint32_t ms)
{
    uint8_t sum;
    uint8_t i;
    if (p->length && (uint32_t)(ms - p->byte_ms) > HWT101_FRAME_GAP_MS)
        p->length = 0U;
    p->byte_ms = ms;
    p->frame[p->length++] = byte;
    /* Sliding resynchronization preserves a header embedded in a bad frame. */
    while (p->length)
    {
        if (p->frame[0] != 0x55U ||
            (p->length >= 2U && p->frame[1] != 0x52U && p->frame[1] != 0x53U))
        {
            p->length--;
            memmove(p->frame, p->frame + 1, p->length);
            continue;
        }
        if (p->length < 11U) return;
        sum = 0U;
        for (i = 0U; i < 10U; i++) sum = (uint8_t)(sum + p->frame[i]);
        if (sum == p->frame[10])
        {
            if (p->frame[1] == 0x53U) HWT101_AcceptAngle(p, ms);
            p->length = 0U;
            return;
        }
        p->length--;
        memmove(p->frame, p->frame + 1, p->length);
    }
}
