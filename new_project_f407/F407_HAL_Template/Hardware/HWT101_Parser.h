#ifndef HWT101_PARSER_H
#define HWT101_PARSER_H
#include <stdint.h>
#define HWT101_TIMEOUT_MS 500U
#define HWT101_FRAME_GAP_MS 20U

typedef struct {
    float yaw_deg;
    float heading_deg;
    float continuous_deg;
    float yaw_rate_dps; /* Finite difference of yaw, not the raw 0x52 gyro. */
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t valid;
    uint8_t continuous_valid;
    uint8_t rate_valid;
} HWT101_Data;

typedef struct {
    HWT101_Data data;
    uint8_t frame[11];
    uint8_t length;
    uint8_t received;
    uint8_t broken;
    uint32_t byte_ms;
    float zero_deg;
} HWT101_Parser;

void HWT101_ParserInit(HWT101_Parser *parser);
void HWT101_ParserFeed(HWT101_Parser *parser, uint8_t byte, uint32_t ms);
void HWT101_ParserExpire(HWT101_Parser *parser, uint32_t now);
void HWT101_ParserBreak(HWT101_Parser *parser);
uint8_t HWT101_ParserZero(HWT101_Parser *parser, uint32_t now);
/* Finite angles in degrees; +180 tie resolves to -180. */
float HWT101_AngleError(float target_deg, float current_deg);
#endif
