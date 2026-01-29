#ifndef PROTO_WL2_H
#define PROTO_WL2_H

#include <stdint.h>

#pragma pack(push, 1)

// ===============================
// WL-2 Summary Packet (6 bytes)
// ===============================
//
// Byte 0-1 : Distance(12b) + Reserved(4b)
// Byte 2   : Lane
// Byte 3   : [7:4] Severity  / [3:0] Reserved
// Byte 4-5 : Timestamp (10ms tick)
//
// Total = 6 bytes
// ===============================
typedef struct {
    uint16_t dist_rsv;   // [15:4] Distance(m)  [3:0] Reserved
    uint8_t  lane;       // 차선 번호
    uint8_t  sev_rsv;    // [7:4] Severity  / [3:0] Reserved nibble
    uint16_t timestamp;  // 10ms tick
} wl2_packet_t;

#pragma pack(pop)

// ---------- size 검증 ----------
_Static_assert(sizeof(wl2_packet_t) == 6, "WL2 packet must be 6 bytes");


// ===============================
// Distance (12bit) helper
// ===============================
static inline uint16_t wl2_get_distance(const wl2_packet_t *p)
{
    return (p->dist_rsv >> 4) & 0x0FFF;   // 상위 12bit
}

static inline void wl2_set_distance(wl2_packet_t *p, uint16_t d)
{
    d &= 0x0FFF;
    p->dist_rsv = (p->dist_rsv & 0x000F) | (d << 4);
}


// ===============================
// Severity (upper nibble)
// ===============================
static inline uint8_t wl2_get_severity(const wl2_packet_t *p)
{
    return (p->sev_rsv >> 4) & 0x0F;
}

static inline void wl2_set_severity(wl2_packet_t *p, uint8_t s)
{
    s &= 0x0F;
    p->sev_rsv = (p->sev_rsv & 0x0F) | (s << 4);
}


// ===============================
// Reserved nibble (lower4bit)
// ===============================
static inline uint8_t wl2_get_reserved_low4(const wl2_packet_t *p)
{
    return p->sev_rsv & 0x0F;
}

static inline void wl2_set_reserved_low4(wl2_packet_t *p, uint8_t r)
{
    r &= 0x0F;
    p->sev_rsv = (p->sev_rsv & 0xF0) | r;
}

#endif


