#ifndef PROTO_WL3_H
#define PROTO_WL3_H

#include <stdint.h>



// ======================================
// WL-3 Debug / Yocto Internal Packet
// Total = 23 bytes
// ======================================
//
// Byte 0-1   : debug_time (16b)
// Byte 2   : accident_type (8b)
// Byte 3   : [7:4] severity / [3:0] action
// Byte 4   : lane (8b)
//
// Byte 5-6 : [15:7] direction(9b) / [6:0] reserved
//
// Byte 7-14 : accident_time (64b)
// Byte 15-22 :  accident_id      // 8 bytes (Yocto가 생성한 고유 ID)
// ======================================
#pragma pack(push, 1)
typedef struct {
    // --- 프레임 헤더 ---
    uint8_t  stx;             // 0xFD

    uint8_t  type;            // 0x03 (TYPE_WL3)
    uint8_t  reserved_pad;    // 바이트 패딩
    uint16_t timestamp;       // ms 단위 타임스탬프
    
    // --- 데이터 필드 ---
    uint16_t direction;  //[15:7] direction(9b) / [6:0] reserved
    uint8_t  lane;
    uint8_t  severity;      

    uint64_t accident_id;     // Decision이 결정한 ID
    uint64_t accident_time;   // Decision이 기록한 시간
    
    // --- 프레임 트레일러 ---
    uint8_t  etx;             // 0xFE
} wl3_packet_t;

#pragma pack(pop)

// ---------- 크기 보증 ----------
_Static_assert(sizeof(wl3_packet_t) == 23, "WL3 packet must be 23 bytes");


// ===================================================
// Helper functions (권장 사용 방식)
// ===================================================

// -------- Severity / Action --------
static inline uint8_t wl3_get_severity(uint8_t sev_action)
{
    return (sev_action >> 4) & 0x0F;
}

static inline uint8_t wl3_get_action(uint8_t sev_action)
{
    return sev_action & 0x0F;
}

static inline void wl3_set_severity(uint8_t *sev_action, uint8_t s)
{
    s &= 0x0F;
    *sev_action = (*sev_action & 0x0F) | (s << 4);
}

static inline void wl3_set_action(uint8_t *sev_action, uint8_t a)
{
    a &= 0x0F;
    *sev_action = (*sev_action & 0xF0) | a;
}


// -------- Direction / Reserved --------
// direction = upper 9 bits
static inline uint16_t wl3_get_direction(uint16_t dir_rsv)
{
    return (dir_rsv >> 7) & 0x01FF;   // 0–511
}

static inline uint8_t wl3_get_reserved7(uint16_t dir_rsv)
{
    return dir_rsv & 0x7F;
}

static inline void wl3_set_direction(uint16_t *dir_rsv, uint16_t d)
{
    d &= 0x01FF;
    *dir_rsv = (*dir_rsv & 0x007F) | (d << 7);
}

static inline void wl3_set_reserved7(uint16_t *dir_rsv, uint8_t r)
{
    r &= 0x7F;
    *dir_rsv = (*dir_rsv & 0xFF80) | r;
}

#endif

