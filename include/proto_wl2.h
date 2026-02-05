#ifndef PROTO_WL2_H
#define PROTO_WL2_H

#include <stdint.h>



/**
 * WL-2 Summary Packet (UART Interface)
 * -----------------------------------------------------------------------
 * [Header - 4B]
 * Byte 0    : STX (0xFD)
 * Byte 1    : TYPE (0x02 고정)
 * Byte 2    : Reserved (바이트 패딩 0x00)
 * Byte 3-4  : Timestamp (2B, ms 단위) -> 이미지상 Header 영역 포함
 * * [Payload - 4B]
 * Byte 5-6  : Distance (2B, 사고 지점까지의 직선거리)
 * Byte 7    : Lane (1B, 사고 차선 번호)
 * Byte 8    : Severity (1B, 사고 규모/위험도)
 * * [Trailer - 1B]
 * Byte 9    : ETX (0xFE)
 * -----------------------------------------------------------------------
 */

#pragma pack(push, 1)
typedef struct {
    // --- Header (5B로 보이나 이미지상 Header 4B 구성에 맞춰 배치) ---
    uint8_t  stx;           // 0xFD
    uint8_t  type;          // 0x02 (WL-2)
    uint8_t  reserved_pad;  // 바이트 패딩
    uint16_t timestamp;     // 시간 측정용 타임스탬프 (ms)

    // --- Payload (4B) ---
    uint16_t distance;      // 사고 지점까지의 직선거리 (2Byte)
    uint8_t  lane;          // 사고 차선 번호 (1Byte)
    uint8_t  severity;      // 사고 규모 (1Byte)

    // --- Trailer ---
    uint8_t  etx;           // 0xFE
} wl2_packet_t;
#pragma pack(pop)

// ---------- size 검증 ----------
_Static_assert(sizeof(wl2_packet_t) == 10, "WL2 packet must be 10 bytes");


// ===============================
// Helper functions 
// ===============================

static inline void wl2_set_distance(wl2_packet_t *p, uint16_t dist) {
    if (p) p->distance = dist;
}

static inline uint16_t wl2_get_distance(const wl2_packet_t *p) {
    return p ? p->distance : 0;
}

static inline void wl2_set_severity(wl2_packet_t *p, uint8_t sev) {
    if (p) p->severity = sev;
}

static inline uint8_t wl2_get_severity(const wl2_packet_t *p) {
    return p ? p->severity : 0;
}

#endif


