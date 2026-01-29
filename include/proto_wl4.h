#ifndef PROTO_WL4_H
#define PROTO_WL4_H

#include <stdint.h>

#pragma pack(push, 1)

/**
 * WL-4 Vehicle Status Packet (Yocto -> RPi via I2C)
 * Total = 4 bytes (32-bit unsigned int)
 * --------------------------------------------------
 * Bit 31-25 : Reserved (7 bits)
 * Bit 24-16 : Direction (9 bits, 0-359 degrees)
 * Bit 15-0  : Timestamp (16 bits, millisecond unit)
 * --------------------------------------------------
 */
typedef struct {
    uint32_t raw; // 비트 연산을 위해 32비트 통으로 관리
} wl4_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(wl4_packet_t) == 4, "WL4 packet must be 4 bytes");

// ===================================================
// Helper functions (비트 추출 및 설정)
// ===================================================

// -------- Direction (Bit 24-16) --------
static inline uint16_t wl4_get_direction(wl4_packet_t pkt)
{
    // 16비트 시프트 후 9비트 마스킹 (0x01FF)
    return (uint16_t)((pkt.raw >> 16) & 0x01FF);
}
static inline void wl4_set_direction(wl4_packet_t *pkt, uint16_t dir)
{
    dir &= 0x01FF; // 9비트 제한
    pkt->raw = (pkt->raw & ~(0x01FF << 16)) | ((uint32_t)dir << 16);
}

// -------- Timestamp (Bit 15-0) --------
static inline uint16_t wl4_get_timestamp(wl4_packet_t pkt)
{
    // 하위 16비트 마스킹
    return (uint16_t)(pkt.raw & 0xFFFF);
}

static inline void wl4_set_timestamp(wl4_packet_t *pkt, uint16_t ts)
{
    pkt->raw = (pkt->raw & ~0xFFFF) | (ts & 0xFFFF);
}

// -------- Reserved (Bit 31-25) --------
static inline uint8_t wl4_get_reserved7(wl4_packet_t pkt)
{
    return (uint8_t)((pkt.raw >> 25) & 0x7F);
}

#endif
