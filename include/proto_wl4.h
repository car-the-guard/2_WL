#ifndef PROTO_WL4_H
#define PROTO_WL4_H

#include <stdint.h>



/**
 * WL-4 Vehicle Status Packet (Decision -> Wireless via UART)
 * Total = 8 bytes (STX/ETX 포함 프레임 방식)
 * -----------------------------------------------------------------------
 * Byte 0    : STX (8 bits)         - 0xFD 고정
 * Byte 1    : Type (8 bits)        - 0x04 고정 (TYPE_WL4)
 * Byte 2    : Reserved Pad (8 bits)- 바이트 패딩 (0x00)
 * Byte 3-4  : Timestamp (16 bits)  - ms 단위 타임스탬프
 * Byte 5-6  : Payload (16 bits)
 * [Bit 15:7] Direction (9 bits, 0-359 degrees) -> 상위 9비트
 * [Bit  6:0] Reserved  (7 bits)                -> 하위 7비트
 * Byte 7    : ETX (8 bits)         - 0xFE 고정
 * -----------------------------------------------------------------------
 */

#pragma pack(push, 1)
typedef struct {
    //uint32_t raw; // 비트 연산을 위해 32비트 통으로 관리
    uint8_t  stx;           // 0xFD 고정
    // --- Header (4 Bytes) ---
    uint8_t  type;            // WL-X 번호 (WL-4)
    uint8_t  reserved_pad;    // 바이트 패딩
    uint16_t timestamp;      // 시간 측정용 타임스탬프 (ms)

    // --- Payload (2 Bytes) ---
    // 상위 9비트를 direction으로 쓰고 싶다면:
    uint16_t reserved  : 7;  // [비트 0~6] 하위 7비트
    uint16_t direction : 9;  // [비트 7~15] 상위 9비트 (읽기 연산 시 유리)

    uint8_t  etx;           // 0xFE 고정

} wl4_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(wl4_packet_t) == 8, "WL4 packet must be 8 bytes");

// -------- Direction (상위 9비트 필드 접근) --------
static inline uint16_t wl4_get_direction(wl4_packet_t pkt)
{
    // 컴파일러가 내부적으로 (data >> 7) & 0x1FF 연산을 수행합니다.
    return (uint16_t)pkt.direction;
}

static inline void wl4_set_direction(wl4_packet_t *pkt, uint16_t dir)
{
    // 9비트(0~511) 범위를 초과하지 않도록 마스킹 후 대입
    pkt->direction = (dir & 0x01FF);
}

// -------- Timestamp (uint16_t 직접 접근) --------
static inline uint16_t wl4_get_timestamp(wl4_packet_t pkt)
{
    return pkt.timestamp;
}

static inline void wl4_set_timestamp(wl4_packet_t *pkt, uint16_t ts)
{
    pkt->timestamp = ts;
}

// -------- Reserved (하위 7비트 필드 접근) --------
static inline uint8_t wl4_get_reserved7(wl4_packet_t pkt)
{
    return (uint8_t)pkt.reserved;
}

static inline void wl4_set_reserved7(wl4_packet_t *pkt, uint8_t res)
{
    pkt->reserved = (res & 0x7F);
}
#endif
