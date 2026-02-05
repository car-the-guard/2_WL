#ifndef PROTO_WL3_H
#define PROTO_WL3_H

#include <stdint.h>



/**
 * WL-3 My Accident Packet (Decision -> Wireless via UART)
 * Total = 26 bytes (STX/ETX 프레임 포함)
 * -----------------------------------------------------------------------
 * 
 * Byte 0      : STX (8 bits)         - 0xFD 고정
 [Header - 4B]
 * 
 * Byte 1      : Type (8 bits)        - 0x03 고정 (TYPE_WL3)
 * Byte 2      : Reserved Pad (8 bits)- 바이트 패딩 (0x00)
 * Byte 3-4    : Timestamp (16 bits)  - ms 단위 타임스탬프
 * * [Payload - 20B]
 * Byte 5-6    : Direction/Reserved (16 bits)
 * [Bit 15:7] Direction (9 bits, 0-359 degrees) -> 상위 9비트
 * [Bit  6:0] Reserved  (7 bits)                -> 하위 7비트
 * Byte 7      : Lane (8 bits)        - 차선 번호
 * Byte 8      : Severity (8 bits)    - 위험도
 * Byte 9-16   : Accident ID (64 bits)- Decision이 결정한 고유 ID
 * Byte 17-24  : Accident Time (64 bits)- 사고 발생 시간
 * * [Trailer - 1B]
 * Byte 25     : ETX (8 bits)         - 0xFE 고정
 * -----------------------------------------------------------------------
 */

#pragma pack(push, 1)
typedef struct {
    // --- 프레임 헤더 ---
    uint8_t  stx;             // 0xFD

    uint8_t  type;            // 0x03 (TYPE_WL3)
    uint8_t  reserved_pad;    // 바이트 패딩
    uint16_t timestamp;       // ms 단위 타임스탬프
    
    

    // --- 데이터 필드 (20 Bytes) ---
    // [WL-4와 동일 방식] 하위 7비트 reserved, 상위 9비트 direction
    //uint16_t direction;  //[15:7] direction(9b) / [6:0] reserved
    
    //uint16_t reserved : 7;   // Bit 0 ~ 6
    //uint16_t direction : 9;   // Bit 7 ~ 15 (상위 9비트)
    uint16_t direction;  
    uint8_t  lane;
    uint8_t  severity;      

    
    uint64_t accident_time;   // Decision이 기록한 시간
    uint64_t accident_id;     // Decision이 결정한 ID
    
    // --- 프레임 트레일러 ---
    uint8_t  etx;             // 0xFE
} wl3_packet_t;

#pragma pack(pop)

// ---------- 크기 보증 (STX + Header 4B + Payload 20B + ETX = 26B) ----------
_Static_assert(sizeof(wl3_packet_t) == 26, "WL3 packet must be 26 bytes");


// ===================================================
// Helper functions (권장 사용 방식)
// ===================================================

/**
 * @brief Direction 설정 (상위 9비트)
 * 구조체 멤버가 이미 9비트로 제한되어 있어 직접 대입이 가능합니다.
 */
static inline void wl3_set_direction(wl3_packet_t *pkt, uint16_t dir) {
    if (pkt) {
        //pkt->direction = (dir & 0x01FF); // 9비트(0~511) 마스킹 후 대입
        pkt->direction = dir;
        // htons를 사용하여 30(0x001E)을 [00][1E] 순서로 변경
        //pkt->direction = htons(dir);
    }
}

/*
 * @brief Direction 읽기
 */
static inline uint16_t wl3_get_direction(const wl3_packet_t *pkt) {
    return pkt ? pkt->direction : 0;
}



// severity가 1바이트 통으로 있으므로 별도의 비트 연산 없이 직접 접근하면 되지만,
// 기존 코드와의 호환성을 위해 래퍼 함수를 유지합니다.
static inline void wl3_set_severity(wl3_packet_t *pkt, uint8_t sev) {
    if (pkt) pkt->severity = sev;
}

static inline uint8_t wl3_get_severity(const wl3_packet_t *pkt) {
    return pkt ? pkt->severity : 0;
}

#endif

