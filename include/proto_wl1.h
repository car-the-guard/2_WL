#ifndef PROTO_WL1_H
#define PROTO_WL1_H

#include <stdint.h>

#pragma pack(push, 1)

// HEADER (4B)
typedef struct {
    uint8_t version;
    uint8_t msg_type;
    uint8_t ttl;
    uint8_t reserved;
} wl1_header_t;


// SENDER (24B → 그대로 유지)
typedef struct {
    uint32_t sender_id;
    //uint64_t send_time;

    int32_t  lat_uDeg;   // 위도 (micro-degree)
    int32_t  lon_uDeg;   // 경도 (micro-degree)
    int32_t  alt_mm;     // 고도 (mm 단위 권장 or meter*1000)
    uint64_t send_time;
} wl1_sender_t;


// ACCIDENT (36B → 그대로 유지)
typedef struct {
    uint16_t direction;
    uint16_t reservedA;

    uint64_t accident_time;
    uint64_t accident_id;

    uint8_t  type;
    uint8_t  sev_action;
    uint8_t  lane;
    uint8_t  reservedB;

    int32_t  lat_uDeg;   // 사고지점 위도
    int32_t  lon_uDeg;   // 사고지점 경도
    int32_t  alt_mm;     // 사고 고도
} wl1_accident_t;


// SECURITY (64B)
typedef struct {
    uint8_t signature[64];
} wl1_security_t;


// TOTAL = 128 Bytes (동일)
typedef struct {
    wl1_header_t   header;
    wl1_sender_t   sender;
    wl1_accident_t accident;
    wl1_security_t security;
} wl1_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(wl1_packet_t) == 128, "WL1 size must be 128");

#endif

