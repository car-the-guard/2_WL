#ifndef PROTO_WL1_H
#define PROTO_WL1_H

#include <stdint.h>

#pragma pack(push, 1)

// [1] HEADER (4B)
typedef struct {
    uint8_t version;    // 메시지 버전
    uint8_t msg_type;   // 메시지 종류
    uint8_t ttl;        // TTL
    uint8_t reserved;   // 예약 영역
} wl1_header_t;

// [2] SENDER INFO (28B)
// 이미지 순서: ID(4) -> Time(8) -> Lat(4) -> Lon(4) -> Alt(4) + Pad(4)
typedef struct {
    uint32_t sender_id;  // 송신자 ID
    uint64_t send_time;  // 재송신 시각
    int32_t  lat_uDeg;   // 위도 (마이크로 Deg)
    int32_t  lon_uDeg;   // 경도 (마이크로 Deg)
    int32_t  alt_mm;     // 고도 (마이크로 Deg/mm)
    uint8_t  reserved[4]; // 28B를 맞추기 위한 패딩
} wl1_sender_t;

// [3] ACCIDENT INFO (32B)
// 이미지 순서: Dir(2) -> Lane(1) -> Sev(1) -> Time(8) -> ID(8) -> GPS(12)
typedef struct {
    uint16_t direction;     // 사고 차량의 지자기 Heading
    uint8_t  lane;          // 차선 번호
    uint8_t  severity;      // 위험도
    uint64_t accident_time; // 사고 발생 시간
    uint64_t accident_id;   // 사고 ID
    
    int32_t  lat_uDeg;      // 사고지점 위도
    int32_t  lon_uDeg;      // 사고지점 경도
    int32_t  alt_mm;        // 사고지점 고도
} wl1_accident_t;

// [4] SECURITY (192B)
// 기존 64바이트에서 192바이트로 확장
typedef struct {
    uint8_t signature[192];
} wl1_security_t;

// [TOTAL] 4 + 28 + 32 + 192 = 256 Bytes
typedef struct {
    wl1_header_t   header;   // 4B
    wl1_sender_t   sender;   // 28B
    wl1_accident_t accident; // 32B
    wl1_security_t security; // 192B
} wl1_packet_t;

// 해당 WL 패킷을 시스템시간 기준 몇 ms에 송신할 것인지 정보를 함께 담은 구조체
// 여기의 target_send_time_ms가 WL1 패킷이 실제로 외부로 나가는 목표 시간임
// 얼마나 딜레이 될 지가 아닌, 정확한 송신 시각을 지정
// 이 시간이 안되었다면 대기. 지나갔다면 즉시 송신.
typedef struct {
    wl1_packet_t packet;
    uint32_t target_send_time_ms;
} wl1_delayed_packet_t;

#pragma pack(pop)

_Static_assert(sizeof(wl1_packet_t) == 256, "WL1 size must be 256");

#endif

/*
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
*/