#ifndef VAL_MSG_H
#define VAL_MSG_H

#include <stdint.h>
#include <stdbool.h>
#include "proto_wl1.h" // wl1_packet_t 정의 포함

// RPi 분석 데이터
typedef struct {
    double   dist_3d;           // 3D 거리
    uint16_t target_bearing;    // 방위각
    bool     is_danger;         // 위험 여부
} analysis_data_t;

// WL-2_ 최종 데이터 구조
typedef struct {
    wl1_accident_t  accident;   // 원본 사고 정보
    analysis_data_t analysis;   // 분석 결과
} wl2_data_t;

#endif
