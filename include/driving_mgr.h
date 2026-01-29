#ifndef DRIVING_MGR_H
#define DRIVING_MGR_H

#include <pthread.h>
#include "proto_wl4.h"
#include "gps.h"        // gps_data_t 정의를 위해 필수
// 주행 상태 통합 구조체
typedef struct {
    double   lat;       // GPS 데이터
    double   lon;       // GPS 데이터
    double  alt;       // GPS 데이터
    uint16_t heading;   // WL-4 데이터
    pthread_mutex_t lock;
} driving_status_t;

// 외부에서 접근 가능하도록 선언
extern driving_status_t g_driving_status;

// 스레드 함수
void* thread_driving_manager(void* arg);

#endif
