#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <pthread.h>

// [1] GPS 데이터를 담는 표준 구조체 정의
// driving_mgr.c에서 gps_raw.lat, gps_raw.lon 등으로 접근할 수 있게 합니다.
typedef struct {
    double lat;       // 위도
    double lon;       // 경도
    double alt;       // 고도
    float  speed;     // 속도 (필요 시)
    float  course;    // 진행 방향 (GPS 기준)
} gps_data_t;

// [2] 외부(main.c, driving_mgr.c)에서 호출할 함수 선언


/**
 * @brief 가장 최신의 GPS 데이터를 반환합니다.
 * @return gps_data_t 구조체
 */
gps_data_t GPS_get_latest(void);

/**
 * @brief GPS 모듈 또는 gpsd 클라이언트를 초기화합니다.
 */
void GPS_init(void);

/**
 * @brief GPS 데이터를 주기적으로 업데이트하는 스레드 함수
 */
void* gps_thread(void* arg);

#endif // GPS_H
