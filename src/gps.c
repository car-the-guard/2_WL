#include "gps.h"
#include "val.h"
#include "debug.h"
#include <unistd.h>
#include <math.h>
#include "driving_mgr.h"

extern volatile bool g_keep_running;
extern driving_status_t g_driving_status;


extern double g_accident_point = 37.5690;

// [1] GPS 초기화 함수 구현
void GPS_init(void) {
    DBG_INFO("GPS: Mock Mode Initialized.");
}

// [2] 최신 데이터 반환 함수 구현 
gps_data_t GPS_get_latest(void) {
    gps_data_t data;
    
    // 현재 전역 상태에 업데이트된 값을 가져오기
    pthread_mutex_lock(&g_driving_status.lock);
    data.lat = g_driving_status.lat;
    data.lon = g_driving_status.lon;
    data.alt = (double)g_driving_status.alt;
    pthread_mutex_unlock(&g_driving_status.lock);
    
    return data;
}

/**
 * GPS 정보를 수집하여 전역 차량 상태를 업데이트하는 스레드
 */
void *thread_gps(void *arg) {
    (void)arg;
    DBG_INFO("Thread 7: GPS Client Module started.");

    double current_lat = 37.5675; 
    double current_lon = 126.9780;
    double current_alt = 15.0;

    /*while (g_keep_running) {
        /* * [확장 포인트]
         * 1. 실제 gpsd 연동 시: gps_read() 등을 통해 real data 획득
         * 2. 시뮬레이션 시: 아래와 같이 가상 좌표 이동
         */
       /* current_lat += 0.00001; // 가상 이동 시뮬레이션

        // VAL 모듈의 공유 자원 보호를 위한 뮤텍스 잠금
        pthread_mutex_lock(&g_driving_status.lock);
        
        g_driving_status.lat = current_lat;
        g_driving_status.lon = current_lon;
        g_driving_status.alt = current_alt; 
        pthread_mutex_unlock(&g_driving_status.lock);

        // 너무 잦은 업데이트 방지 (10Hz: 100ms)
        usleep(1000000); 
       
        
        // 10초마다 로그 출력
        static int count = 0;
        if (++count % 100 == 0) {
            printf("GPS: Moving...");
            printf("GPS: Current Pos (Lat: %.6f, Lon: %.6f)", current_lat, current_lon);
        }
    }
    */
    while (g_keep_running) {
        pthread_mutex_lock(&g_driving_status.lock);
        
        // 사고 지점(37.5690) 도달 전까지 1초에 약 11m씩 전진
        if (current_lat < g_accident_point) {
            current_lat += 0.0001; 
            if (current_lat >= g_accident_point) {
                current_lat = g_accident_point;
                //printf("\n[GPS-차1] ★ 사고 지점 도착! (Lat: %.6f)\n", current_lat);
            }
        }
        
        g_driving_status.lat = current_lat;
        g_driving_status.lon = 126.9780;
        g_driving_status.alt = 15.0;
        
        pthread_mutex_unlock(&g_driving_status.lock);

        if (current_lat >= g_accident_point) {
            
            static int arrived = 0;
            if (arrived == 0) {
                //printf("\n[GPS-차1] 사고 지점 도착! 주행을 멈추고 사고 모드로 전환.\n");
                arrived = 1;
            }
        }
        
        usleep(1000000); // 1초 주행 루프
    }
    DBG_INFO("Thread 7: GPS Client Module terminating.");
    return NULL;
}
