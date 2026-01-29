#ifndef VAL_H
#define VAL_H

#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>

/* 1. 내 차량의 실시간 통합 상태 구조체 (T5, T9에서 갱신) */
typedef struct {
    double latitude;    // 위도 (degree)
    double longitude;   // 경도 (degree)
    double altitude;    // 고도 (meter 단위 추천)
    double heading;     // 진행 방향 (지자기 센서 값, 0~359도)
    
    pthread_mutex_t lock;
} vehicle_status_t;

/* 2. 전역 변수 참조 */
extern vehicle_status_t g_vehicle_status;
extern volatile bool g_keep_running;

/* 3. 주요 스레드 함수 선언 */
void *thread_val(void *arg); // T4 스레드

/* 4. 헬퍼 함수 선언 (val.c 구현용) */
// 지리적/방향성 판단을 위한 수학 함수들
double calculate_2d_dist(double lat1, double lon1, double lat2, double lon2);
int get_angle_diff(int head1, int head2);
bool is_same_level(double my_alt, double target_alt);

#endif
