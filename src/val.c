#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "val.h"
#include "queue.h"
#include "debug.h"
#include "val_msg.h"
#include "driving_mgr.h"
#include "proto_wl2.h"
#include "i2c_io.h"

#define MAX_ACCIDENTS 20
#define DIST_LIMIT    1000.0   // 1km 유효거리
#define ALT_LIMIT     25.0      // 5m 고도차 필터, 현재 기본 고도 15m (교량/지하도로 구분용)
#define TIMEOUT_SEC   5        // // 5초간 수신 없으면 리스트에서 삭제
#define HEADING_LIMIT 45       // 45도 이내 차이만 동일 방향으로 간주
extern volatile bool g_keep_running;
extern driving_status_t g_driving_status;
extern uint32_t g_sender_id;

extern queue_t q_pkt_val, q_val_pkt_tx, q_val_yocto;


typedef struct {
    wl2_data_t data;
    uint64_t   last_seen_ms;
    bool       is_active;
} managed_accident_t;

managed_accident_t accident_list[MAX_ACCIDENTS];

uint64_t get_now_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

double calc_dist(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * 111319.9;
    double dlon = (lon2 - lon1) * 88804.0;
    return sqrt(dlat * dlat + dlon * dlon);
}


int get_angle_diff(int a, int b) {
    int diff = a - b;
    if (diff < 0) diff = -diff; // 정수 절대값 처리
    
    if (diff > 180) {
        diff = 360 - diff;
    }
    return diff;
}

void *thread_val(void *arg) {
    (void)arg;
    DBG_INFO("Thread 4: VAL Controller started.");
    memset(accident_list, 0, sizeof(accident_list));
    
    uint64_t last_report_ms = 0;

    while (g_keep_running) {
        // [1] 패킷 처리
        wl1_packet_t *rx = Q_pop(&q_pkt_val);
    
        if (rx) {
            printf("[DEBUG-VAL] >>> VAL Controller Got Data! From: 0x%X\n", rx->sender.sender_id);
            
            // 1. 내 현재 상태 스냅샷 가져오기
            pthread_mutex_lock(&g_driving_status.lock);
            double my_lat = g_driving_status.lat;
            double my_lon = g_driving_status.lon;
            double my_alt = g_driving_status.alt;
            int my_heading = g_driving_status.heading;
            pthread_mutex_unlock(&g_driving_status.lock);

            // 2. 상대방 데이터 파싱
            int32_t raw_lat = (int32_t)rx->accident.lat_uDeg; 
            int32_t raw_lon = (int32_t)rx->accident.lon_uDeg;
            int32_t raw_alt_mm = (int32_t)rx->accident.alt_mm;
            int acc_heading = rx->accident.direction;

            double target_lat = (double)raw_lat / 1000000.0;
            double target_lon = (double)raw_lon / 1000000.0;
            double target_alt = (double)raw_alt_mm / 1000.0;

            double dist_2d = calc_dist(my_lat, my_lon, target_lat, target_lon);
            double alt_diff = fabs(my_alt - target_alt);
            int head_diff = get_angle_diff(my_heading, acc_heading);

            printf("[CHECK-VAL] 수신: From 0x%X, 거리:%.2fm, 고도차:%.1fm, 방향차:%d도\n", 
                    rx->sender.sender_id, dist_2d, alt_diff, head_diff);
            
            // 유효 범위 및 방향성 필터링
            // 내 패킷 제외 && 거리/고도 유효
            if (rx->sender.sender_id != g_sender_id && dist_2d < DIST_LIMIT && alt_diff < ALT_LIMIT) {
                
                // 동일 방향성 확인 (45도 이내)
                if (head_diff <= HEADING_LIMIT) {
                    int mode = (dist_2d > 500.0) ? MODE_ALERT : MODE_RELAY;
                    uint16_t display_dist = (uint16_t)dist_2d;

                    printf("\x1b[1;32m[VAL-WARN] 동일 방향 사고! LCD 출력 시도\x1b[0m\n");
                    LCD_display_v2x_mode(mode, g_sender_id, rx->accident.accident_id, display_dist, rx->accident.lane, rx->accident.type);

                    // --- 사고 리스트 업데이트 ---
                    int target_idx = -1;
                    for (int i = 0; i < MAX_ACCIDENTS; i++) {
                        if (accident_list[i].is_active && accident_list[i].data.accident.accident_id == rx->accident.accident_id) {
                            target_idx = i; break;
                        }
                    }
                    if (target_idx == -1) {
                        for (int i = 0; i < MAX_ACCIDENTS; i++) {
                            if (!accident_list[i].is_active) {
                                target_idx = i;
                                memset(&accident_list[i], 0, sizeof(managed_accident_t));
                                break;
                            }
                        }
                    }
                    if (target_idx != -1) {
                        accident_list[target_idx].is_active = true;
                        accident_list[target_idx].last_seen_ms = get_now_ms();
                        memcpy(&accident_list[target_idx].data.accident, &rx->accident, sizeof(wl1_accident_t));
                        accident_list[target_idx].data.analysis.dist_3d = dist_2d;
                        accident_list[target_idx].data.analysis.is_danger = (dist_2d < 100.0);
                    }
                } else {
                    printf("\x1b[1;33m[VAL-SKIP] 반대 방향 사고 무시 (차이: %d도)\x1b[0m\n", head_diff);
                }

                // --- 재전송(Relay) 로직 (방향 무관하게 수행) ---
                if (rx->header.ttl > 0) {
                    wl1_packet_t *relay = malloc(sizeof(wl1_packet_t));
                    if (relay) {
                        memcpy(relay, rx, sizeof(wl1_packet_t));
                        relay->header.ttl--;
                        relay->sender.sender_id = g_sender_id;
                        Q_push(&q_val_pkt_tx, relay);
                        printf("[RELAY-ACT] 사고 0x%lX 패킷 중계 큐 삽입\n", relay->accident.accident_id);
                    }
                }
            }
            // 수신 패킷 메모리 해제 (모든 로직 종료 후 한 번만)
            free(rx);
        }

        // [2] 주기적 보고 (0.5초마다)
        uint64_t now = get_now_ms();
        if (now - last_report_ms >= 500) {
            last_report_ms = now;
            int best_idx = -1;
            double min_dist = 999999.0;

            for (int i = 0; i < MAX_ACCIDENTS; i++) {
                if (!accident_list[i].is_active) continue;
                
                if (now - accident_list[i].last_seen_ms > (TIMEOUT_SEC * 1000)) {
                    accident_list[i].is_active = false;
                    continue;
                }
                
                if (accident_list[i].data.analysis.dist_3d < min_dist) {
                    min_dist = accident_list[i].data.analysis.dist_3d;
                    best_idx = i;
                }
            }

            if (best_idx != -1) {
                wl2_packet_t *wl2 = malloc(sizeof(wl2_packet_t));
                memset(wl2, 0, sizeof(wl2_packet_t));
                wl2->dist_rsv = (uint16_t)((uint16_t)min_dist << 4);
                wl2->lane = accident_list[best_idx].data.accident.lane;
                wl2->sev_rsv = (accident_list[best_idx].data.analysis.is_danger ? 0x10 : 0x00);
                
                Q_push(&q_val_yocto, wl2);
                DBG_INFO("VAL: Reporting Nearest -> ID: 0x%lX, Dist: %.1fm", 
                         accident_list[best_idx].data.accident.accident_id, min_dist);
            }
        }
        usleep(10000); 
    }
    return NULL;
}


