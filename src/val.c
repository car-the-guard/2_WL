#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "val.h"
#include "queue.h"
#include "debug.h"
#include "val_msg.h"
#include "driving_mgr.h"
#include "proto_wl2.h"
//#include "i2c_io.h"

#define MAX_ACCIDENTS 20
#define DIST_LIMIT    1000.0   // 1km 유효거리
#define ALT_LIMIT     25.0      // 5m 고도차 필터, 현재 기본 고도 15m (교량/지하도로 구분용)
#define TIMEOUT_SEC   5        // // 5초간 수신 없으면 리스트에서 삭제
#define HEADING_LIMIT 45       // 45도 이내 차이만 동일 방향으로 간주

#define MODE_TX    0  // 내 사고 송신
#define MODE_RELAY 1  // 타인 사고 재전송
#define MODE_ALERT 2  // 위험 경고 (LN, TYPE, DIST 표시)

#define HOST_NOTIFY_PORT 38474
#define TRIGGER_BINARY_SIZE 6  /* 거리32 + 방향8 + 위험8 (trigger_send_binary 호환) */

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

/** 127.0.0.1:HOST_NOTIFY_PORT 로 바이너리 6바이트 전송 (trigger_send_binary 형식) */
static void send_binary_to_host(const void *buf, size_t len) {
    if (!buf || len == 0) return;
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HOST_NOTIFY_PORT);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(s); return; }
    send(s, buf, len, 0);
    close(s);
}

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
            DBG_INFO("[DEBUG-VAL] >>> VAL Controller Got Data! From: 0x%X\n", rx->sender.sender_id);
            
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
                    //LCD_display_v2x_mode(mode, g_sender_id, rx->accident.accident_id, display_dist, rx->accident.lane, rx->accident.type);

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
                        DBG_INFO("[VAL-DBG] accident_list[%d] 등록 -> ID:0x%lX (WL-2 후보)\n", target_idx, rx->accident.accident_id);
                    }
                } else {
                    DBG_INFO("\x1b[1;33m[VAL-SKIP] 반대 방향 사고 무시 (차이: %d도)\x1b[0m\n", head_diff);
                }

                // --- 재전송(Relay) 로직 (방향 무관하게 수행) ---
                if (rx->header.ttl > 0) {
                    wl1_packet_t *relay = malloc(sizeof(wl1_packet_t));
                    if (relay) {
                        memcpy(relay, rx, sizeof(wl1_packet_t));
                        relay->header.ttl--;
                        relay->sender.sender_id = g_sender_id;
                        Q_push(&q_val_pkt_tx, relay);
                        DBG_INFO("[RELAY-ACT] 사고 0x%lX 패킷 중계 큐 삽입\n", relay->accident.accident_id);
                    }
                }
            }else {
                /* WL-2 로그 원인 추적: 여기서 걸리면 accident_list에 안 쌓임 */
                if (rx->sender.sender_id == g_sender_id)
                    printf("[VAL-DBG] 패킷 제외: sender_id == 내 ID (0x%X)\n", (unsigned)g_sender_id);
                else if (dist_2d >= DIST_LIMIT)
                    printf("[VAL-DBG] 패킷 제외: 거리 %.0fm >= %.0fm\n", dist_2d, DIST_LIMIT);
                else if (alt_diff >= ALT_LIMIT)
                    printf("[VAL-DBG] 패킷 제외: 고도차 %.1fm >= %.1fm\n", alt_diff, ALT_LIMIT);
            
            }        // 수신 패킷 메모리 해제 (모든 로직 종료 후 한 번만)
            free(rx);
        }

        // [2] 주기적 보고 (0.5초마다)
        uint64_t now = get_now_ms();
        if (now - last_report_ms >= 500) {
            last_report_ms = now;


            // [2차 추가] 1. 내 현재 위치 스냅샷
            pthread_mutex_lock(&g_driving_status.lock);
            double cur_lat = g_driving_status.lat;
            double cur_lon = g_driving_status.lon;
            pthread_mutex_unlock(&g_driving_status.lock);

            int best_idx = -1;
            double min_dist = 999999.0;

            for (int i = 0; i < MAX_ACCIDENTS; i++) {
                if (!accident_list[i].is_active) continue;
                
                // 2. 타임아웃 체크 (5초 경과 시 삭제)
                if (now - accident_list[i].last_seen_ms > (TIMEOUT_SEC * 1000)) {
                    accident_list[i].is_active = false;
                    continue;
                }

                double target_lat = (double)accident_list[i].data.accident.lat_uDeg / 1000000.0;
                double target_lon = (double)accident_list[i].data.accident.lon_uDeg / 1000000.0;

                // 내 최신 위치(cur_lat, cur_lon) 기준으로 거리 업데이트
                double updated_dist = calc_dist(cur_lat, cur_lon, target_lat, target_lon);
                
                // 리스트 데이터 갱신 (트래킹 반영)
                accident_list[i].data.analysis.dist_3d = updated_dist;
                accident_list[i].data.analysis.is_danger = (updated_dist < 100.0);
                
                        // 갱신된 거리로 최단 거리 사고 선별
                if (updated_dist < min_dist) {
                    min_dist = updated_dist;
                    best_idx = i;
                    }
                } // for 루프 끝
                //if (accident_list[i].data.analysis.dist_3d < min_dist) {
                    //min_dist = accident_list[i].data.analysis.dist_3d;
                    //best_idx = i;
                //}
            //}
            
            /*// --- [수정 포인트] 실시간 거리 재계산 로직 추가 ---
        // 리스트에 저장된 사고의 고정 좌표 추출
        double target_lat = (double)accident_list[i].data.accident.lat_uDeg / 1000000.0;
        double target_lon = (double)accident_list[i].data.accident.lon_uDeg / 1000000.0;

        // 내 최신 위치(cur_lat, cur_lon) 기준으로 거리 업데이트
        double updated_dist = calc_dist(cur_lat, cur_lon, target_lat, target_lon);
        
        // 리스트 데이터 갱신 (트래킹 반영)
        accident_list[i].data.analysis.dist_3d = updated_dist;
        accident_list[i].data.analysis.is_danger = (updated_dist < 100.0);
        // ----------------------------------------------
        */
        // 갱신된 거리로 최단 거리 사고 선별
        /*if (updated_dist < min_dist) {
            min_dist = updated_dist;
            best_idx = i;
            }
        }*/

            //}

            /* WL-2 로그 원인 추적: 500ms마다 활성 사고 수 / best_idx 출력 (best_idx>=0 일 때만 WL-2 로그 나옴) */
            {
                int active_cnt = 0;
                for (int i = 0; i < MAX_ACCIDENTS; i++)
                    if (accident_list[i].is_active && now - accident_list[i].last_seen_ms <= (TIMEOUT_SEC * 1000))
                        active_cnt++;
                DBG_INFO("[VAL-DBG] 500ms 보고: active=%d best_idx=%d -> WL-2 %s\n",
                       active_cnt, best_idx, (best_idx != -1) ? "생성" : "미생성");
            }

            if (best_idx != -1) {
                wl2_packet_t *wl2 = malloc(sizeof(wl2_packet_t));
                memset(wl2, 0, sizeof(wl2_packet_t));
                
                // --- Header 설정 ---
                wl2->stx = 0xFD;
                wl2->type = 0x02; // WL-2
                wl2->timestamp = (uint16_t)(get_now_ms() & 0xFFFF);
                //wl2->dist_rsv = (uint16_t)((uint16_t)min_dist << 4);
                // --- Payload 설정 (16비트 직접 대입) ---
                wl2->distance = (uint16_t)min_dist;
                wl2->lane = accident_list[best_idx].data.accident.lane;
                //wl2->sev_rsv = (accident_list[best_idx].data.analysis.is_danger ? 0x10 : 0x00);
                wl2->severity = (accident_list[best_idx].data.analysis.is_danger ? 0x01 : 0x00);
        
                // --- Trailer 설정 ---
                wl2->etx = 0xFE;
                
                
                //wl2->dist_rsv = (uint16_t)((uint16_t)min_dist << 4);
                //wl2->lane = accident_list[best_idx].data.accident.lane;
                //wl2->sev_rsv = (accident_list[best_idx].data.analysis.is_danger ? 0x10 : 0x00);
                
                Q_push(&q_val_yocto, wl2);
                printf("[VAL] WL-2 보고 -> ID: 0x%lX, Dist: %.1fm\n",
                       (unsigned long)accident_list[best_idx].data.accident.accident_id, min_dist);
                DBG_INFO("VAL: Reporting Nearest -> ID: 0x%lX, Dist: %.1fm",
                         accident_list[best_idx].data.accident.accident_id, min_dist);
                /* 38474 포트로 바이너리 전송 (방향 L/F/R, 거리, 위험도 1~3) */
                {
                    uint8_t lane = accident_list[best_idx].data.accident.lane;
                    uint8_t dir = (lane == 1) ? 'L' : (lane == 2) ? 'F' : (lane == 3) ? 'R' : (uint8_t)'F';
                    uint8_t danger = accident_list[best_idx].data.analysis.is_danger ? 3 : 1;
                    uint32_t dist_be = htonl((uint32_t)(uint16_t)min_dist);
                    unsigned char bin[TRIGGER_BINARY_SIZE];
                    memcpy(bin, &dist_be, 4);
                    bin[4] = dir;
                    bin[5] = danger;
                    send_binary_to_host(bin, TRIGGER_BINARY_SIZE);
                }
            }
        }
        usleep(10000); 
    }
    return NULL;
}


