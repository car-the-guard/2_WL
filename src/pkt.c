#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include "pkt.h"
#include "queue.h"
#include "debug.h"
#include "val.h"
#include "val_msg.h"
#include "proto_wl1.h"
#include "proto_wl3.h"
#include "driving_mgr.h"


extern queue_t q_pkt_val;       // RX 패킷 -> VAL 모듈 전달용
extern queue_t q_sec_rx_pkt;    // 보안 검증 완료된 수신 패킷 큐
extern queue_t q_val_pkt_tx;    // VAL 모듈로부터 온 재전파 요청 큐(외부사고 릴레이용)
extern queue_t q_yocto_if_to_pkt_tx;   // 내 사고(Yocto) 직통용
extern queue_t q_pkt_sec_tx;    // 조립 완료된 패킷 -> 보안 서명 큐
extern volatile bool g_keep_running;
extern uint32_t g_sender_id;
extern driving_status_t g_driving_status;

// 현재 타임스탬프를 마이크로초 단위로 가져오는 헬퍼 함수
static uint64_t get_current_timestamp_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000000 + (uint64_t)(tv.tv_usec);
}
void *sub_thread_pkt_tx(void *arg) {
    (void)arg;
    DBG_INFO("  - PKT-TX Sub-thread started (Dual-Path Aggregation).");

    while (g_keep_running) {
        wl1_packet_t *pkt = NULL; // 매 루프마다 초기화

        // --- [경로 1] 내 사고 정보 (Yocto 직통 큐) 우선 확인 ---
        // pop_nowait를 써야 데이터가 없어도 바로 다음(릴레이 큐)으로 넘어간다.
        wl3_packet_t *wl3 = Q_pop_nowait(&q_yocto_if_to_pkt_tx);
        
        if (wl3 != NULL) {
            // 내 사고라면 새 WL-1 구조체 할당 및 조립 시작
            pkt = (wl1_packet_t *)malloc(sizeof(wl1_packet_t));
            if (pkt) {
                memset(pkt, 0, sizeof(wl1_packet_t));
                // Yocto(WL-3) 데이터를 무선 규격(WL-1)으로 맵핑
                /* --- HEADER 조립 --- */
                pkt->header.version  = 0x01;
                pkt->header.msg_type = 0x03; // WL-3 기반 사고 알림 타입
                pkt->header.ttl      = 3;    // 최초 발생 패킷 TTL 설정
                pkt->header.reserved = 0x00;

                /* --- ACCIDENT 조립 (Yocto 데이터 활용) --- */
                pkt->accident.accident_id = wl3->accident_id;
                pkt->accident.type        = wl3->accident_type; 
                pkt->accident.lane        = wl3->lane;
                
                // 위도/경도/고도 주입
                pkt->accident.lat_uDeg  = (int32_t)(g_driving_status.lat * 1000000.0);
                pkt->accident.lon_uDeg  = (int32_t)(g_driving_status.lon * 1000000.0);
                pkt->accident.alt_mm    = (int32_t)(g_driving_status.alt * 1000.0);

                // 사고 정보의 방향(Direction)을 WL-4에서 받은 최신 주행 방향으로 설정
                // 내 차량이 사고가 난 것이므로, 현재 내 주행 방향이 사고 방향이다.
                pthread_mutex_lock(&g_driving_status.lock);
                pkt->accident.direction = (uint16_t)g_driving_status.heading;
                pthread_mutex_unlock(&g_driving_status.lock);
                
                // 로그에서 %lX를 써야 64비트가 다 보임.
                DBG_INFO("\x1b[32m[PKT-TX] 내 사고 조립 완료 (ID: 0x%lX, Dir: %d, Lane: %d)\x1b[0m", 
                    wl3->accident_id, pkt->accident.direction, pkt->accident.lane);
                
            }
                free(wl3); // 사용이 끝난 WL-3 메모리 해제
        }
// [2] 내 사고가 없다면 외부 릴레이(Relay) 큐 확인
        else {
            pkt = Q_pop_nowait(&q_val_pkt_tx);
            if (pkt && pkt->header.ttl > 0) {
                pkt->header.ttl -= 1; // 릴레이 시 TTL 차감
            }
        }
        // [3] SENDER 조립 (내 GPS 및 상태 정보 결합)
        if (pkt != NULL) {
            if (pkt->header.ttl > 0) {
                pthread_mutex_lock(&g_driving_status.lock);
                
                /* --- SENDER 조립 --- */
                pkt->sender.sender_id = g_sender_id; 
                
                // 위도/경도/고도 주입
                pkt->sender.lat_uDeg  = (int32_t)(g_driving_status.lat * 1000000.0);
                pkt->sender.lon_uDeg  = (int32_t)(g_driving_status.lon * 1000000.0);
                pkt->sender.alt_mm    = (int32_t)(g_driving_status.alt * 1000.0);
                                

                // 타임스탬프
                pkt->sender.send_time = get_current_timestamp_us();
                
                pthread_mutex_unlock(&g_driving_status.lock);

                // [4] 최종 보안 서명 큐로 전달
                Q_push(&q_pkt_sec_tx, pkt);
                DBG_INFO("[PKT-TX] WL-1 전체 패킷 조립 완료 (SenderID: 0x%X, AccID: 0x%lX)", 
          pkt->sender.sender_id, pkt->accident.accident_id);
            } else {
                free(pkt);
            }
        }
        usleep(10000); 
    }
    return NULL;
}



// --- [PKT-RX] 수신 전담 서브 스레드 ---
void *sub_thread_pkt_rx(void *arg) {
    (void)arg;
    DBG_INFO("  - PKT-RX Sub-thread started.");

    while (g_keep_running) {
        // 보안 검증(T2)이 완료된 데이터 대기
        wl1_packet_t *rx_pkt = Q_pop(&q_sec_rx_pkt);
        
        if (rx_pkt) {
            // 자기 패킷 필터링 로직
            // g_sender_id는 메인에서 선언된 내 기기의 ID 
            if (rx_pkt->sender.sender_id == g_sender_id) {
                DBG_INFO("PKT-RX: Loopback packet detected (ID: 0x%X). Self-dropping.", 
                          rx_pkt->sender.sender_id);
                free(rx_pkt); // VAL로 넘기지 않고 여기서 메모리 해제
                continue;     // 다음 패킷 대기
            }
            DBG_INFO("PKT-RX: Incoming Packet (Sender: 0x%lX, Accident: 0x%lX)", 
                     rx_pkt->sender.sender_id, rx_pkt->accident.accident_id);
            // 2. [추가] VAL 스레드(T4)로 데이터 배달
            wl1_packet_t *to_val = malloc(sizeof(wl1_packet_t));
            if (to_val) {
                memcpy(to_val, rx_pkt, sizeof(wl1_packet_t));
                Q_push(&q_pkt_val, to_val); // VAL 큐로 전달
                printf("[DEBUG-PKT] Packet pushed to q_pkt_val\n"); 
            }
            // [4] 판단 모듈(VAL, T4)로 전달하여 1km 필터링 및 사고 관리 수행
            
        
        
        }
    }
    return NULL;
}

// --- 메인 PKT 관리 스레드 ---
void *thread_pkt(void *arg) {
    (void)arg;
    pthread_t tx_tid, rx_tid;

    DBG_INFO("Thread 2: Packet Management Module (TX/RX Sub-threads) initializing...");

    // 내부적으로 송신/수신 서브 스레드 생성
    pthread_create(&tx_tid, NULL, sub_thread_pkt_tx, NULL);
    pthread_create(&rx_tid, NULL, sub_thread_pkt_rx, NULL);

    // 스레드 생존 주기를 메인과 동기화
    pthread_join(tx_tid, NULL);
    pthread_join(rx_tid, NULL);

    DBG_INFO("Thread 2: Packet Management Module terminating.");
    return NULL;
}





