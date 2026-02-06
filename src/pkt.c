#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <stddef.h> // offsetof 사용을 위해 필요

#include "pkt.h"
#include "queue.h"
#include "debug.h"
#include "val.h"
#include "val_msg.h"
#include "proto_wl1.h"
#include "proto_wl3.h"
#include "driving_mgr.h"


#define PKT_STX 0xFD
#define PKT_ETX 0xFE

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

// [추가] 밀리초 단위 타임스탬프 반환 함수
static uint64_t get_current_timestamp_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void *sub_thread_pkt_tx(void *arg) {
    (void)arg;
    DBG_INFO("  - PKT-TX Sub-thread started (Dual-Path Aggregation).");

    while (g_keep_running) {
        wl1_packet_t *pkt = NULL; // 매 루프마다 초기화

        // --- [경로 1] 내 사고 정보 (Yocto 직통 큐: 26B 규격) 우선 확인 ---
        // pop_nowait를 써야 데이터가 없어도 바로 다음(릴레이 큐)으로 넘어간다.
        wl3_packet_t *wl3 = Q_pop_nowait(&q_yocto_if_to_pkt_tx);
        
        if (wl3 != NULL) {
            // 내 사고라면 새 WL-1 구조체 할당 및 조립 시작
            pkt = (wl1_packet_t *)malloc(sizeof(wl1_packet_t));
            if (pkt) {
                memset(pkt, 0, sizeof(wl1_packet_t));
                // Yocto(WL-3) 데이터를 무선 규격(WL-1)으로 맵핑


                /* --- [1] HEADER 조립 --- */
                pkt->header.version  = 0x01;
                //pkt->header.msg_type = 0x03; // WL-3 기반 사고 알림 타입
                pkt->header.msg_type = wl3->type; 
                pkt->header.ttl      = 3;    // 최초 발생 패킷 TTL 설정
                pkt->header.reserved = 0x00;

                /*
                //pkt->accident.type        = 0x03; 
                
                // 비트 필드 헬퍼 함수를 사용하여 방향 추출
                pkt->accident.direction = wl3_get_direction(wl3);
                pkt->accident.lane        = wl3->lane;
                // [추가] Severity 설정
                // WL-3의 severity(1바이트)를 WL-1의 해당 필드에 대입
                // 만약 WL-1 구조체에 severity 필드가 있다면 아래와 같이 직접 대입합니다.
                pkt->accident.severity      = wl3->severity;

                /* --- [수정 포인트] ACCIDENT 조립 (26B 규격 데이터 맵핑) --- */
                //pkt->accident.accident_id = wl3->accident_id;
                //pkt->accident.accident_time = wl3->accident_time; // [추가] Decision이 기록한 64비트 시간
            
                /* --- [2] 핵심 페이로드 구간(20B) 메모리 복사 --- */
                // Direction(2) + Lane(1) + Severity(1) + Time(8) + ID(8) = 총 20바이트
                // WL-3의 reserved(비트필드 포함 2B) 위치부터 WL-1의 accident.direction으로 복사합니다.
                //memcpy(&(pkt->accident.direction), &(wl3->reserved), 20);
                // wl3 구조체 내에서 reserved 비트필드가 속한 바이트의 오프셋을 계산
                //size_t offset = offsetof(wl3_packet_t, lane) - 2; // lane(Byte 7) 앞의 2바이트 지점
                //memcpy(&(pkt->accident.direction), (uint8_t *)wl3 + offset, 20);


                memcpy(&(pkt->accident.direction), (uint8_t *)wl3 + 5, 20);
                /* --- [3] 앞뒤 부족한 GPS 정보는 직접 참조(Reference)로 주입 --- */
                // 위도/경도/고도는 WL-3에 없으므로 g_driving_status에서 직접 가져옵니다.
                pkt->accident.lat_uDeg  = (int32_t)(g_driving_status.lat * 1000000.0);
                pkt->accident.lon_uDeg  = (int32_t)(g_driving_status.lon * 1000000.0);
                pkt->accident.alt_mm    = (int32_t)(g_driving_status.alt * 1000.0);

                // 사고 정보의 방향(Direction)을 WL-4에서 받은 최신 주행 방향으로 설정
                // 내 차량이 사고가 난 것이므로, 현재 내 주행 방향이 사고 방향이다.
                //pthread_mutex_lock(&g_driving_status.lock);
                //pkt->accident.direction = (uint16_t)g_driving_status.heading;
                //pthread_mutex_unlock(&g_driving_status.lock);
                
                // 로그에서 %lX를 써야 64비트가 다 보임.
                //DBG_INFO("\x1b[32m[PKT-TX] 내 사고 조립 완료 (ID: 0x%lX, Dir: %d, Lane: %d)\x1b[0m", 
                    //wl3->accident_id, pkt->accident.direction, pkt->accident.lane);
                /* --- 로그 출력 (ID, Lane, Sev, Time, Dir) --- */
                /*DBG_INFO("\x1b[32m[PKT-TX] 내 사고 조립 완료 (ID: 0x%lX, Lane: %d, Sev: %d, Time: %lu, Dir: %d)\x1b[0m", 
                    pkt->accident.accident_id, 
                    pkt->accident.lane, 
                    pkt->accident.severity, 
                    pkt->accident.accident_time,
                    pkt->accident.direction); // 방향 정보 추가
                    */
                /* --- [4] 로그 출력 (ID, Lane, Sev, Time, Dir) --- */
                // memcpy로 데이터가 잘 들어왔는지 확인합니다.
                DBG_INFO("\x1b[32m[PKT-TX] 내 사고 조립 완료 (ACC_ID: 0x%lX, Lane: %d, Sev: %d, Time: %lu, Dir: %d)\x1b[0m", 
                    pkt->accident.accident_id, 
                    pkt->accident.lane, 
                    pkt->accident.severity, 
                    pkt->accident.accident_time,
                    pkt->accident.direction);

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





