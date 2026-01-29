#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "debug.h"
#include "queue.h"
#include "wl.h"
#include "pkt.h"
#include "sec.h"
#include "gps.h"
#include "val.h"
#include "yocto_if.h" // T9 인터페이스 모듈
#include "driving_mgr.h"
#include "i2c_io.h"


// 외부 파일(pkt.c, sec.c, wl.c 등)에서 정의된 스레드 함수들 선언
extern void *thread_rx(void *arg);       // T1
extern void *thread_sec_rx(void *arg);   // T2

extern void *thread_val(void *arg);      // T4
extern void *sub_thread_pkt_tx(void *arg);   // T6: 이 선언이 없어서 에러가 났습니다.
extern void *sub_thread_pkt_rx(void *arg);   // T6: 이 선언이 없어서 에러가 났습니다.
extern void *thread_sec_tx(void *arg);   // T7
extern void *thread_yocto_if(void *arg);
extern void *thread_driving_manager(void *arg);
extern void *thread_tx(void *arg);
extern void *thread_gps(void *arg); 
// ==========================================
// 1. 전역 상태 및 큐 정의 (vT11 파이프라인)
// ==========================================
volatile bool g_keep_running = true;
uint32_t g_sender_id;
extern double g_accident_point; 
//wl_ctx_t g_wl_tx_ctx;


// 차량 및 주행 상태 (T5 주행관리 스레드용)
driving_status_t g_driving_status = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .lat = 0.0, .lon = 0.0, .alt = 0, .heading = 0
};

/*// 파이프라인 큐 (vT11 흐름)
queue_t q_rx_sec, q_sec_pkt, q_pkt_val;      // RX 라인
queue_t q_val_pkt, q_pkt_sec, q_sec_tx;      // TX 라인
queue_t q_val_yocto;                         // T4 -> T9 (WL-2 송신)
queue_t q_yocto_to_driving;                  // T9 -> T5 (WL-4 전달)
*/
// [해결 2] 큐 이름 통일: 각 .c 파일들이 extern으로 기대하는 이름들
queue_t q_rx_sec_rx;    // T1 -> T2
queue_t q_sec_rx_pkt;   // T2 -> T3
queue_t q_val_pkt_tx;   // T4 -> T6
queue_t q_pkt_val;
queue_t q_pkt_sec_tx;   // T6 -> T7
queue_t q_sec_tx_wl_tx; // T7 -> T8
queue_t q_val_yocto;      // T4 -> T9
queue_t q_yocto_to_driving; // T9 -> T5
queue_t q_yocto_pkt_tx;
queue_t q_wl_sec;
queue_t q_yocto_if_to_pkt_tx; // [추가] T9 -> T6 (내 사고 직통용)
// ==========================================
// 2. 종료 및 초기화 로직
// ==========================================
void signal_handler(int sig) {
    (void)sig;
    g_keep_running = false;
    
    // 블로킹 상태의 큐들을 깨우기 위해 NULL 푸시
    Q_push(&q_rx_sec_rx, NULL); Q_push(&q_sec_rx_pkt, NULL);
    Q_push(&q_pkt_val, NULL); Q_push(&q_val_pkt_tx, NULL);
    Q_push(&q_pkt_sec_tx, NULL); Q_push(&q_sec_tx_wl_tx, NULL);
    Q_push(&q_val_yocto, NULL); Q_push(&q_yocto_to_driving, NULL);
    Q_push(&q_yocto_if_to_pkt_tx, NULL); // [추가] 신규 큐 종료 처리
    printf("\n[MAIN] Shutdown signal received. Cleaning up...\n");
}

/*void *test_loop_thread(void *arg) {
    printf("\n[TEST] WL-4 주기적 투입 시작 (30초 필터 통과 대기 중...)\n");
    
    while (g_keep_running) {
        wl4_packet_t *test_wl4 = malloc(sizeof(wl4_packet_t));
        if (test_wl4) {
            memset(test_wl4, 0, sizeof(wl4_packet_t));
            
            // 리틀 엔디안 기준 하위 8비트에 0x04 주입 (yocto_if 인식용)
            uint8_t *raw_ptr = (uint8_t *)&(test_wl4->raw);
            raw_ptr[0] = 0x04; 

            // 방향을 270도로 설정
            wl4_set_direction(test_wl4, 270); 

            Q_push(&q_val_yocto, test_wl4); 
        }
        // 1초마다 한 번씩 계속 쏩니다. 
        // 이렇게 하면 yocto_if의 30초 주기 필터에 결국 한 번은 걸림
        sleep(1); 
    }
    return NULL;
}
*/
void *final_test_thread(void *arg) {
    sleep(3);
    if (g_sender_id != 0x1111) return NULL;

    int mock_heading = 0;
    double accident_point = 37.5690;
    // 15초 주행을 위해 시작 지점을 사고 지점 165m 전(0.0015도 전)으로 설정
    double start_lat = 37.5675; 
    bool arrived = false;

    // 초기 위치 설정
    pthread_mutex_lock(&g_driving_status.lock);
    g_driving_status.lat = start_lat;
    pthread_mutex_unlock(&g_driving_status.lock);

    //printf("[TEST-차1] 15초 주행 시뮬레이션 시작 (목표: 37.5690)\n");

    while (g_keep_running) {
        pthread_mutex_lock(&g_driving_status.lock);
        // 1. 매초 위도를 0.0001도(약 11m)씩 증가시켜 주행 시뮬레이션
        if (!arrived) {
            g_driving_status.lat += 0.0001;
        }
        double now_lat = g_driving_status.lat;
        pthread_mutex_unlock(&g_driving_status.lock);

        // 2. 사고 지점에 도착했는지 체크
        if (!arrived && now_lat >= accident_point) {
            arrived = true;
            //printf("\n[TEST-차1] ★사고 발생!★ (지점: %f)\n", now_lat);
        }

        // 3. 주행 데이터(WL-4) 주입 - 매초 쏴서 디스플레이 업데이트 유지
        wl4_packet_t *test_wl4 = malloc(sizeof(wl4_packet_t));
        if (test_wl4) {
            memset(test_wl4, 0, sizeof(wl4_packet_t));
            ((uint8_t *)test_wl4)[0] = 0x04;
            wl4_set_direction(test_wl4, mock_heading);
            Q_push(&q_val_yocto, test_wl4);
        }

        // 4. 사고 지점 도달 이후에는 사고 데이터(WL-3) 주입
        if (arrived) {
            wl3_packet_t *test_wl3 = malloc(sizeof(wl3_packet_t));
            if (test_wl3) {
                memset(test_wl3, 0, sizeof(wl3_packet_t));
                ((uint8_t *)test_wl3)[0] = 0x03;
                test_wl3->accident_type = 3;  // wl-3 패킷
                test_wl3->lane = 2;
                // accident_id가 있다면 고유값 부여
                test_wl3->accident_id = 0x1234567812345678ULL; 
                //Q_push(&q_val_yocto, test_wl3);
                Q_push(&q_yocto_if_to_pkt_tx, test_wl3);
                
                
                //printf("[TEST-차1] 사고 패킷 송신 중 (%f)\n", now_lat);
            }
        }

        // 주기를 1초로 줄여야 주행과 사고 전환이 매끄럽게 보입니다.
        sleep(1); 
    }
    return NULL;
}

// ==========================================
// 4. 메인 진입점
// ==========================================
int main(int argc, char *argv[]) {
    // 1. 초기화 및 시그널 설정
    signal(SIGINT, signal_handler);
    g_sender_id = (argc > 1) ? (uint32_t)strtol(argv[1], NULL, 16) : 0x1111;
    

   // 큐 초기화
    Q_init(&q_rx_sec_rx);
    Q_init(&q_sec_rx_pkt);
    Q_init(&q_val_pkt_tx);
    Q_init(&q_pkt_sec_tx);
    Q_init(&q_sec_tx_wl_tx);
    Q_init(&q_val_yocto);
    Q_init(&q_yocto_to_driving);
    Q_init(&q_yocto_if_to_pkt_tx); 

    
    // 3. 하드웨어 초기화
    GPS_init();
    

    printf("--- Integrated V2X System Start ---\n");

    
    // 2. 스레드 생성
    pthread_t ths[12];
    
    // 1. RX 파이프라인 (T1 ~ T4)
    pthread_create(&ths[0], NULL, thread_rx, NULL);           // T1: Wireless RX
    pthread_create(&ths[1], NULL, thread_sec_rx, NULL);       // T2: Security RX
    pthread_create(&ths[2], NULL, sub_thread_pkt_rx, NULL);       // T3: Packet RX
    pthread_create(&ths[3], NULL, thread_val, NULL);          // T4: Valuation (판단)

    // 2. 주행 데이터 통합 관리 (T5) - GPS 확장판
    pthread_create(&ths[4], NULL, thread_driving_manager, NULL); // T5: Driving Manager

    // 3. TX 파이프라인 (T6 ~ T8)
    pthread_create(&ths[5], NULL, sub_thread_pkt_tx, NULL);       // T6: Packet TX
    pthread_create(&ths[6], NULL, thread_sec_tx, NULL);       // T7: Security TX
    
    pthread_create(&ths[7], NULL, thread_tx, NULL);   // T8: Wireless TX

    // 4. 외부 센서 인터페이스 (T9)
    pthread_create(&ths[8], NULL, thread_yocto_if, NULL);     // T9: Yocto I2C
    pthread_create(&ths[9], NULL, thread_gps, NULL);
    pthread_create(&ths[10], NULL, thread_gps, NULL);          // T10

    // main.c 스레드 생성 섹션에 추가
    pthread_t final_test_tid;
    if (g_sender_id == 0x1111) {
        pthread_create(&final_test_tid, NULL, final_test_thread, NULL);
        //printf("[MAIN] 차1: Final Test Thread(사고/방향회전)가 시작되었습니다.\n");
    }
    

    // 3. 메인 모니터링 루프
    
    
    while (g_keep_running) {
        
        sleep(1);
    }

    // 4. 종료 및 자원 해제
    for (int i = 0; i < 11; i++) {
        pthread_join(ths[i], NULL);
    }

    pthread_mutex_destroy(&g_driving_status.lock);
    printf("[MAIN] V2X System Gracefully Terminated.\n");

    return 0;
}
    
