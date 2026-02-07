#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "spi.h"
#include "queue.h"
#include "debug.h"
#include "val_msg.h"
#include "val.h"
#include "driving_mgr.h"
/* 외부 참조 */
extern queue_t q_val_yocto;
extern driving_status_t g_driving_status;
extern volatile bool g_keep_running;

void *thread_spi(void *arg) {
    DBG_INFO("Thread 9: [SIMULATION] SPI-Yocto Module started.");

    while (g_keep_running) {
        // [기능 1] VAL 스레드로부터 분석 리포트 수신 확인
        wl2_data_t *wl2 = (wl2_data_t *)Q_pop_nowait(&q_val_yocto);
        
        if (wl2 != NULL) {
            // Yocto 화면에 표시된다고 가정하는 로그 출력
            printf("\n>>> [YOCTO DISPLAY] ALERT! Accident ID: 0x%X, Distance: %.1f m\n", 
                   wl2->accident.accident_id, wl2->analysis.dist_3d);
            
            // 사용한 메모리 해제
            free(wl2);
        }

        // [기능 2] Yocto(지자기 센서)로부터 데이터 수신 시뮬레이션
        // 실제 하드웨어가 없으므로 0~360도를 순전하게 도는 데이터를 생성
        static uint16_t sim_heading = 0;
        sim_heading = (sim_heading + 5) % 360; // 0.1초마다 5도씩 회전하는 가상 차량

        // 수신한 가상 Heading 값을 전역 변수에 저장 (VAL 스레드가 필터링에 사용함)
        pthread_mutex_lock(&g_driving_status.lock);
        g_driving_status.heading = (double)sim_heading;
        pthread_mutex_unlock(&g_driving_status.lock);

        // 주기적인 동작 확인 로그 (너무 잦으면 주석 처리)
        // DBG_DEBUG("SPI-SIM: Current Heading Updated to %d", sim_heading);

        usleep(100000); // 100ms 주기
    }

    DBG_INFO("Thread 9: SPI Simulation Module terminating.");
    return NULL;
}
