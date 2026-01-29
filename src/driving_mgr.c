#include "driving_mgr.h"
#include "gps.h"
#include "queue.h"
#include "proto_wl4.h" 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
extern volatile bool g_keep_running;
extern queue_t q_yocto_to_driving;
extern driving_status_t g_driving_status;


void *thread_driving_manager(void *arg) {
    (void)arg;
    while (g_keep_running) {
        // T9(yocto_if)에서 전달되는 WL-4 대기
        wl4_packet_t *wl4 = (wl4_packet_t *)Q_pop(&q_yocto_to_driving);
        if (!wl4) continue;

        // 최신 GPS 데이터 가져오기
        gps_data_t gps = GPS_get_latest();

        pthread_mutex_lock(&g_driving_status.lock);
        g_driving_status.lat = gps.lat;
        g_driving_status.lon = gps.lon;
        g_driving_status.alt = gps.alt;
        
        // [수정] wl4->heading 대신 헬퍼 함수 wl4_get_direction(*wl4) 사용
        g_driving_status.heading = *((uint8_t*)wl4+1) + (*((uint8_t*)wl4) & 0x1) * 256;
        pthread_mutex_unlock(&g_driving_status.lock);

        // [수정] 출력부에서도 헬퍼 함수를 사용하여 비트 데이터를 파싱.
        printf("\x1b[32m[T5-DRIVING] LAT:%.6f, HDG:%u\x1b[0m\n", 
               gps.lat, g_driving_status.heading);
            //    printf("%b DEBUG HERE\r\n", *((uint8_t*)wl4+1));
        
        free(wl4);
    }
    return NULL;
}
