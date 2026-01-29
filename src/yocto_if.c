#include "yocto_if.h"
#include "i2c_io.h"
#include "queue.h"
//#include "val_msg.h"
//#include "proto_wl1.h"
#include "proto_wl2.h"
#include "proto_wl3.h"
#include "proto_wl4.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "debug.h"
#include "driving_mgr.h"
#include "gps.h"

// I2C 설정 (환경에 맞게 수정 필요)
#define I2C_BUS_YOCTO  "/dev/i2c-1" 
#define YOCTO_ADDR      0x27        

extern queue_t q_wl_sec;     // 수신 파이프라인 시작점
extern queue_t q_yocto_pkt_tx; // 송신 파이프라인 시작점

extern driving_status_t g_driving_status;

extern queue_t q_val_pkt_tx, q_val_yocto, q_rx_sec_rx;
extern volatile bool g_keep_running;
extern uint32_t g_sender_id;
extern queue_t q_pkt_val;
extern queue_t q_val_yocto; // VAL -> Yocto (WL-2 송신: 가장 가까운 사고 정보)
extern queue_t q_yocto_to_driving; // Yocto_IF -> Driving (WL-4 수신: 주행정보)
extern queue_t q_pkt_sec_tx;
extern queue_t q_yocto_if_to_pkt_tx;   // Yocto_IF -> PKT (WL-3 수신: 내 사고 직통 큐)

// 시간 측정을 위한 유틸리티 함수
static uint64_t get_now_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}
// [2] 정밀 주기 제어 함수 
static void wait_next_period(struct timespec *next, long ms) {
    next->tv_nsec += ms * 1000000L;
    while (next->tv_nsec >= 1000000000L) {
        next->tv_sec++;
        next->tv_nsec -= 1000000000L;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
}

// UART3 초기화 함수: 포트 설정(9600bps, 8N1, Raw Mode)
int UART3_init(void) {
    int fd = open(UART3_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;

    struct termios opt;
    tcgetattr(fd, &opt);
    cfsetispeed(&opt, B9600);
    cfsetospeed(&opt, B9600);

    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_cflag &= ~PARENB;   // 패리티 없음
    opt.c_cflag &= ~CSTOPB;   // 정지 비트 1
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;       // 데이터 8비트

    // 중요: 바이너리 패킷 보존을 위한 Raw Mode 설정
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &opt);
    return fd;
}

void* thread_yocto_if(void* arg) {
    (void)arg;
    
    // 1. UART3 포트 오픈
    int uart_fd = UART3_init();
    if (uart_fd < 0) {
        DBG_ERR("[T9] UART3 (/dev/ttyAMA3) Open Failed!");
        return NULL;
    }

    // 2. 정밀 시간 설정을 위한 구조체
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    uint32_t loop_cnt = 0;
    // 50ms 루프 기준으로 30초는 600번 루프 (30,000ms / 50ms = 600)
    const uint32_t wl4_trigger = (PERIOD_WL4_S * 1000) / PERIOD_YOCTO_MS;

    DBG_INFO("Thread 9: Yocto Interface Module started (UART3 Mode).");
    DBG_INFO("[T9] UART3 모듈 시작 (WL2/3: 50ms, WL4: 30s 주기)");

    while (g_keep_running) {
        // 50ms 정밀 주기 제어 (WL-2, WL-3을 위해 빠르게 회전)
        wait_next_period(&next_time, PERIOD_YOCTO_MS);

        // --- [A] 송신: VAL에서 온 가장 가까운 사고 정보(WL-2) 전송 ---
        wl2_packet_t *wl2 = (wl2_packet_t *)Q_pop_nowait(&q_val_yocto);
        if (wl2) {
            int tx_res = write(uart_fd, wl2, sizeof(wl2_packet_t));

            if (tx_res > 0) {
                //DBG_INFO("[T9-TX] WL-2(가까운 사고)를 Yocto로 송신 완료");
            }
            free(wl2);
        }
        // --- [B] 수신: Yocto로부터 데이터 읽기 (WL-3, WL-4) ---
        uint8_t rx_buf[256] = {0};
        int rx_len = read(uart_fd, rx_buf, sizeof(rx_buf));

        if (rx_len > 0) {
        uint8_t type = rx_buf[0];
        printf("yocto로부터 uart3으로 메세지 받음 %lX, %d, %d \r\n", rx_buf, rx_len, type);
        //printf("DEBUG-- %x %x\r\n", rx_buf[0], rx_buf[1]);
            // 1. 내 사고 정보 (WL-3) -> 패킷 모듈로 직통 전송
            if (type == TYPE_WL3) {
                wl3_packet_t *wl3 = malloc(sizeof(wl3_packet_t));
                if (wl3) {
                    memcpy(wl3, rx_buf, sizeof(wl3_packet_t));
                    wl3->accident_id = *((unsigned long long*)(rx_buf + 16));
                    wl3->lane = *(rx_buf + 5);
                    
                    Q_push(&q_yocto_if_to_pkt_tx, wl3); // 신규 직통 큐 사용
                    printf("\x1b[32m[T9-RX] WL-3 수신: 내 사고 발생 -> PKT-TX로 직통 전달\x1b[0m\n");
                }
            }
            // 2. 주행 정보 (WL-4) -> 주행 관리 모듈로 전송
            else if (type == TYPE_WL4 || type == 5) {
                // 테스트를 위해 주기 조건을 주석 처리.
                //printf("DEBUG HERE %d\r\n", wl4_trigger);

                // if (loop_cnt % wl4_trigger == 0) {
                    wl4_packet_t *wl4 = malloc(sizeof(wl4_packet_t));
                    if (wl4) {
                        memcpy(wl4, rx_buf, sizeof(wl4_packet_t));
                        Q_push(&q_yocto_to_driving, wl4);
                        DBG_INFO("[T9-RX] WL-4 수신: 주행 정보 업데이트 (30s 주기)");
                    }
                // }
            }
        }

        loop_cnt++;
        if (loop_cnt >= wl4_trigger) loop_cnt = 0;
    }

    close(uart_fd);
    return NULL;
}














//#ifdef SIMULATION
        // --- 시뮬레이션 데이터 생성 (테스트용) ---
        /*if (loop_cnt > 0 && loop_cnt % 100 == 0) { 
            rx_buf[0] = TYPE_WL3;
            uint64_t aid = 2000 + loop_cnt;
            memcpy(&rx_buf[15], &aid, 8); 
            rx_len = 23;
        }
//#else
        // --- [수정] 실제 하드웨어 수신 (UART3 read) ---
        //rx_len = read(uart_fd, rx_buf, sizeof(rx_buf));
//#endif

        // [1] 데이터 수신 처리
        if (rx_len > 0) {
            if (rx_buf[0] == TYPE_WL3) {
                wl3_packet_t *wl3 = malloc(sizeof(wl3_packet_t));
                if (wl3) {
                    memcpy(wl3, rx_buf, sizeof(wl3_packet_t));
                    Q_push(&q_pkt_val, wl3);
                    printf("\x1b[31m[T9-RX] WL-3 Received (AID:0x%lX)\x1b[0m\n", wl3->accident_id);
                }
            } else if (rx_buf[0] == TYPE_WL4) {
                wl4_packet_t *wl4 = malloc(sizeof(wl4_packet_t));
                if (wl4) {
                    memcpy(wl4, rx_buf, sizeof(wl4_packet_t));
                    Q_push(&q_yocto_to_driving, wl4);
                    printf("\x1b[34m[T9-RX] WL-4 Received\x1b[0m\n");
                }
            }
        }

        // [2] 데이터 송신 처리 (WL-2 결과 전송)
        wl2_packet_t *wl2 = (wl2_packet_t *)Q_pop_nowait(&q_val_yocto);
        if (wl2) {
//#ifdef SIMULATION
            uint16_t distance = (wl2->dist_rsv >> 4); 
            printf("\x1b[35m[T9-TX-SIM] WL-2 Dist: %dm\x1b[0m\n", distance);
//#else
            // [수정] 실제 UART3 전송 (write)
            //int tx_res = write(uart_fd, wl2, sizeof(wl2_packet_t));
            //if (tx_res > 0) {
                //printf("\x1b[35m[T9-TX-HW] WL-2 Result Sent to Partner Pi\x1b[0m\n");
            //}
//#endif
            free(wl2);
        }
        loop_cnt++;
    }

    close(uart_fd);
    return NULL;
}


/*void* thread_yocto_if(void* arg) {
    (void)arg;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint32_t loop_cnt = 0;
    const uint32_t wl4_trigger = (PERIOD_WL4_S * 1000) / PERIOD_YOCTO_MS;

    DBG_INFO("Thread 9: Yocto Interface Module started.");
while (g_keep_running) {
        // 정밀 주기 제어 (예: 100ms)
        wait_next_period(&next_time, PERIOD_YOCTO_MS);

        uint8_t rx_buf[128] = {0};
        int rx_len = 0;

#ifdef SIMULATION
        // --- 시뮬레이션 데이터 생성 ---
        if (loop_cnt > 0 && loop_cnt % 100 == 0) { // 사고(WL-3)
            rx_buf[0] = TYPE_WL3;
            uint64_t aid = 2000 + loop_cnt;
            memcpy(&rx_buf[15], &aid, 8); 
            rx_len = 23;
        } else if (loop_cnt % wl4_trigger == 0) { // 주행(WL-4)
            rx_buf[0] = TYPE_WL4;
            uint16_t hdg = 180; int32_t alt = 150;
            memcpy(&rx_buf[1], &hdg, 2); memcpy(&rx_buf[3], &alt, 4);
            rx_len = 17;
        }
#else
        // --- 실제 하드웨어 수신 (i2c_io.c의 함수 사용) ---
        // I2C_BUS_YOCTO는 "/dev/i2c-1", YOCTO_ADDR은 상대방 주소
        rx_len = I2C_read_data(I2C_BUS_YOCTO, YOCTO_ADDR, rx_buf, sizeof(rx_buf));
#endif

        // [1] 데이터 수신 처리
        if (rx_len > 0) {
            if (rx_buf[0] == TYPE_WL3) {
                wl3_packet_t *wl3 = malloc(sizeof(wl3_packet_t));
                if (wl3) {
                    memcpy(wl3, rx_buf, sizeof(wl3_packet_t));
                    Q_push(&q_pkt_val, wl3);
                    printf("\x1b[31m[T9-RX] WL-3 High-Priority (AID:0x%lX)\x1b[0m\n", wl3->accident_id);
                }
            } else if (rx_buf[0] == TYPE_WL4) {
                wl4_packet_t *wl4 = malloc(sizeof(wl4_packet_t));
                if (wl4) {
                    memcpy(wl4, rx_buf, sizeof(wl4_packet_t));
                    Q_push(&q_yocto_to_driving, wl4);
                    printf("\x1b[34m[T9-RX] WL-4 Periodic Update\x1b[0m\n");
                }
            }
        }

        // [2] 데이터 송신 처리 (WL-2 결과 전송)
        wl2_packet_t *wl2 = (wl2_packet_t *)Q_pop_nowait(&q_val_yocto);
        if (wl2) {
#ifdef SIMULATION
            uint16_t distance = (wl2->dist_rsv >> 4); 
            printf("\x1b[35m[T9-TX-SIM] WL-2 Result -> Dist: %dm, Lane: %d\x1b[0m\n", distance, wl2->lane);
#else
            // 실제 I2C 전송 (i2c_io.c의 함수 사용)
            int tx_res = I2C_write_data(I2C_BUS_YOCTO, YOCTO_ADDR, (uint8_t*)wl2, sizeof(wl2_packet_t));
            if (tx_res > 0) {
                printf("\x1b[35m[T9-TX-HW] WL-2 Result Sent to Yocto\x1b[0m\n");
            }
#endif
            free(wl2); // 송신 후 메모리 해제
        }
        loop_cnt++;
    }
    return NULL;
}
*/


/*void *thread_yocto_if(void *arg) {
    sleep(2); // GPS 스레드가 좌표를 잡을 때까지 2초 대기
    (void)arg;
    //int step = 0;             // [추가] 이 줄이 반드시 있어야 합니다!
    uint16_t test_dist = 600;
    DBG_INFO("Thread 9: Yocto Interface Module started (Simulation Mode).");

    
    // 시연용 고정 사고 지점 설정 (수신차의 경로 앞쪽)
    // 수신차가 37.5650에서 시작하므로, 그보다 북쪽인 37.5665로 설정
    const double ACCIDENT_LAT = 37.566500;
    const double ACCIDENT_LON = 126.978000;
    const uint32_t ACCIDENT_ID = 0xAAAA; // 사고 고유 번호
    

    DBG_INFO("Thread 9: Yocto Interface Module started (Accident Vehicle 0x1111).");

    while (g_keep_running) {
       
        wl1_packet_t *pkt = malloc(sizeof(wl1_packet_t));
        if (!pkt) continue;
        memset(pkt, 0, sizeof(wl1_packet_t));
        
        // 1. 송신자 정보 설정
        pkt->sender.sender_id = 0x1111; // 내 ID
        pkt->header.ttl = 3;           // 중계 가능 횟수

        // 2. 사고 정보 및 고정 좌표 설정
        // 실제 GPS와 상관없이 시뮬레이션을 위해 고정된 사고 지점 좌표를 패킷에 담음
        pkt->accident.accident_id = ACCIDENT_ID;
        pkt->sender.lat_uDeg = (int32_t)(ACCIDENT_LAT * 1000000.0);
        pkt->sender.lon_uDeg = (int32_t)(ACCIDENT_LON * 1000000.0);
        
        pkt->accident.lane = 1;        // 1차선 사고
        pkt->accident.type = 3;        // 사고 유형 (예: 차량 고장)
        

        // 0. 안전하게 내 현재 위치 정보 가져오기
        gps_data_t my_pos = GPS_get_latest();
        int32_t my_lat_uDeg = (int32_t)(my_pos.lat * 1000000.0);
        int32_t my_lon_uDeg = (int32_t)(my_pos.lon * 1000000.0);
        

        Q_push(&q_pkt_sec_tx, pkt);
        printf("\x1b[31m[TX-1111] 사고 발생 송신 중! ID:0x%X, 위도:%.6f\x1b[0m\n", 
                ACCIDENT_ID, ACCIDENT_LAT); 
        // ----------------------------

        //step++;
        sleep(3); // 5초 대기
    }

    DBG_INFO("Thread 9: Yocto Interface Module terminating.");
    return NULL;
}
*/
