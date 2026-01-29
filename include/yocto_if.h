#ifndef YOCTO_IF_H
#define YOCTO_IF_H

#include <stdint.h>
#include <stdbool.h>
#include "proto_wl4.h"

// UART 설정
#include <fcntl.h>    // open()
#include <termios.h>  // UART 설정을 위한 termios 구조체
#include <unistd.h>   // read(), write(), close()

// 시뮬레이션 모드 활성화 (하드웨어 연결 시 주석 처리)
// #define SIMULATION 

// 통신 설정
// UART 장치 경로
#define UART3_DEV "/dev/ttyAMA3"
#define PERIOD_YOCTO_MS 50    // 50ms 고속 루프
#define PERIOD_WL4_S    30    // 30s 주행정보 필터링

// 패킷 타입
#define TYPE_WL2 0x02  // Wireless -> Yocto (상태 보고)
#define TYPE_WL3 0x03
#define TYPE_WL4 0x04


// UART 초기화 함수 선언
int UART3_init(void);

// 스레드 함수
void* thread_yocto_if(void* arg);


#endif
