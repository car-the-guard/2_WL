#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/* 기능 확인을 위한 최소 메시지 구조체 */
typedef struct {
    uint32_t accident_id;   // 사고 ID
    int32_t  relative_dist;  // VAL에서 계산된 거리
    uint16_t heading;       // Yocto가 보내줄 지자기 값
} spi_sim_msg_t;

/* 스레드 함수 선언 */
void *thread_spi(void *arg);

#endif
