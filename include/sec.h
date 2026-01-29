#ifndef SEC_H
#define SEC_H

#include <stdbool.h>
#include "proto_wl1.h"

// [ths6] 보안 송신 스레드: PKT -> SEC-TX -> WL-TX
void *thread_sec_tx(void *arg);

// [ths1] 보안 수신 스레드: WL-RX -> SEC-RX -> PKT
void *thread_sec_rx(void *arg);

// 보안 핵심 함수
void SEC_sign(wl1_packet_t *pkt);     // 디지털 서명 생성
bool SEC_verify(wl1_packet_t *pkt);   // 디지털 서명 검증

#endif
