#include "sec.h"
#include "queue.h"
#include "debug.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
extern bool g_keep_running;


extern queue_t q_pkt_sec_tx;   // 송신: T6(PKT) -> T7(SEC-TX)
extern queue_t q_sec_tx_wl_tx; // 송신: T7(SEC-TX) -> T8(WL-TX)


extern queue_t q_rx_sec_rx;    // 수신: T1(WL-RX) -> T2(SEC-RX)
extern queue_t q_sec_rx_pkt;   // 수신: T2(SEC-RX) -> T3(PKT)

/**
 * [보안 로직] 송신 패킷에 64바이트 서명을 추가
 */
void SEC_sign(wl1_packet_t *pkt) {
    if (!pkt) return;
    // 시뮬레이션을 위해 서명 필드에 특정 패턴(0xAA)을 채움
    memset(pkt->security.signature, 0xAA, sizeof(pkt->security.signature));
    DBG_DEBUG("SEC: Signed packet for Sender 0x%X", pkt->sender.sender_id);
}

/**
 * [보안 로직] 수신 패킷의 서명을 검증
 */
bool SEC_verify(wl1_packet_t *pkt) {
    if (!pkt) return false;
    // 시뮬레이션을 위해 모든 패킷을 검증 통과로 처리
    return true; 
}

/**
 * [Thread 1] 보안 송신 (SEC-TX)
 */
void *thread_sec_tx(void *arg) {
    (void)arg;
    DBG_INFO("Thread 7: Security TX Module started.");

    while (g_keep_running) {
        
        wl1_packet_t *msg = Q_pop(&q_pkt_sec_tx);
        if (msg == NULL) break; 

        SEC_sign(msg); 
        
        Q_push(&q_sec_tx_wl_tx, msg); 
    }

    DBG_INFO("Thread 7: Security TX Module terminating.");
    return NULL;
}

/**
 * [Thread 6] 보안 수신 (SEC-RX)
 */
void *thread_sec_rx(void *arg) {
    (void)arg;
    DBG_INFO("Thread 2: Security RX Module started.");

    while (g_keep_running) {
        
        wl1_packet_t *msg = Q_pop(&q_rx_sec_rx);
        if (msg == NULL) break; 

        if (SEC_verify(msg)) {
            DBG_DEBUG("SEC: Verification success for 0x%X", msg->sender.sender_id);
            Q_push(&q_sec_rx_pkt, msg); 
        } else {
            DBG_WARN("SEC: Verification failed for 0x%X! Dropping.", msg->sender.sender_id);
            free(msg); 
        }
    }

    DBG_INFO("Thread 2: Security RX Module terminating.");
    return NULL;
}
