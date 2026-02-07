#include "wl.h"
#include "debug.h"
#include "queue.h"
#include "pqueue.h"
//#include "i2c_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
extern volatile bool g_keep_running;

extern queue_t q_rx_sec_rx;       // RX 스레드가 수신 패킷을 넣는 큐
extern pqueue_t q_sec_tx_wl_tx;   // TX 스레드가 송신할 패킷을 가져오는 큐 (우선순위 큐)

 wl_ctx_t tx_ctx;
 wl_ctx_t rx_ctx;

// 1. WL_init 구현
bool WL_init(wl_ctx_t *ctx, bool is_tx) {
    ctx->sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->sock < 0) return false;

    int yes = 1;
    setsockopt(ctx->sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    
    /*// wlan0 바인딩
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "wlan0");
    setsockopt(ctx->sock, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr));
    */
    if (is_tx) {
        setsockopt(ctx->sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
        
        memset(&ctx->tx_addr, 0, sizeof(ctx->tx_addr));
        ctx->tx_addr.sin_family = AF_INET;
        ctx->tx_addr.sin_port = htons(WL_PORT);
        //inet_aton("10.0.0.255", &ctx->tx_addr.sin_addr);
        ctx->tx_addr.sin_addr.s_addr = inet_addr("10.0.0.255");
    } else {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(WL_PORT);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(ctx->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;
    }
    return true;
}


void *thread_rx(void *arg) {
    (void)arg;
    if (!WL_init(&rx_ctx, false)) return NULL;
    DBG_INFO("Thread 1: Wireless RX Module started.");

    while (g_keep_running) {
        wl1_packet_t *pkt = malloc(sizeof(wl1_packet_t));
        if (recv(rx_ctx.sock, pkt, sizeof(wl1_packet_t), 0) > 0) {
            //printf("\n\033[1;32m[T1-RX] Packet Received!\033[0m\n");
            DBG_INFO("\033[1;32m[RX] Packet Received!\033[0m");
            // 수신 성공 시 보안 모듈 큐에 넣기
            Q_push(&q_rx_sec_rx, pkt);
            //printf("pushing to q_rx_sec_rx.\n");
            printf("\n");
            fflush(stdout);
        } else {
            free(pkt);
            DBG_ERR("\n\03WL-1 Packet receive failed\n\033");
        }
    }
    close(rx_ctx.sock);
    return NULL;
}


void *thread_tx(void *arg) {
    (void)arg;
    if (!WL_init(&tx_ctx, true)) {
        printf("\033[1;31m[CRITICAL] TX Thread failed to initialize socket!\033[0m\n");
        return NULL;
    }
    DBG_INFO("Thread 8: Wireless TX Module started.");
    wl_ctx_t *ctx = &tx_ctx; 
    
    while (g_keep_running) {
        // 우선순위 큐에서 target_send_time_ms 도래한 패킷만 꺼냄
        wl1_delayed_packet_t *pkt = PQ_pop_wait_until_ready(
            &q_sec_tx_wl_tx, &g_keep_running);
        if (!pkt) continue;

        if (WL_send_msg(ctx, &(pkt->packet), sizeof(wl1_packet_t))) {
            DBG_INFO("\033[1;32m[TX] WL-1 Broadcast Success! (ID: 0x%lX)\033[0m", 
                   pkt->packet.sender.sender_id);

                   fflush(stdout); // 로그가 밀리지 않고 즉시 출력
            }
        // 사용이 끝난 메모리 해제
        free(pkt);
    }
    return NULL;
}

void WL_cleanup(wl_ctx_t *ctx) {
    if (ctx && ctx->sock >= 0) {
        close(ctx->sock);
        ctx->sock = -1;
    }
    DBG_INFO("[WL] Driver context cleaned up.");
}
bool WL_send_msg(wl_ctx_t *ctx, const void *buf, size_t len) {
    ssize_t sent = sendto(ctx->sock, buf, len, 0, (struct sockaddr *)&ctx->tx_addr, sizeof(ctx->tx_addr));
    //perror("\033[1;31m[WL-ERROR] sendto failed\033[0m"); // 왜 실패했는지 시스템 메시지 출력 // 이후에 코드 삭제하기
    return (sent >= 0);
    
}





