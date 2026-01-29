#ifndef WL_H
#define WL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>      // size_t 사용을 위해 추가
#include <netinet/in.h>
#include "proto_wl1.h"

#define WL_PORT 5555
//#define WL_PORT 8080

typedef struct {
    int sock;
    struct sockaddr_in tx_addr;
} wl_ctx_t;


bool WL_init(wl_ctx_t *ctx, bool is_tx);
bool WL_send_msg(wl_ctx_t *ctx, const void *buf, size_t len);
void WL_cleanup(wl_ctx_t *ctx);

void *thread_rx(void *arg);
void *thread_tx(void *arg);

#endif
