#ifndef PQUEUE_H
#define PQUEUE_H

#include <stdint.h>
#include <pthread.h>
#include <stdbool.h>

#define PQ_DEFAULT_CAP 64

typedef uint32_t (*pq_get_key_fn)(const void *data);

typedef struct {
    void **heap;
    int size;
    int capacity;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pq_get_key_fn get_key;
} pqueue_t;

static inline bool time_before(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) < 0;
}
static inline bool time_after_eq(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) >= 0;
}

void  PQ_init(pqueue_t *pq, int capacity, pq_get_key_fn get_key);
void  PQ_destroy(pqueue_t *pq);
void  PQ_push(pqueue_t *pq, void *data);
void *PQ_pop(pqueue_t *pq);                                          // 블로킹: 데이터 오면 즉시 min 반환
void *PQ_pop_wait_until_ready(pqueue_t *pq, volatile bool *keep_running); // 블로킹: 시간 도래까지 대기
void  PQ_wake_all(pqueue_t *pq);

// 콜백 매칭 함수로 힙에서 조건에 맞는 element 제거. 제거된 개수 반환.
int   PQ_remove_if(pqueue_t *pq,
                   bool (*match)(const void *data, const void *ctx),
                   const void *ctx);

#endif
