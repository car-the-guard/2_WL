#ifndef QUEUE_H
#define QUEUE_H

#include <pthread.h>

/*typedef enum {
    Q_SUCCESS = 0,    // 성공
    Q_ERR_EMPTY,      // 큐가 비어있음
    Q_ERR_FULL,       // 큐가 가득 참
    Q_ERR_NULL_PTR    // 잘못된 메모리 접근
} q_status_t;*/


// 큐의 각 노드 구조체
typedef struct node {
    void *data;         // 실제 패킷 데이터 (wl1_packet_t 등)
    struct node *next;
} node_t;

// 스레드 안전한 큐 구조체
typedef struct {
    node_t *head;
    node_t *tail;
    pthread_mutex_t mutex;  // 공유 자원 보호용 뮤텍스
    pthread_cond_t cond;    // 데이터 대기/알림용 조건 변수
} queue_t;

// 큐 관련 함수 선언
void Q_init(queue_t *q);
void Q_push(queue_t *q, void *data);
void *Q_pop(queue_t *q);
// [추가] 데이터가 없으면 기다리지 않고 즉시 NULL을 반환하는 함수 선언
void* Q_pop_nowait(queue_t *q); 

void Q_push(queue_t *q, void *data);
#endif
