#include "queue.h"
#include <stdlib.h>

/**
 * 큐 초기화
 */
void Q_init(queue_t *q) {
    q->head = q->tail = NULL;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
}

/**
 * 데이터 넣기 (Push)
 */
void Q_push(queue_t *q, void *data) {
    node_t *new_node = (node_t *)malloc(sizeof(node_t));
    new_node->data = data;
    new_node->next = NULL;

    pthread_mutex_lock(&q->mutex);
    
    if (q->tail == NULL) {
        q->head = q->tail = new_node;
    } else {
        q->tail->next = new_node;
        q->tail = new_node;
    }

    // 데이터가 들어왔음을 대기 중인 스레드에게 알림
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

/**
 * 데이터 가져오기 (Pop)
 * 데이터가 없으면 데이터가 들어올 때까지 스레드가 블로킹(대기).
 */
void *Q_pop(queue_t *q) {
    pthread_mutex_lock(&q->mutex);

    // 큐가 비어있으면 데이터가 들어올 때까지 대기
    while (q->head == NULL) {
        pthread_cond_wait(&q->cond, &q->mutex);
    }

    node_t *temp = q->head;
    void *data = temp->data;
    q->head = q->head->next;

    if (q->head == NULL) {
        q->tail = NULL;
    }

    free(temp);
    pthread_mutex_unlock(&q->mutex);

    return data;
}

void* Q_pop_nowait(queue_t *q) {
    pthread_mutex_lock(&q->mutex);

    // 1. 큐가 비어있는지 확인 (연결 리스트 방식에서는 head가 NULL인지 체크)
    if (q->head == NULL) {
        pthread_mutex_unlock(&q->mutex);
        return NULL; // 데이터가 없으므로 즉시 리턴
    }

    // 2. 데이터 꺼내기 (기존 Q_pop 로직과 동일하지만 대기 과정만 없음)
    node_t *temp = q->head;
    void *data = temp->data;
    q->head = q->head->next;

    // 3. 마지막 노드였다면 tail도 정리
    if (q->head == NULL) {
        q->tail = NULL;
    }

    free(temp);
    pthread_mutex_unlock(&q->mutex);

    return data;
}
