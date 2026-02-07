#include "pqueue.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

static uint32_t get_now_ms_u32(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)((uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000);
}

static void swap(void **a, void **b) {
    void *tmp = *a;
    *a = *b;
    *b = tmp;
}

static void bubble_up(pqueue_t *pq, int idx) {
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        uint32_t key_idx = pq->get_key(pq->heap[idx]);
        uint32_t key_par = pq->get_key(pq->heap[parent]);
        if (time_before(key_idx, key_par)) {
            swap(&pq->heap[idx], &pq->heap[parent]);
            idx = parent;
        } else {
            break;
        }
    }
}

static void bubble_down(pqueue_t *pq, int idx) {
    while (1) {
        int smallest = idx;
        int left  = 2 * idx + 1;
        int right = 2 * idx + 2;

        if (left < pq->size &&
            time_before(pq->get_key(pq->heap[left]),
                        pq->get_key(pq->heap[smallest])))
            smallest = left;

        if (right < pq->size &&
            time_before(pq->get_key(pq->heap[right]),
                        pq->get_key(pq->heap[smallest])))
            smallest = right;

        if (smallest != idx) {
            swap(&pq->heap[idx], &pq->heap[smallest]);
            idx = smallest;
        } else {
            break;
        }
    }
}

static void *heap_extract_min(pqueue_t *pq) {
    void *data = pq->heap[0];
    pq->size--;
    if (pq->size > 0) {
        pq->heap[0] = pq->heap[pq->size];
        bubble_down(pq, 0);
    }
    return data;
}

void PQ_init(pqueue_t *pq, int capacity, pq_get_key_fn get_key) {
    pq->capacity = capacity;
    pq->size = 0;
    pq->get_key = get_key;
    pq->heap = (void **)malloc(sizeof(void *) * capacity);
    pthread_mutex_init(&pq->mutex, NULL);
    pthread_cond_init(&pq->cond, NULL);
}

void PQ_destroy(pqueue_t *pq) {
    pthread_mutex_lock(&pq->mutex);
    for (int i = 0; i < pq->size; i++)
        free(pq->heap[i]);
    free(pq->heap);
    pq->heap = NULL;
    pq->size = 0;
    pthread_mutex_unlock(&pq->mutex);
    pthread_mutex_destroy(&pq->mutex);
    pthread_cond_destroy(&pq->cond);
}

void PQ_push(pqueue_t *pq, void *data) {
    pthread_mutex_lock(&pq->mutex);
    if (pq->size >= pq->capacity) {
        pq->capacity *= 2;
        pq->heap = (void **)realloc(pq->heap, sizeof(void *) * pq->capacity);
    }
    pq->heap[pq->size] = data;
    bubble_up(pq, pq->size);
    pq->size++;
    pthread_cond_signal(&pq->cond);
    pthread_mutex_unlock(&pq->mutex);
}

void *PQ_pop(pqueue_t *pq) {
    pthread_mutex_lock(&pq->mutex);
    while (pq->size == 0) {
        pthread_cond_wait(&pq->cond, &pq->mutex);
    }
    void *data = heap_extract_min(pq);
    pthread_mutex_unlock(&pq->mutex);
    return data;
}

void *PQ_pop_wait_until_ready(pqueue_t *pq, volatile bool *keep_running) {
    pthread_mutex_lock(&pq->mutex);

    while (*keep_running) {
        while (pq->size == 0 && *keep_running) {
            pthread_cond_wait(&pq->cond, &pq->mutex);
        }
        if (!*keep_running) break;

        uint32_t target = pq->get_key(pq->heap[0]);
        uint32_t now = get_now_ms_u32();

        if (time_after_eq(now, target)) {
            void *data = heap_extract_min(pq);
            pthread_mutex_unlock(&pq->mutex);
            return data;
        }

        uint32_t remaining_ms = target - now;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += (long)(remaining_ms % 1000) * 1000000L;
        ts.tv_sec  += remaining_ms / 1000;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        pthread_cond_timedwait(&pq->cond, &pq->mutex, &ts);
    }

    pthread_mutex_unlock(&pq->mutex);
    return NULL;
}

void PQ_wake_all(pqueue_t *pq) {
    pthread_mutex_lock(&pq->mutex);
    pthread_cond_broadcast(&pq->cond);
    pthread_mutex_unlock(&pq->mutex);
}
