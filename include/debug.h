/*//# 로그 출력 매크로 및 설정
#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>
#include <time.h>

// 로그 레벨 정의
typedef enum {
    LOG_ERROR = 0,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG
} log_level_t;

// 전역 로그 레벨 설정 (기본 INFO)
extern log_level_t g_log_level;

/**
 * 로그 출력 매크로
 * 사용 예: DBG_INFO("Packet received from ID: 0x%X", id);
 */
/*
#define DBG(fmt, ...) \
    do { if (g_log_level >= LOG_DEBUG) debug_log("DEBUG", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)

#define DBG_INFO(fmt, ...) \
    do { if (g_log_level >= LOG_INFO) debug_log("INFO ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)

#define DBG_WARN(fmt, ...) \
    do { if (g_log_level >= LOG_WARN) debug_log("WARN ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)

#define DBG_ERR(fmt, ...) \
    do { debug_log("ERROR", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)


// 실제 출력 처리 함수
void DBG_DEBUG(const char *level,
               const char *file,
               int line,
               const char *fmt,
               ...);

#endif
*/
#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

typedef enum {
    LOG_ERROR = 0,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG
} log_level_t;

extern log_level_t g_log_level;

// 실제 출력 함수 (src/debug.c에 구현됨)
void debug_log(const char *level, const char *file, int line, const char *fmt, ...);

// 매크로 정의 (인자 전달 방식 수정)
#define DBG_ERR(fmt, ...)   debug_log("ERROR", __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define DBG_WARN(fmt, ...)  do { if (g_log_level >= LOG_WARN)  debug_log("WARN ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)
#define DBG_INFO(fmt, ...)  do { if (g_log_level >= LOG_INFO)  debug_log("INFO ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)
#define DBG_DEBUG(fmt, ...) do { if (g_log_level >= LOG_DEBUG) debug_log("DEBUG", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)

#endif
