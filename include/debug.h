
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

// 실제 출력 함수 (src/debug.c에 구현)
void debug_init();
void debug_log(const char *level, const char *file, int line, const char *fmt, ...);

// 매크로 정의 (인자 전달 방식 수정)
#define DBG_ERR(fmt, ...)   debug_log("ERROR", __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define DBG_WARN(fmt, ...)  do { if (g_log_level >= LOG_WARN)  debug_log("WARN ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)
#define DBG_INFO(fmt, ...)  do { if (g_log_level >= LOG_INFO)  debug_log("INFO ", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)
#define DBG_DEBUG(fmt, ...) do { if (g_log_level >= LOG_DEBUG) debug_log("DEBUG", __FILE__, __LINE__, fmt, ##__VA_ARGS__); } while(0)

#endif
