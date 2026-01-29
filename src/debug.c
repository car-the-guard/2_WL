#include "debug.h"
#include <stdarg.h>
#include <string.h>
#include <time.h>

// 전역 로그 레벨 초기화
log_level_t g_log_level = LOG_INFO;

void debug_log(const char *level,
               const char *file,
               int line,
               const char *fmt,
               ...)
{
    char time_buf[64];
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);

    // [HH:MM:SS] 형식으로 시간 생성
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm);

    // [시간][로그레벨][파일명:라인] 헤더 출력
    printf("[%s][%s][%s:%d] ",
           time_buf,
           level,
           file,
           line);

    // 본문 내용 출력 (가변 인자 처리)
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    printf("\n");
    fflush(stdout); // 즉시 출력을 위해 스트림 비움
}
