#include "debug.h"
#include <stdarg.h>
#include <string.h>
#include <time.h>

//[추가]
#include <sys/time.h>
#include <stdlib.h> // system() 사용

// 전역 로그 레벨 초기화
log_level_t g_log_level = LOG_INFO;

void debug_init() {
    // 프로그램 시작 시 인터넷이 연결되어 있다면 시간 동기화 
    system("sudo ntpdate -u pool.ntp.org"); 
    // 환경 변수 TZ를 한국 시간(KST-9)으로 설정
    setenv("TZ", "KST-9", 1);
    tzset();
    printf("[DEBUG] Log System Initialized (KST Timezone applied)\n");
}

void debug_log(const char *level,
               const char *file,
               int line,
               const char *fmt,
               ...)
{
    char time_buf[64];
    struct timeval tv;
    gettimeofday(&tv, NULL); // 현재 시간(초, 마이크로초) 가져오기
    time_t t = time(NULL);

    struct tm *tm = localtime(&t);

    // 에러 해결 지점: long 타입을 명시하여 millis 선언
    long millis = tv.tv_usec / 1000;

    //[추가]
    // ms 포함 형식: [HH:MM:SS.mmm]
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm);
    printf("[%s.%03ld][%s][%s:%d] ", time_buf, millis, level, file, line);
    // [HH:MM:SS] 형식으로 시간 생성
    //strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm);

    /*[시간][로그레벨][파일명:라인] 헤더 출력
    //printf("[%s][%s][%s:%d] ",
           time_buf,
           level,
           file,
           line);
    */
    // 본문 내용 출력 (가변 인자 처리)
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    printf("\n");
    fflush(stdout); // 즉시 출력을 위해 스트림 비움
}
