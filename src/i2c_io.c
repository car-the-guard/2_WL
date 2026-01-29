
#include "i2c_io.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <pthread.h>

// 버스 및 주소 설정
#define LCD_BUS  "/dev/i2c-4"
#define OLED_BUS "/dev/i2c-1"
#define LCD_ADDR 0x27
#define OLED_ADDR 0x3C

#define LCD_CHR  1
#define LCD_CMD  0
#define LINE1    0x80
#define LINE2    0xC0
#define LCD_BACKLIGHT 0x08
#define ENABLE   0b00000100

static int g_fd_lcd = -1;
static int g_fd_oled = -1;
static pthread_mutex_t g_i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

// --- [OLED용 8x8 폰트 데이터] ---
static const unsigned char font8x8[][8] = {
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    [':'] = {0x00, 0x00, 0x24, 0x24, 0x00, 0x00, 0x00, 0x00},
    ['0'] = {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00},
    ['1'] = {0x00, 0x44, 0x42, 0x7E, 0x40, 0x40, 0x00, 0x00},
    ['2'] = {0x00, 0x62, 0x52, 0x4A, 0x46, 0x42, 0x00, 0x00},
    ['3'] = {0x3C, 0x42, 0x02, 0x3C, 0x02, 0x42, 0x3C, 0x00},
    ['4'] = {0x18, 0x14, 0x12, 0x7E, 0x10, 0x10, 0x00, 0x00},
    ['5'] = {0x2E, 0x4A, 0x4A, 0x4A, 0x4A, 0x32, 0x00, 0x00},
    ['6'] = {0x3C, 0x4A, 0x4A, 0x4A, 0x4A, 0x30, 0x00, 0x00},
    ['7'] = {0x02, 0x02, 0x02, 0x72, 0x0A, 0x06, 0x00, 0x00},
    ['8'] = {0x34, 0x4A, 0x4A, 0x4A, 0x4A, 0x34, 0x00, 0x00},
    ['9'] = {0x0C, 0x52, 0x52, 0x52, 0x52, 0x3C, 0x00, 0x00},
    ['A'] = {0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00, 0x00, 0x00},
    ['D'] = {0x7F, 0x41, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00},
    ['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00},
    ['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},
    ['N'] = {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00, 0x00},
    ['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41, 0x41, 0x00, 0x00},
    ['S'] = {0x22, 0x49, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00},
    ['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00, 0x00},
    ['m'] = {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00, 0x00},
    [','] = {0x00, 0x50, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}
};

// --- [제어 함수] ---
static void lcd_toggle_enable(int fd, uint8_t bits) {
    uint8_t data = (bits | ENABLE | LCD_BACKLIGHT);
    write(fd, &data, 1); usleep(600);
    data = (bits & ~ENABLE | LCD_BACKLIGHT);
    write(fd, &data, 1); usleep(600);
}

static void lcd_byte(int fd, uint8_t bits, uint8_t mode) {
    uint8_t hi = mode | (bits & 0xF0) | LCD_BACKLIGHT;
    uint8_t lo = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT;
    write(fd, &hi, 1); lcd_toggle_enable(fd, hi);
    write(fd, &lo, 1); lcd_toggle_enable(fd, lo);
}

static void oled_command(int fd, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    write(fd, buf, 2);
}

static void oled_data(int fd, uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    write(fd, buf, 2);
}

static void oled_set_pos(int fd, uint8_t x, uint8_t y) {
    x += 2; // SH1106 offset
    oled_command(fd, 0xB0 + y);
    oled_command(fd, ((x & 0xF0) >> 4) | 0x10);
    oled_command(fd, (x & 0x0F));
}

// --- [매끄러운 사선 정삼각형 + 굵은 느낌표] ---
static void oled_draw_mission_icon(int fd) {
    // 128x64 화면 버퍼 (메모리 맵핑)
    uint8_t buffer[128][8] = {0};

    // 정삼각형 (세로 56px 사용, Page 0~6)
    for (int y = 0; y < 56; y++) {
        // 사선을 매끄럽게 하기 위해 y값에 따른 x폭 계산
        // tan(30도) 근사값인 0.58 사용 (정삼각형의 한쪽 기울기)
        int width = (int)(y * 0.6); 
        int start_x = 64 - width;
        int end_x = 64 + width;

        for (int x = start_x; x <= end_x; x++) {
            if (x < 0 || x >= 128) continue;

            // [굵은 느낌표 & 상하 PADDING]
            // 느낌표 몸통 (굵기 6px, 상단 PADDING 8px)
            if (y >= 10 && y <= 35 && x >= 61 && x <= 67) continue;
            // 느낌표 점 (굵기 6px, 하단 PADDING 및 점 사이 간격)
            if (y >= 42 && y <= 48 && x >= 61 && x <= 67) continue;

            // 픽셀 세팅
            buffer[x][y / 8] |= (1 << (y % 8));
        }
    }

    // 버퍼 데이터를 OLED로 전송
    for (int p = 0; p < 7; p++) {
        oled_set_pos(fd, 0, p);
        for (int x = 0; x < 128; x++) {
            oled_data(fd, buffer[x][p]);
        }
    }
}

// --- [90도 회전 없는 가로 텍스트 출력] ---
static void oled_print_mission_text(int fd, const char *str) {
    int len = strlen(str);
    int x = (128 - (len * 8)) / 2; // 중앙 정렬
    if (x < 0) x = 0;

    while (*str) {
        unsigned char c = (unsigned char)*str;
        oled_set_pos(fd, x, 7); // 최하단 Page 7
        for (int i = 0; i < 8; i++) {
            oled_data(fd, font8x8[c][i]);
        }
        x += 8;
        str++;
    }
}

// --- [API: 초기화 함수] ---
int I2C_Display_Open_And_Init(void) {
    g_fd_lcd = open(LCD_BUS, O_RDWR);
    if (g_fd_lcd >= 0) {
        ioctl(g_fd_lcd, I2C_SLAVE, LCD_ADDR);
        uint8_t cmds[] = {0x33, 0x32, 0x28, 0x0C, 0x01};
        for(int i=0; i<5; i++) { lcd_byte(g_fd_lcd, cmds[i], LCD_CMD); usleep(5000); }
    }

    g_fd_oled = open(OLED_BUS, O_RDWR);
    if (g_fd_oled >= 0) {
        ioctl(g_fd_oled, I2C_SLAVE, OLED_ADDR);
        // 정방향 가로 출력을 위한 SH1106 초기화 시퀀스
        uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
        for(int i=0; i<sizeof(init); i++) oled_command(g_fd_oled, init[i]);
        // 초기 화면 클리어
        for(int p=0; p<8; p++) { oled_set_pos(g_fd_oled, 0, p); for(int x=0; x<130; x++) oled_data(g_fd_oled, 0x00); }
    }
    return (g_fd_lcd >= 0 && g_fd_oled >= 0) ? 0 : -1;
}

// --- [API: 통합 출력 함수] ---
void LCD_display_v2x_mode(int mode, uint32_t sid, uint64_t aid, uint16_t dist, uint8_t lane, uint8_t type) {
    pthread_mutex_lock(&g_i2c_mutex);

    char l1[17]={0}, l2[17]={0}, o_msg[32]={0};
    sprintf(o_msg, "LANE:%d, DIST:%dm", lane, dist);

    // LCD 텍스트 구성
    if (mode == 2) { 
    // 옵션 A: 차량 3 (ALERT 모드 - 최종 수신자)
    // Line 1: 16자 (공백 포함) - 최종 수신지임을 명시
    sprintf(l1, "* ALERT: FINAL *"); 
    
    // Line 2: 16자 (공백 포함) - 차2(CH2)로부터 받은 거리 표시
    // %4d를 사용하여 거리가 바뀌어도 글자 위치가 고정되도록 함
    sprintf(l2, "RECV FR CH2:%4dm", dist); 
    }
    else if (mode == 1) { sprintf(l1, "RELAYING...%04X", (uint16_t)sid); sprintf(l2, "DIST:%dm", dist); }
    else { sprintf(l1, "MY ACCIDENT!"); sprintf(l2, "L:%d AID:%04X", lane, (uint16_t)aid); }

    // 1. LCD 출력 (i2c-4)
    if (g_fd_lcd >= 0) {
        lcd_byte(g_fd_lcd, 0x01, LCD_CMD); usleep(2000);
        lcd_byte(g_fd_lcd, LINE1, LCD_CMD); for(int i=0; i<strlen(l1); i++) lcd_byte(g_fd_lcd, l1[i], LCD_CHR);
        lcd_byte(g_fd_lcd, LINE2, LCD_CMD); for(int i=0; i<strlen(l2); i++) lcd_byte(g_fd_lcd, l2[i], LCD_CHR);
    }

    // 2. OLED 출력 (i2c-1)
    if (g_fd_oled >= 0) {
        // 화면 클리어
        for(int p=0; p<8; p++) { oled_set_pos(g_fd_oled, 0, p); for(int x=0; x<130; x++) oled_data(g_fd_oled, 0x00); }
        
        // 아이콘 그리기 및 가로 텍스트 출력
        oled_draw_mission_icon(g_fd_oled); 
        oled_print_mission_text(g_fd_oled, o_msg);
    }

    pthread_mutex_unlock(&g_i2c_mutex);
}






