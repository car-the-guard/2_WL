/*#ifndef I2C_IO_H
#define I2C_IO_H

#include <stdint.h>

// LCD 출력 모드 정의
#define MODE_TX    0  // 내 사고 송신
#define MODE_RELAY 1  // 타인 사고 재전송
#define MODE_ALERT 2  // 위험 경고 (LN, TYPE, DIST 표시)

// I2C 공통 함수
int LCD_open_and_init(void);
void LCD_init_device(int fd);
void LCD_display_v2x_mode(int mode, uint32_t sid, uint64_t aid, uint16_t dist, uint8_t lane, uint8_t type);

// I2C 기본 통신용 (기존 유지)
int  I2C_read_data(const char* device, uint8_t addr, uint8_t* buf, int len);
int  I2C_write_data(const char* device, uint8_t addr, uint8_t* buf, int len);


#endif
*/
#ifndef I2C_IO_H
#define I2C_IO_H

#include <stdint.h>

// ==========================================
// 1. 디스플레이 출력 모드 정의 (V2X 상태)
// ==========================================
#define MODE_TX    0  // 내 사고 송신 (차1)
#define MODE_RELAY 1  // 타인 사고 중계 (차2)
#define MODE_ALERT 2  // 위험 경고 수신 (차3)

// ==========================================
// 2. I2C 디스플레이 통합 제어 API
// ==========================================

/**
 * @brief 프로그램 시작 시 LCD(i2c-4)와 OLED(i2c-1)를 한 번에 열고 초기화합니다.
 * @return 성공 시 0, 실패 시 -1
 * @note main() 함수 시작부에서 딱 한 번 호출해야 합니다.
 */
int I2C_Display_Open_And_Init(void);

/**
 * @brief LCD와 OLED에 V2X 주행/사고 상태를 통합 출력합니다.
 * @param mode  출력 모드 (MODE_TX, MODE_RELAY, MODE_ALERT)
 * @param sid   송신자 ID (Sender ID)
 * @param aid   사고 ID (Accident ID)
 * @param dist  사고 지점까지의 거리 (m)
 * @param lane  현재 또는 사고 발생 차선
 * @param type  사고 유형/심각도
 * @note 내부적으로 전역 파일 디스크립터와 뮤텍스를 사용하여 스레드 안전을 보장합니다.
 */
void LCD_display_v2x_mode(int mode, uint32_t sid, uint64_t aid, uint16_t dist, uint8_t lane, uint8_t type);

// ==========================================
// 3. I2C 기본 통신 함수 (기존 센서 제어용 유지)
// ==========================================
int I2C_read_data(const char* device, uint8_t addr, uint8_t* buf, int len);
int I2C_write_data(const char* device, uint8_t addr, uint8_t* buf, int len);

#endif // I2C_IO_H