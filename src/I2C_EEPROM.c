/****************************************************************************************************************************
 * Project    : I2C(TWI)를 이용한 EEPROM(M24C08) 제어 실습
 * Author     : 배민서 (20183369)
 * Board      : KUT-128 Combo Board (OHM/RAC company)
 * Tool       : CodeVisionAVR (CVAVR)
 *--------------------------------------------------------------------------------------------------------------------------
 * [기술 스택]
 *   - I2C (Bit-Bang 방식, TWI 하드웨어 미사용)
 *   - EEPROM : M24C08 (1KB, I2C 인터페이스)
 *   - 구현 기능 : Byte Write, Page Write, Random Read, Sequential Read
 *--------------------------------------------------------------------------------------------------------------------------
 * [I2C 핀 구성]
 *   - SCL : PD0 (출력)
 *   - SDA : PD1 (입출력 전환)
 *--------------------------------------------------------------------------------------------------------------------------
 * [EEPROM 주소]
 *   - Device Write Address : 0xA0
 *   - Device Read  Address : 0xA1
 ****************************************************************************************************************************/

#include <mega128a.h>
#include <delay.h>

/*============================ 핀 제어 매크로 ============================*/
#define SCL_OUT     DDRD |= 0x01    /* PD0 출력 설정 */
#define SDA_OUT     DDRD |= 0x02    /* PD1 출력 설정 */
#define SDA_IN      DDRD &= 0xFD    /* PD1 입력 설정 */

#define CLK_HIGH    PORTD |= 0x01   /* SCL HIGH */
#define CLK_LOW     PORTD &= 0xFE   /* SCL LOW  */
#define DAT_HIGH    PORTD |= 0x02   /* SDA HIGH */
#define DAT_LOW     PORTD &= 0xFD   /* SDA LOW  */

/*============================ typedef ============================*/
typedef unsigned char U8;

/*============================ 전역 변수 ============================*/
U8 DEV_ADD_W = 0xA0;               /* EEPROM Write 주소 */
U8 DEV_ADD_R = 0xA1;               /* EEPROM Read  주소 */
U8 read_buffer[4];                 /* Sequential Read 수신 버퍼 */
U8 write_data[4] = {0x11, 0x22, 0x33, 0x44}; /* Page Write 테스트 데이터 */
U8 READ_DATA = 0;                  /* Random Read 수신 데이터 */

/*============================ I2C 기본 함수 ============================*/

/* START 조건 : SDA HIGH -> LOW (SCL HIGH 상태에서) */
void IIC_START(void) {
    DAT_HIGH; CLK_HIGH; delay_us(5);
    DAT_LOW;  delay_us(5);
    CLK_LOW;  delay_us(5);
}

/* STOP 조건 : SDA LOW -> HIGH (SCL HIGH 상태에서) */
void IIC_STOP(void) {
    DAT_LOW;  CLK_LOW;  delay_us(5);
    CLK_HIGH; delay_us(5);
    DAT_HIGH; delay_us(5);
}

/* 8비트 데이터 송신 (MSB first) */
void Process_8data_send(U8 value) {
    U8 i;
    for(i = 0; i < 8; i++) {
        if((value << i) & 0x80) DAT_HIGH; else DAT_LOW;
        CLK_HIGH; delay_us(5);
        CLK_LOW;  delay_us(5);
    }
}

/* 8비트 데이터 수신 (MSB first) */
U8 Process_8data_receive(void) {
    U8 i, data = 0;
    SDA_IN;
    for (i = 0; i < 8; i++) {
        CLK_HIGH; delay_us(5);
        data <<= 1;
        if ((PIND & 0x02) != 0x00) data |= 0x01;
        CLK_LOW; delay_us(5);
    }
    SDA_OUT;
    return data;
}

/* ACK 수신 대기 */
void Wait_ACK(void) {
    SDA_IN;
    CLK_HIGH; delay_us(5);
    CLK_LOW;  delay_us(5);
    SDA_OUT;
}

/* ACK 송신 (연속 수신 시 사용) */
void Send_ACK(void) {
    DAT_LOW;
    CLK_HIGH; delay_us(5);
    CLK_LOW;  delay_us(5);
}

/* NoACK 송신 (마지막 수신 후 사용) */
void Send_NoACK(void) {
    DAT_HIGH;
    CLK_HIGH; delay_us(5);
    CLK_LOW;  delay_us(5);
}

/*============================ EEPROM 제어 함수 ============================*/

/* Byte Write : 1바이트를 지정 주소에 기록 */
void Byte_write(U8 addr, U8 data) {
    IIC_START();
    Process_8data_send(DEV_ADD_W); Wait_ACK();
    Process_8data_send(addr);      Wait_ACK();
    Process_8data_send(data);      Wait_ACK();
    IIC_STOP();
}

/* Page Write : 연속된 주소에 여러 바이트 기록 */
void Page_write(U8 start_addr, U8 *data_ptr, U8 length) {
    U8 i;
    IIC_START();
    Process_8data_send(DEV_ADD_W);  Wait_ACK();
    Process_8data_send(start_addr); Wait_ACK();
    for(i = 0; i < length; i++) {
        Process_8data_send(data_ptr[i]);
        Wait_ACK();
    }
    IIC_STOP();
}

/* Random Read : 지정 주소의 1바이트 읽기 */
void Random_read(U8 addr) {
    IIC_START();
    Process_8data_send(DEV_ADD_W); Wait_ACK();
    Process_8data_send(addr);      Wait_ACK();
    IIC_START();
    Process_8data_send(DEV_ADD_R); Wait_ACK();
    READ_DATA = Process_8data_receive();
    Send_NoACK();
    IIC_STOP();
}

/* Sequential Read : 연속된 주소에서 여러 바이트 읽기 */
void Sequential_Read(U8 start_addr, U8 length) {
    U8 i;
    IIC_START();
    Process_8data_send(DEV_ADD_W);  Wait_ACK();
    Process_8data_send(start_addr); Wait_ACK();
    IIC_START();
    Process_8data_send(DEV_ADD_R);  Wait_ACK();
    for(i = 0; i < length; i++) {
        read_buffer[i] = Process_8data_receive();
        if(i < (length - 1)) Send_ACK();
        else                  Send_NoACK();
    }
    IIC_STOP();
}

/****************************************************************************************************************************
 *                                                main Function
 ****************************************************************************************************************************/
void main(void) {
    SCL_OUT;
    SDA_OUT;
    DDRC  = 0xFF;   /* LED 포트 출력 설정 */
    PORTC = 0xFF;   /* 초기 상태: LED 모두 끔 (Active Low 기준) */

    while(1) {

        /* [실습 1] Byte Write 테스트
        Byte_write(0x80, 0xAA);
        delay_ms(10);
        */

        /* [실습 2] Random Read 테스트
        Random_read(0x80);
        delay_ms(10);
        */

        /* [실습 3] Page Write 테스트 (0x80번지부터 4바이트 기록)
        Page_write(0x80, write_data, 4);
        delay_ms(10);
        */

        /* [실습 4] Sequential Read 테스트 (기록된 4바이트 확인)
        Sequential_Read(0x80, 4);
        delay_ms(10);
        */
    }
}
