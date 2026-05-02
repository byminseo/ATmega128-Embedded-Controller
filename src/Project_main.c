/****************************************************************************************************************************
 * Project    : Project2 - 8bit AVR(ATmega128) 기반 통합 제어 시스템
 * Author     : 배민서 (20183369)
 * Modify     : 2026.04.14
 * Date       : 2026.04.14
 * Board      : KUT-128 Combo Board (OHM/RAC company)
 * Tool       : CodeVisionAVR (CVAVR)
 *--------------------------------------------------------------------------------------------------------------------------
 * [기술 스택]
 *   1. CPU clock           : 16MHz
 *   2. External Interrupt  : 4채널 (INT4 ~ INT7)
 *                            - Rising  edge : INT4, INT6
 *                            - Falling edge : INT5, INT7
 *                            - Nesting 제한 (ISR 진입 시 SREG의 I-flag 차단)
 *                            - ISR : Bit flag 방식 (수 us 단위 처리, 실제 작업은 main에서)
 *   3. Timer               : 3채널
 *                            - Timer1 : 16bit CTC mode, 100ms 기본 주기 -> 69회 누적 -> 6900ms
 *                            - Timer2 :  8bit OVF mode,   1ms 기본 주기 -> 4500회 누적 -> 4500ms
 *                            - Timer3 : 16bit PWM output (OC3A)
 *   4. PWM                 : 9bit Fast PWM (Mode 6) / 9bit Phase Correct PWM (Mode 2)
 *                            - Duty : 69% (학번 뒤 2자리)
 *                            - OC3A pin (PE3) 출력
 *                            - Fast PWM 주파수     : 16MHz / (8 x 512)     = 3.906 kHz
 *                            - Phase Correct 주파수: 16MHz / (8 x 2 x 511) = 1.957 kHz
 *   5. UART0               : 9600bps, 8 data bit, No parity, 1 stop bit (8N1 frame)
 *                            - TX(송신) + RX Complete Interrupt(수신) 모두 사용
 *                            - ADC 전압 data PC 전송
 *                            - RX 명령 제어 : T/V/F/P/S 문자로 원격 트리거
 *   6. ADC                 : 10bit resolution (0 ~ 1023)
 *                            - ADC clock : 16MHz / 128 = 125 kHz
 *                            - Conversion time : 104 us (normal) / 200 us (first)
 *                            - ADC7 (PF7) : LM35DZ 온도센서 (10mV/도), Free Running mode
 *                            - ADC6 (PF6) : VR2 가변저항(10kohm) 전압, Single mode
 *--------------------------------------------------------------------------------------------------------------------------
 * [설계 원칙]
 *   - MISRA C 가이드라인 참조 (주석 style /* */ only, typedef 사용, volatile 사용, <stdio.h> 미사용)
 *   - 1-Period 측정 : PC0 pin HIGH/LOW 토글 (오실로스코프로 실행 시간 측정)
 *                     while 진입 전        : PC0 = LOW
 *                     while 루프 시작 시점 : PC0 = HIGH
 *                     while 루프 끝 시점   : PC0 = LOW
 *                     Min(Best case)  : 학번 표시만, 플래그 없음    = 약 4 ms
 *                     Max(Worst case) : 모든 flag + ADC + UART 송신 = 약 10~15 ms (실측 필요)
 *   - 변수 선언     : typedef (U8, U16, U32)
 *   - ISR 최소 시간 : 플래그 set 동작만, 실제 처리는 main loop에서 수행
 *   - JTAG          : PF6/PF7 ADC 사용을 위해 소프트웨어로 비활성화
 ****************************************************************************************************************************/

#include <mega128.h>
#include <delay.h>
/* <stdio.h> 미사용 (MISRA C Rule 21.6 : standard library stdio.h 사용 금지)
   printf 대체 : UART0_transmit / UART0_print / UART0_send_voltage 직접 구현 */

/*============================ typedef =============================*/
typedef unsigned char  U8;
typedef unsigned short U16;
typedef unsigned int   U32;

/*============================ 7-segment 패턴 (공통 음극) ============================*/
/* 비트 순서 : dp g f e d c b a (1 = ON, 0 = OFF)
   숫자 0 ~ 9의 7-segment 점등 패턴 */
flash U8 seg_pat[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/*============================ 학번 정의 ============================*/
/* 학번 : 20183369
   앞 4자리 : 2018
   뒤 4자리 : 3369
   뒤 2자리 : 69 (Timer 주기, PWM Duty 산출 기준) */
#define ID_FRONT_THOUSAND  2
#define ID_FRONT_HUNDRED   0
#define ID_FRONT_TEN       1
#define ID_FRONT_ONE       8

#define ID_BACK_THOUSAND   3
#define ID_BACK_HUNDRED    3
#define ID_BACK_TEN        6
#define ID_BACK_ONE        9

#define ID_LAST_TWO        69         /* 학번 뒤 2자리 */
#define ADC_TEMP_PERIOD_MS 4500       /* 온도 측정 주기 4500ms (Timer2 카운터 누적 목표) */
#define ADC_TEMP_MAX_CNT   10         /* 온도 표시 반복 횟수 */
#define PWM_DUTY_PERCENT   ID_LAST_TWO                                      /* PWM Duty = 69% */
#define PWM_TOP_9BIT       511UL                                            /* 9bit PWM TOP 값 */
#define PWM_OCR_VALUE      ((PWM_TOP_9BIT * PWM_DUTY_PERCENT) / 100UL)      /* 511 x 69 / 100 = 352 */

/*============================ 1-Period 측정용 핀 ============================*/
/* PC0 pin : while 루프 1주기 HIGH/LOW 토글용
   오실로스코프 측정 시 HIGH 구간 폭 = 실행 시간 (Best/Worst case 관찰 가능) */
#define PERIOD_PIN_MASK    0x01       /* PC0 = bit 0 */

/*============================ 전역 플래그 / 변수 ============================*/
/* External Interrupt flag */
volatile U8  EINT4_FLAG_BIT = 0;
volatile U8  EINT5_FLAG_BIT = 0;
volatile U8  EINT6_FLAG_BIT = 0;
volatile U8  EINT7_FLAG_BIT = 0;

/* Timer interrupt flag */
volatile U8  TIMER2_OVF_FLAG_BIT = 0;
volatile U8  TIMER1_CTC_FLAG_BIT = 0;

/* UART RX flag (수신 완료 Interrupt 발생 시 set) */
volatile U8   UART_RX_FLAG_BIT = 0;
volatile char UART_RX_CHAR     = 0;

/* Timer 누적 카운터 */
volatile U16 timer2_ms_count    = 0;   /* Timer2 OVF 1ms씩 누적  -> 4500ms 측정용 */
volatile U8  timer1_100ms_count = 0;   /* Timer1 CTC 100ms씩 누적 -> 6900ms 측정용 */

/* 동작 상태 */
volatile U8  display_mode    = 0;      /* 0 : 학번 표시, 1 : 온도 표시 */
volatile U8  temp_disp_count = 0;      /* 온도 표시 누적 횟수 (10회 도달 시 학번 복귀) */
volatile U16 current_temp_x10 = 0;     /* 현재 온도 x 10 (예 : 31.2도 -> 312) */

/*============================ 함수 프로토타입 ============================*/
void Port_init(void);
void EINT_init(void);
void UART0_init(void);
void JTAG_disable(void);

void Seg4_out (U8 N1, U8 N10, U8 N100, U8 N1000);    /* 초기 학번 표시 (40회 반복) */
void Seg4_out2(U8 N1, U8 N10, U8 N100, U8 N1000);    /* while 내에서 1회 스캔 */
void Seg_temp_disp(U16 temp_x10);                    /* 온도 표시 (XX.X 형태) */

void Timer2_start(void);
void Timer2_stop (void);
void Timer1_start(void);
void Timer3_FastPWM_start(void);
void Timer3_PhaseCorrectPWM_start(void);
void Timer3_stop(void);

U16  ADC_read_temp   (void);          /* ADC7 (LM35) Free Running */
U16  ADC_read_voltage(void);          /* ADC6 (VR2)  Single       */

void UART0_transmit(char data);                 /* 1문자 송신 */
void UART0_print   (char *str);                 /* 문자열 송신 */
void UART0_put_dec (U16 value);                 /* 10진수 정수 송신 */
void UART0_send_voltage(U16 v_x100);            /* "Voltage: X.XX V\r\n" 형식 */

/****************************************************************************************************************************
 *                                                main Function
 ****************************************************************************************************************************/
void main(void)
{
    /*--- (1) JTAG 비활성화 : PF6, PF7 ADC 사용을 위한 전제 ---*/
    JTAG_disable();

    /*--- (2) 포트/주변장치 초기화 ---*/
    Port_init();
    EINT_init();
    UART0_init();

    /*--- (3) while(1) 진입 전 학번 8자리 표시 (2018 -> 3369) ---*/
    Seg4_out(ID_FRONT_ONE, ID_FRONT_TEN, ID_FRONT_HUNDRED, ID_FRONT_THOUSAND);   /* 2018 */
    Seg4_out(ID_BACK_ONE,  ID_BACK_TEN,  ID_BACK_HUNDRED,  ID_BACK_THOUSAND);    /* 3369 */

    /*--- (4) UART 초기 메시지 송신 (부팅 확인용) ---*/
    UART0_print("\r\n=== Project2 Start (ID: 20183369) ===\r\n");
    UART0_print("Commands : T=Temp, V=Voltage, F=FastPWM, P=PhasePWM, S=Status\r\n");

    /*--- (5) 전역 인터럽트 활성화 (I-flag set) ---*/
    SREG |= 0x80;

    /*--- (6) 1-Period 측정용 PC0 = LOW (while 진입 전 상태) ---*/
    PORTC &= ~PERIOD_PIN_MASK;

    /*========================================================================
     *                            MAIN LOOP
     *  1-Period 실행 시간 측정 : PC0 pin HIGH 구간 폭 = 실행 시간
     *   - Min (Best case)  : 학번 표시만, 플래그 없음 = 약 4 ms (Seg4_out2 × 4)
     *   - Max (Worst case) : Flag 동시 set + ADC + UART 송신 = 약 10~15 ms
     *======================================================================*/
    while(1)
    {
        PORTC |= PERIOD_PIN_MASK;          /* 1-Period START : PC0 = HIGH */

        /*=========================================================
         * (A) 디스플레이 : 모드에 따라 학번 또는 온도 표시
         *=========================================================*/
        if (display_mode == 0) {
            /* 학번 모드 : 뒤 4자리(3369) 계속 표시 */
            Seg4_out2(ID_BACK_ONE, ID_BACK_TEN, ID_BACK_HUNDRED, ID_BACK_THOUSAND);
        } else {
            /* 온도 모드 : XX.X도 표시 */
            Seg_temp_disp(current_temp_x10);
        }

        /*=========================================================
         * (B) UART RX flag 처리 : 수신 문자에 따라 원격 트리거
         *     UART 명령으로 set된 EINTx 플래그는 아래 (C)에서 같은 루프 내 처리됨
         *=========================================================*/
        if (UART_RX_FLAG_BIT == 1) {
            UART_RX_FLAG_BIT = 0;
            switch (UART_RX_CHAR) {
                case 'T' : case 't' :
                    EINT4_FLAG_BIT = 1;         /* 온도 측정 수동 트리거 */
                    UART0_print("[CMD] Temp measurement start\r\n");
                    break;
                case 'V' : case 'v' :
                    EINT5_FLAG_BIT = 1;         /* 전압 측정 수동 트리거 */
                    UART0_print("[CMD] Voltage measurement start\r\n");
                    break;
                case 'F' : case 'f' :
                    EINT6_FLAG_BIT = 1;         /* Fast PWM 수동 트리거 */
                    UART0_print("[CMD] Fast PWM start\r\n");
                    break;
                case 'P' : case 'p' :
                    EINT7_FLAG_BIT = 1;         /* Phase Correct PWM 수동 트리거 */
                    UART0_print("[CMD] Phase Correct PWM start\r\n");
                    break;
                case 'S' : case 's' :
                    UART0_print("[STATUS] Mode: ");
                    UART0_print((display_mode == 0) ? "ID\r\n" : "Temp\r\n");
                    break;
                default :
                    /* 알 수 없는 문자는 무시 */
                    break;
            }
        }

        /*=========================================================
         * (C) External Interrupt flag 처리
         *=========================================================*/
        if (EINT4_FLAG_BIT == 1) {
            EINT4_FLAG_BIT = 0;
            /* INT4 (Rising) : 온도 측정 모드 진입 */
            display_mode    = 1;
            temp_disp_count = 0;
            timer2_ms_count = 0;
            Timer2_start();
        }

        if (EINT5_FLAG_BIT == 1) {
            EINT5_FLAG_BIT = 0;
            /* INT5 (Falling) : Timer1 CTC 시작 -> 6900ms 후 ADC + UART 전송 */
            timer1_100ms_count = 0;
            Timer1_start();
        }

        if (EINT6_FLAG_BIT == 1) {
            EINT6_FLAG_BIT = 0;
            /* INT6 (Rising) : Timer3 9bit Fast PWM 시작 (Duty 69%) */
            Timer3_FastPWM_start();
        }

        if (EINT7_FLAG_BIT == 1) {
            EINT7_FLAG_BIT = 0;
            /* INT7 (Falling) : Fast PWM 정지 -> Phase Correct PWM 전환 (Duty 69%) */
            Timer3_stop();
            Timer3_PhaseCorrectPWM_start();
        }

        /*=========================================================
         * (D) Timer2 OVF flag 처리 : 4500ms 도달 시 온도 측정 + 10회 카운트
         *=========================================================*/
        if (TIMER2_OVF_FLAG_BIT == 1) {
            TIMER2_OVF_FLAG_BIT = 0;
            timer2_ms_count++;                      /* 1 ms 단위 누적 */

            if (timer2_ms_count >= ADC_TEMP_PERIOD_MS) {
                timer2_ms_count = 0;
                current_temp_x10 = ADC_read_temp();
                temp_disp_count++;

                if (temp_disp_count >= ADC_TEMP_MAX_CNT) {
                    /* 10회 도달 -> Timer2 정지 후 학번 표시 복귀 */
                    Timer2_stop();
                    display_mode    = 0;
                    temp_disp_count = 0;
                }
            }
        }

        /*=========================================================
         * (E) Timer1 CTC flag 처리 : 6900ms 도달 시 ADC 전압 + UART 전송
         *=========================================================*/
        if (TIMER1_CTC_FLAG_BIT == 1) {
            TIMER1_CTC_FLAG_BIT = 0;
            timer1_100ms_count++;                   /* 100 ms 단위 누적 */

            if (timer1_100ms_count >= ID_LAST_TWO) {   /* 69회 x 100ms = 6900ms */
                timer1_100ms_count = 0;
                {
                    U16 v = ADC_read_voltage();
                    UART0_send_voltage(v);
                }
            }
        }

        PORTC &= ~PERIOD_PIN_MASK;         /* 1-Period END : PC0 = LOW */
    }
}

/****************************************************************************************************************************
 *                                       Initialization Functions
 ****************************************************************************************************************************/

/*--- JTAG 비활성화 ---
 * MCUCSR의 JTD bit(bit 7)를 1로 set -> JTAG Interface OFF
 * 안전장치 : 4 cycle 이내에 두 번 연속 set 해야만 하드웨어에 적용됨
 * 이유    : KUT-128 보드에서 PF6(ADC6)와 PF7(ADC7)이 JTAG pin(TDO/TDI)과 공유됨 */
void JTAG_disable(void)
{
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);
}

/*--- 포트 초기화 ---*/
void Port_init(void)
{
    /* PORTC : 8bit 출력 (PC0 = 1-Period 측정용, PC1~PC7 = Debug LED) */
    DDRC  = 0xFF;
    PORTC = 0x00;                               /* 초기값 LOW */

    /* PORTB : 상위 4bit 출력 (7-seg 세그먼트 e,f,g + dp) */
    DDRB  = 0xF0;
    PORTB = 0x00;

    /* PORTD : 상위 4bit 출력 (7-seg 세그먼트 a,b,c,d) */
    DDRD  = 0xF0;
    PORTD = 0x00;

    /* PORTG : 하위 4bit 출력 (7-seg digit 선택 1~4) */
    DDRG  = 0x0F;
    PORTG = 0x00;

    /* PORTE : PE3 = OC3A (PWM 출력) */
    DDRE  = (1 << 3);

    /* PORTF : 전체 입력 (ADC 채널) */
    DDRF  = 0x00;
}

/*--- External Interrupt 초기화 ---
 *  EICRB (Interrupt Control B) : INT7 ~ INT4 edge 설정
 *    INT7 : ISC71 ISC70 = 1 0 -> Falling edge
 *    INT6 : ISC61 ISC60 = 1 1 -> Rising  edge
 *    INT5 : ISC51 ISC50 = 1 0 -> Falling edge
 *    INT4 : ISC41 ISC40 = 1 1 -> Rising  edge
 *    -> 0b 10 11 10 11 = 0xBB
 *  EIMSK : INT4 ~ INT7 enable -> 0b 1111 0000 */
void EINT_init(void)
{
    EICRB = 0b10111011;
    EIMSK = 0b11110000;
}

/*--- UART0 초기화 ---
 *  Baud rate : 9600 bps
 *  Frame     : 8 data bit, No Parity, 1 stop bit (8N1)
 *  UBRR calc : 16,000,000 / (16 x 9600) - 1 = 103
 *  Interrupt : TX + RX Complete Interrupt 모두 활성화 */
void UART0_init(void)
{
    UCSR0A = 0x00;
    UCSR0B = (1 << TXEN0)                       /* TX enable */
           | (1 << RXEN0)                       /* RX enable */
           | (1 << RXCIE0);                     /* RX Complete Interrupt enable */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     /* Async, No parity, 1 stop, 8 data bit */
    UBRR0H = 0x00;
    UBRR0L = 103;
}

/****************************************************************************************************************************
 *                                         7-Segment Display Functions
 ****************************************************************************************************************************/

/*--- 초기 학번 표시 (40회 반복, 약 320 ms 동안 잔상으로 4자리 표시) ---*/
void Seg4_out(U8 N1, U8 N10, U8 N100, U8 N1000)
{
    U8 i;
    for (i = 0; i < 40; i++) {
        PORTG = 0b00001000;
        PORTD = ((seg_pat[N1] & 0x0F) << 4) | (PORTD & 0x0F);
        PORTB = (seg_pat[N1] & 0x70) | (PORTB & 0x0F);
        delay_ms(2);

        PORTG = 0b00000100;
        PORTD = ((seg_pat[N10] & 0x0F) << 4) | (PORTD & 0x0F);
        PORTB = (seg_pat[N10] & 0x70) | (PORTB & 0x0F);
        delay_ms(2);

        PORTG = 0b00000010;
        PORTD = ((seg_pat[N100] & 0x0F) << 4) | (PORTD & 0x0F);
        PORTB = (seg_pat[N100] & 0x70) | (PORTB & 0x0F);
        delay_ms(2);

        PORTG = 0b00000001;
        PORTD = ((seg_pat[N1000] & 0x0F) << 4) | (PORTD & 0x0F);
        PORTB = (seg_pat[N1000] & 0x70) | (PORTB & 0x0F);
        delay_ms(2);
    }
}

/*--- while(1) 내에서 호출용 : 1회 스캔만 (잔상은 메인 루프 반복으로 형성) ---*/
void Seg4_out2(U8 N1, U8 N10, U8 N100, U8 N1000)
{
    PORTG = 0b00001000;
    PORTD = ((seg_pat[N1] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[N1] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    PORTG = 0b00000100;
    PORTD = ((seg_pat[N10] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[N10] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    PORTG = 0b00000010;
    PORTD = ((seg_pat[N100] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[N100] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    PORTG = 0b00000001;
    PORTD = ((seg_pat[N1000] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[N1000] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);
}

/*--- 온도 표시 (XX.X 형태, 정수부 두 자리 + 소수점 + 소수 한 자리)
 *    temp_x10 = 312 -> 31.2 표시
 *    자리 배치 : digit1 = 10의자리, digit2 = 1의자리(+dp), digit3 = 소수 첫째, digit4 = 빈칸 */
void Seg_temp_disp(U16 temp_x10)
{
    U8 d_tens = (U8)((temp_x10 / 100U) % 10U);    /* 10의 자리 */
    U8 d_ones = (U8)((temp_x10 / 10U)  % 10U);    /* 1의 자리  */
    U8 d_dec  = (U8)( temp_x10 % 10U);            /* 소수 첫째 자리 */

    /* digit1 (PG0) : 10의 자리 */
    PORTG = 0b00000001;
    PORTD = ((seg_pat[d_tens] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_tens] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    /* digit2 (PG1) : 1의 자리 + 소수점 ON */
    PORTG = 0b00000010;
    PORTD = ((seg_pat[d_ones] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_ones] & 0x70) | (PORTB & 0x0F);
    PORTB = PORTB | 0x80;                         /* dp ON */
    delay_ms(1);

    /* digit3 (PG2) : 소수 첫째 자리 */
    PORTG = 0b00000100;
    PORTD = ((seg_pat[d_dec] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_dec] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    /* digit4 (PG3) : 빈칸 */
    PORTG = 0b00001000;
    PORTD = (0x00) | (PORTD & 0x0F);
    PORTB = (0x00) | (PORTB & 0x0F);
    delay_ms(1);
}

/****************************************************************************************************************************
 *                                              Timer Functions
 ****************************************************************************************************************************/

/*--- Timer2 (8bit Overflow) ---
 *  Prescaler 64 -> 1 tick = 64 / 16MHz = 4 us
 *  Overflow 주기 = (256 - TCNT2) x 4 us
 *  TCNT2 = 6  ->  (256 - 6) x 4 us = 250 x 4 us = 1 ms
 *  -> 1ms 기본 주기 생성 -> main loop에서 4500회 누적 -> 4500ms */
void Timer2_start(void)
{
    TCNT2  = 6;
    TCCR2  = 0x04;                              /* CS22 CS21 CS20 = 100 -> Prescaler 64 */
    TIMSK |= (1 << TOIE2);                      /* Timer2 Overflow Interrupt Enable */
}

void Timer2_stop(void)
{
    TCCR2  = 0x00;                              /* Clock OFF */
    TIMSK &= ~(1 << TOIE2);                     /* OVF Interrupt Disable */
    TIMER2_OVF_FLAG_BIT = 0;
    timer2_ms_count = 0;
}

/*--- Timer1 (16bit CTC, Compare Match A) ---
 *  Prescaler 256 -> 1 tick = 256 / 16MHz = 16 us
 *  OCR1A = 6249 -> (6249+1) x 16 us = 100 ms 주기
 *  -> 100ms 기본 주기 생성 -> main loop에서 69회 누적 -> 6900ms (학번 뒤 2자리 x 100ms) */
void Timer1_start(void)
{
    TCCR1A = 0x00;                              /* WGM11=0, WGM10=0 */
    TCCR1B = (1 << WGM12) | (1 << CS12);        /* WGM12=1 (CTC with OCR1A), CS12=1 -> Prescaler 256 */
    OCR1AH = (6249 >> 8) & 0xFF;
    OCR1AL = 6249 & 0xFF;
    TCNT1H = 0;
    TCNT1L = 0;
    TIMSK |= (1 << OCIE1A);                     /* Compare Match A Interrupt Enable */
}

/*--- Timer3 9bit Fast PWM (Mode 6) ---
 *  WGM33 WGM32 WGM31 WGM30 = 0 1 1 0 -> Fast PWM 9bit, TOP = 0x1FF (511)
 *  COM3A1 COM3A0 = 1 0 -> Non-inverting (Compare match 시 OC3A clear)
 *  Prescaler 8 -> PWM 주파수 = 16MHz / (8 x 512) = 3.906 kHz
 *  Duty 69% -> OCR3A = 511 x 69 / 100 = 352 */
void Timer3_FastPWM_start(void)
{
    TCCR3A = (1 << COM3A1) | (1 << WGM31);          /* COM3A1=1, WGM31=1, WGM30=0 */
    TCCR3B = (1 << WGM32) | (1 << CS31);            /* WGM32=1, CS31=1 -> Prescaler 8 */
    OCR3AH = (PWM_OCR_VALUE >> 8) & 0xFF;
    OCR3AL = PWM_OCR_VALUE & 0xFF;
}

/*--- Timer3 9bit Phase Correct PWM (Mode 2) ---
 *  WGM33 WGM32 WGM31 WGM30 = 0 0 1 0 -> Phase Correct PWM 9bit, TOP = 0x1FF (511)
 *  PWM 주파수 = 16MHz / (8 x 2 x 511) = 1.957 kHz (Fast PWM의 약 절반)
 *  Duty 69% -> OCR3A = 352 (동일) */
void Timer3_PhaseCorrectPWM_start(void)
{
    TCCR3A = (1 << COM3A1) | (1 << WGM31);          /* COM3A1=1, WGM31=1, WGM30=0 */
    TCCR3B = (0 << WGM32) | (1 << CS31);            /* WGM32=0, CS31=1 -> Prescaler 8 */
    OCR3AH = (PWM_OCR_VALUE >> 8) & 0xFF;
    OCR3AL = PWM_OCR_VALUE & 0xFF;
}

void Timer3_stop(void)
{
    TCCR3A = 0x00;
    TCCR3B = 0x00;
}

/****************************************************************************************************************************
 *                                               ADC Functions
 *--------------------------------------------------------------------------------------------------------------------------
 *  ADC Clock       : 16MHz / 128 = 125 kHz -> 1 ADC clock = 8 us
 *  Conversion time : Normal 13 ADC clock = 104 us / First 25 ADC clock = 200 us
 *  Resolution      : 10bit (0 ~ 1023)
 *  기준 전압(AVCC) : 5.0 V
 ****************************************************************************************************************************/

/*--- ADC7 (PF7) : LM35DZ 온도 측정 (Free Running mode) ---
 *  LM35DZ 특성 : 10 mV / 도 (예 25도 -> 250 mV)
 *  온도 수식   : Temperature = (ADC x 5000 / 1024) / 10 = ADC x 500 / 1024 (단위 0.1도)
 *  정수 연산   : temp_x10 = (ADC x 500 + 512) / 1024     (+512는 반올림) */
U16 ADC_read_temp(void)
{
    U16 ad_val;
    U32 calc;

    ADMUX  = 0x07;                              /* REFS=00 (AREF), MUX=00111 -> Channel 7 */
    ADCSRA = 0xE7;                              /* ADEN=1, ADSC=1, ADFR=1, Prescaler 128 */

    delay_us(200);                              /* First conversion 안정화 */

    /* Free Running 중 최신 변환 결과 읽기 (ADCL 먼저 읽어야 함) */
    ad_val = (U16)ADCL;
    ad_val |= ((U16)ADCH << 8);

    calc = ((U32)ad_val * 500UL + 512UL) / 1024UL;
    return (U16)calc;
}

/*--- ADC6 (PF6) : VR2 가변저항 전압 측정 (Single Conversion mode) ---
 *  전압 수식   : Voltage[V] = ADC x 5.0 / 1024
 *  정수 연산   : v_x100 = (ADC x 500 + 512) / 1024 (0.01 V 단위, 예 3.27V -> 327) */
U16 ADC_read_voltage(void)
{
    U16 ad_val;
    U32 calc;

    ADMUX  = 0x06;                              /* REFS=00, MUX=00110 -> Channel 6 */
    ADCSRA = 0x87;                              /* ADEN=1, Free Running OFF, Prescaler 128 */
    delay_us(200);

    ADCSRA |= (1 << ADSC);                      /* 변환 시작 */
    while ((ADCSRA & (1 << ADIF)) == 0) {       /* ADIF=1(변환 완료)까지 대기 */
        ;
    }
    ADCSRA |= (1 << ADIF);                      /* ADIF clear (1 write) */

    ad_val = (U16)ADCL;
    ad_val |= ((U16)ADCH << 8);

    calc = ((U32)ad_val * 500UL + 512UL) / 1024UL;
    return (U16)calc;
}

/****************************************************************************************************************************
 *                                          UART0 TX Functions (stdio.h 미사용)
 ****************************************************************************************************************************/

/*--- 1문자 송신 (UDRE0 = 1 될 때까지 대기 후 UDR0에 쓰기) ---*/
void UART0_transmit(char data)
{
    while ((UCSR0A & (1 << UDRE0)) == 0) {      /* 송신 버퍼 비었을 때까지 대기 */
        ;
    }
    UDR0 = (U8)data;
}

/*--- 문자열 송신 (null terminator 까지) ---*/
void UART0_print(char *str)
{
    while (*str != 0) {
        UART0_transmit(*str);
        str++;
    }
}

/*--- 10진수 정수 송신 (최대 5자리, 0 ~ 65535) ---
 *  재귀 방식 : 상위 자리부터 출력
 *  예 : 327 -> '3','2','7'  /  5 -> '5'  /  12345 -> '1','2','3','4','5' */
void UART0_put_dec(U16 value)
{
    if (value >= 10U) {
        UART0_put_dec(value / 10U);             /* 상위 자리 먼저 출력 */
    }
    UART0_transmit((char)('0' + (value % 10U)));
}

/*--- 전압 송신 : "Voltage: X.XX V\r\n" 형식 ---
 *  v_x100 = 327 -> "Voltage: 3.27 V\r\n"
 *  정수부가 여러 자리여도 안전 (UART0_put_dec가 자동 처리) */
void UART0_send_voltage(U16 v_x100)
{
    U16 vi = v_x100 / 100U;                     /* 정수부 */
    U16 vf = v_x100 % 100U;                     /* 소수부 (2자리) */

    UART0_print("Voltage: ");
    UART0_put_dec(vi);                          /* 정수부 출력 (다자릿수 안전) */
    UART0_transmit('.');

    /* 소수부 앞자리 0 채우기 (예 5 -> "05") */
    if (vf < 10U) {
        UART0_transmit('0');
    }
    UART0_put_dec(vf);
    UART0_print(" V\r\n");
}

/****************************************************************************************************************************
 *                                        Interrupt Service Routines
 *--------------------------------------------------------------------------------------------------------------------------
 *  [설계 원칙]
 *   - 모든 ISR은 SREG 저장 -> flag set -> SREG 복원 구조로 Nesting 차단
 *   - ISR 내부에서 긴 작업(ADC, UART, 함수 호출) 금지 -> main loop에서 처리
 *   - 실행 시간 : 수 us 이내
 ****************************************************************************************************************************/

interrupt [EXT_INT4] void external_int4(void) {
    SREG &= 0x7F;                               /* I-flag OFF (Nesting 차단) */
    EINT4_FLAG_BIT = 1;                         /* main loop에서 처리할 flag set */
    SREG |= 0x80;                               /* I-flag ON 복원 */
}

interrupt [EXT_INT5] void external_int5(void) {
    SREG &= 0x7F;
    EINT5_FLAG_BIT = 1;
    SREG |= 0x80;
}

interrupt [EXT_INT6] void external_int6(void) {
    SREG &= 0x7F;
    EINT6_FLAG_BIT = 1;
    SREG |= 0x80;
}

interrupt [EXT_INT7] void external_int7(void) {
    SREG &= 0x7F;
    EINT7_FLAG_BIT = 1;
    SREG |= 0x80;
}

/*--- Timer2 Overflow ISR ---
 *  1 ms 주기 유지를 위해 TCNT2 재로드 (256 - 250 = 6) 필수 */
interrupt [TIM2_OVF] void timer2_ovf_isr(void) {
    SREG &= 0x7F;
    TCNT2 = 6;                                  /* 1ms 주기 재로드 */
    TIMER2_OVF_FLAG_BIT = 1;
    SREG |= 0x80;
}

/*--- Timer1 Compare Match A ISR ---
 *  CTC mode는 Compare Match 시 TCNT1이 자동 0으로 reset됨 */
interrupt [TIM1_COMPA] void timer1_compa_isr(void) {
    SREG &= 0x7F;
    TIMER1_CTC_FLAG_BIT = 1;
    SREG |= 0x80;
}

/*--- UART0 RX Complete ISR ---
 *  PC로부터 문자 1개 수신 완료 시 발생
 *  UDR0 읽기는 ISR 내에서 수행 (하드웨어 FIFO overflow 방지) */
interrupt [USART0_RXC] void uart0_rx_isr(void) {
    SREG &= 0x7F;
    UART_RX_CHAR     = (char)UDR0;              /* 수신 문자 저장 */
    UART_RX_FLAG_BIT = 1;
    SREG |= 0x80;
}
