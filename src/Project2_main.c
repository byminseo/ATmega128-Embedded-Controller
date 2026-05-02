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

#define ID_LAST_TWO        69
#define ADC_TEMP_PERIOD_MS 4500
#define ADC_TEMP_MAX_CNT   10
#define PWM_DUTY_PERCENT   ID_LAST_TWO
#define PWM_TOP_9BIT       511UL
#define PWM_OCR_VALUE      ((PWM_TOP_9BIT * PWM_DUTY_PERCENT) / 100UL)

/*============================ 1-Period 측정용 핀 ============================*/
/* PC0 pin : while 루프 1주기 HIGH/LOW 토글용
   오실로스코프 측정 시 HIGH 구간 폭 = 실행 시간 (Best/Worst case 관찰 가능) */
#define PERIOD_PIN_MASK    0x01

/*============================ 전역 플래그 / 변수 ============================*/
volatile U8  EINT4_FLAG_BIT = 0;
volatile U8  EINT5_FLAG_BIT = 0;
volatile U8  EINT6_FLAG_BIT = 0;
volatile U8  EINT7_FLAG_BIT = 0;

volatile U8  TIMER2_OVF_FLAG_BIT = 0;
volatile U8  TIMER1_CTC_FLAG_BIT = 0;

volatile U8   UART_RX_FLAG_BIT = 0;
volatile char UART_RX_CHAR     = 0;

volatile U16 timer2_ms_count    = 0;
volatile U8  timer1_100ms_count = 0;

volatile U8  display_mode     = 0;
volatile U8  temp_disp_count  = 0;
volatile U16 current_temp_x10 = 0;

/*============================ 함수 프로토타입 ============================*/
void Port_init(void);
void EINT_init(void);
void UART0_init(void);
void JTAG_disable(void);

void Seg4_out (U8 N1, U8 N10, U8 N100, U8 N1000);
void Seg4_out2(U8 N1, U8 N10, U8 N100, U8 N1000);
void Seg_temp_disp(U16 temp_x10);

void Timer2_start(void);
void Timer2_stop (void);
void Timer1_start(void);
void Timer3_FastPWM_start(void);
void Timer3_PhaseCorrectPWM_start(void);
void Timer3_stop(void);

U16  ADC_read_temp   (void);
U16  ADC_read_voltage(void);

void UART0_transmit(char data);
void UART0_print   (char *str);
void UART0_put_dec (U16 value);
void UART0_send_voltage(U16 v_x100);

/****************************************************************************************************************************
 *                                                main Function
 ****************************************************************************************************************************/
void main(void)
{
    JTAG_disable();

    Port_init();
    EINT_init();
    UART0_init();

    Seg4_out(ID_FRONT_ONE, ID_FRONT_TEN, ID_FRONT_HUNDRED, ID_FRONT_THOUSAND);
    Seg4_out(ID_BACK_ONE,  ID_BACK_TEN,  ID_BACK_HUNDRED,  ID_BACK_THOUSAND);

    UART0_print("\r\n=== Project2 Start (ID: 20183369) ===\r\n");
    UART0_print("Commands : T=Temp, V=Voltage, F=FastPWM, P=PhasePWM, S=Status\r\n");

    SREG |= 0x80;

    PORTC &= ~PERIOD_PIN_MASK;

    while(1)
    {
        PORTC |= PERIOD_PIN_MASK;

        if (display_mode == 0) {
            Seg4_out2(ID_BACK_ONE, ID_BACK_TEN, ID_BACK_HUNDRED, ID_BACK_THOUSAND);
        } else {
            Seg_temp_disp(current_temp_x10);
        }

        if (UART_RX_FLAG_BIT == 1) {
            UART_RX_FLAG_BIT = 0;
            switch (UART_RX_CHAR) {
                case 'T' : case 't' :
                    EINT4_FLAG_BIT = 1;
                    UART0_print("[CMD] Temp measurement start\r\n");
                    break;
                case 'V' : case 'v' :
                    EINT5_FLAG_BIT = 1;
                    UART0_print("[CMD] Voltage measurement start\r\n");
                    break;
                case 'F' : case 'f' :
                    EINT6_FLAG_BIT = 1;
                    UART0_print("[CMD] Fast PWM start\r\n");
                    break;
                case 'P' : case 'p' :
                    EINT7_FLAG_BIT = 1;
                    UART0_print("[CMD] Phase Correct PWM start\r\n");
                    break;
                case 'S' : case 's' :
                    UART0_print("[STATUS] Mode: ");
                    UART0_print((display_mode == 0) ? "ID\r\n" : "Temp\r\n");
                    break;
                default :
                    break;
            }
        }

        if (EINT4_FLAG_BIT == 1) {
            EINT4_FLAG_BIT = 0;
            display_mode    = 1;
            temp_disp_count = 0;
            timer2_ms_count = 0;
            Timer2_start();
        }

        if (EINT5_FLAG_BIT == 1) {
            EINT5_FLAG_BIT = 0;
            timer1_100ms_count = 0;
            Timer1_start();
        }

        if (EINT6_FLAG_BIT == 1) {
            EINT6_FLAG_BIT = 0;
            Timer3_FastPWM_start();
        }

        if (EINT7_FLAG_BIT == 1) {
            EINT7_FLAG_BIT = 0;
            Timer3_stop();
            Timer3_PhaseCorrectPWM_start();
        }

        if (TIMER2_OVF_FLAG_BIT == 1) {
            TIMER2_OVF_FLAG_BIT = 0;
            timer2_ms_count++;

            if (timer2_ms_count >= ADC_TEMP_PERIOD_MS) {
                timer2_ms_count = 0;
                current_temp_x10 = ADC_read_temp();
                temp_disp_count++;

                if (temp_disp_count >= ADC_TEMP_MAX_CNT) {
                    Timer2_stop();
                    display_mode    = 0;
                    temp_disp_count = 0;
                }
            }
        }

        if (TIMER1_CTC_FLAG_BIT == 1) {
            TIMER1_CTC_FLAG_BIT = 0;
            timer1_100ms_count++;

            if (timer1_100ms_count >= ID_LAST_TWO) {
                timer1_100ms_count = 0;
                {
                    U16 v = ADC_read_voltage();
                    UART0_send_voltage(v);
                }
            }
        }

        PORTC &= ~PERIOD_PIN_MASK;
    }
}

/****************************************************************************************************************************
 *                                       Initialization Functions
 ****************************************************************************************************************************/

void JTAG_disable(void)
{
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);
}

void Port_init(void)
{
    DDRC  = 0xFF;
    PORTC = 0x00;

    DDRB  = 0xF0;
    PORTB = 0x00;

    DDRD  = 0xF0;
    PORTD = 0x00;

    DDRG  = 0x0F;
    PORTG = 0x00;

    DDRE  = (1 << 3);

    DDRF  = 0x00;
}

void EINT_init(void)
{
    EICRB = 0b10111011;
    EIMSK = 0b11110000;
}

void UART0_init(void)
{
    UCSR0A = 0x00;
    UCSR0B = (1 << TXEN0)
           | (1 << RXEN0)
           | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UBRR0H = 0x00;
    UBRR0L = 103;
}

/****************************************************************************************************************************
 *                                         7-Segment Display Functions
 ****************************************************************************************************************************/

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

void Seg_temp_disp(U16 temp_x10)
{
    U8 d_tens = (U8)((temp_x10 / 100U) % 10U);
    U8 d_ones = (U8)((temp_x10 / 10U)  % 10U);
    U8 d_dec  = (U8)( temp_x10 % 10U);

    PORTG = 0b00000001;
    PORTD = ((seg_pat[d_tens] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_tens] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    PORTG = 0b00000010;
    PORTD = ((seg_pat[d_ones] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_ones] & 0x70) | (PORTB & 0x0F);
    PORTB = PORTB | 0x80;
    delay_ms(1);

    PORTG = 0b00000100;
    PORTD = ((seg_pat[d_dec] & 0x0F) << 4) | (PORTD & 0x0F);
    PORTB = (seg_pat[d_dec] & 0x70) | (PORTB & 0x0F);
    delay_ms(1);

    PORTG = 0b00001000;
    PORTD = (0x00) | (PORTD & 0x0F);
    PORTB = (0x00) | (PORTB & 0x0F);
    delay_ms(1);
}

/****************************************************************************************************************************
 *                                              Timer Functions
 ****************************************************************************************************************************/

void Timer2_start(void)
{
    TCNT2  = 6;
    TCCR2  = 0x04;
    TIMSK |= (1 << TOIE2);
}

void Timer2_stop(void)
{
    TCCR2  = 0x00;
    TIMSK &= ~(1 << TOIE2);
    TIMER2_OVF_FLAG_BIT = 0;
    timer2_ms_count = 0;
}

void Timer1_start(void)
{
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1AH = (6249 >> 8) & 0xFF;
    OCR1AL = 6249 & 0xFF;
    TCNT1H = 0;
    TCNT1L = 0;
    TIMSK |= (1 << OCIE1A);
}

void Timer3_FastPWM_start(void)
{
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM32) | (1 << CS31);
    OCR3AH = (PWM_OCR_VALUE >> 8) & 0xFF;
    OCR3AL = PWM_OCR_VALUE & 0xFF;
}

void Timer3_PhaseCorrectPWM_start(void)
{
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (0 << WGM32) | (1 << CS31);
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
 ****************************************************************************************************************************/

U16 ADC_read_temp(void)
{
    U16 ad_val;
    U32 calc;

    ADMUX  = 0x07;
    ADCSRA = 0xE7;

    delay_us(200);

    ad_val = (U16)ADCL;
    ad_val |= ((U16)ADCH << 8);

    calc = ((U32)ad_val * 500UL + 512UL) / 1024UL;
    return (U16)calc;
}

U16 ADC_read_voltage(void)
{
    U16 ad_val;
    U32 calc;

    ADMUX  = 0x06;
    ADCSRA = 0x87;
    delay_us(200);

    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADIF)) == 0) {
        ;
    }
    ADCSRA |= (1 << ADIF);

    ad_val = (U16)ADCL;
    ad_val |= ((U16)ADCH << 8);

    calc = ((U32)ad_val * 500UL + 512UL) / 1024UL;
    return (U16)calc;
}

/****************************************************************************************************************************
 *                                          UART0 TX Functions
 ****************************************************************************************************************************/

void UART0_transmit(char data)
{
    while ((UCSR0A & (1 << UDRE0)) == 0) {
        ;
    }
    UDR0 = (U8)data;
}

void UART0_print(char *str)
{
    while (*str != 0) {
        UART0_transmit(*str);
        str++;
    }
}

void UART0_put_dec(U16 value)
{
    if (value >= 10U) {
        UART0_put_dec(value / 10U);
    }
    UART0_transmit((char)('0' + (value % 10U)));
}

void UART0_send_voltage(U16 v_x100)
{
    U16 vi = v_x100 / 100U;
    U16 vf = v_x100 % 100U;

    UART0_print("Voltage: ");
    UART0_put_dec(vi);
    UART0_transmit('.');

    if (vf < 10U) {
        UART0_transmit('0');
    }
    UART0_put_dec(vf);
    UART0_print(" V\r\n");
}

/****************************************************************************************************************************
 *                                        Interrupt Service Routines
 ****************************************************************************************************************************/

interrupt [EXT_INT4] void external_int4(void) {
    SREG &= 0x7F;
    EINT4_FLAG_BIT = 1;
    SREG |= 0x80;
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

interrupt [TIM2_OVF] void timer2_ovf_isr(void) {
    SREG &= 0x7F;
    TCNT2 = 6;
    TIMER2_OVF_FLAG_BIT = 1;
    SREG |= 0x80;
}

interrupt [TIM1_COMPA] void timer1_compa_isr(void) {
    SREG &= 0x7F;
    TIMER1_CTC_FLAG_BIT = 1;
    SREG |= 0x80;
}

interrupt [USART0_RXC] void uart0_rx_isr(void) {
    SREG &= 0x7F;
    UART_RX_CHAR     = (char)UDR0;
    UART_RX_FLAG_BIT = 1;
    SREG |= 0x80;
}
