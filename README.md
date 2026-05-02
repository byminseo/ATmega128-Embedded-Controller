# ATmega128-Embedded-Controller

8bit AVR(ATmega128) 기반 통합 임베디드 제어 시스템

---

## 프로젝트 개요

ATmega128 마이크로컨트롤러를 활용하여 외부 인터럽트, 타이머, PWM, ADC, UART, I2C 통신을 통합한 실시간 제어 시스템입니다.
MISRA C 가이드라인을 참조하여 작성되었으며, ISR 최소화 설계 원칙을 적용하였습니다.

---

## 개발 환경

| 항목 | 내용 |
|---|---|
| MCU | ATmega128 (8bit AVR, 16MHz) |
| Board | KUT-128 Combo Board (OHM/RAC) |
| Tool | CodeVisionAVR (CVAVR) |
| 코딩 표준 | MISRA C 가이드라인 참조 |

---

## 구현 기능

### Project2_main.c — 통합 제어 시스템

| 기능 | 내용 |
|---|---|
| External Interrupt | 4채널 (INT4~INT7), Rising/Falling edge 구분 |
| Timer1 | 16bit CTC mode, 100ms 주기 → 69회 누적 → 6900ms |
| Timer2 | 8bit OVF mode, 1ms 주기 → 4500회 누적 → 4500ms |
| Timer3 | 9bit Fast PWM / Phase Correct PWM, Duty 69% |
| ADC | 10bit, LM35DZ 온도센서(ADC7) + 가변저항 전압(ADC6) |
| UART0 | 9600bps, 8N1, TX/RX 인터럽트, PC 원격 명령 제어 |
| 7-Segment | 4자리 다이나믹 스캔, 학번 표시 / 온도 표시 전환 |

**UART 원격 명령어**
- `T` : 온도 측정 시작
- `V` : 전압 측정 및 UART 전송
- `F` : Fast PWM 시작
- `P` : Phase Correct PWM 전환
- `S` : 현재 동작 상태 확인

### I2C_EEPROM.c — I2C 통신 EEPROM 제어

| 기능 | 내용 |
|---|---|
| 통신 방식 | I2C Bit-Bang (TWI 하드웨어 미사용) |
| 대상 소자 | M24C08 EEPROM (1KB) |
| Byte Write | 1바이트 지정 주소 기록 |
| Page Write | 연속 주소 다중 바이트 기록 |
| Random Read | 지정 주소 1바이트 읽기 |
| Sequential Read | 연속 주소 다중 바이트 읽기 |

---

## 설계 원칙

- **ISR 최소화**: ISR 내부에서는 플래그 set만 수행, 실제 처리는 main loop에서 담당
- **MISRA C 참조**: `<stdio.h>` 미사용, `typedef` 활용, `volatile` 적용, 주석 스타일 통일
- **1-Period 측정**: PC0 핀 HIGH/LOW 토글로 오실로스코프 실행 시간 측정 가능
  - Best case (플래그 없음): 약 4ms
  - Worst case (모든 플래그 + ADC + UART): 약 10~15ms
- **JTAG 비활성화**: PF6/PF7 ADC 사용을 위해 소프트웨어로 JTAG 비활성화

---

## 파일 구조

```
ATmega128-Embedded-Controller/
├── src/
│   ├── Project2_main.c     # 통합 제어 시스템 (Timer, PWM, ADC, UART, Interrupt)
│   └── I2C_EEPROM.c        # I2C Bit-Bang EEPROM 제어
└── README.md
```
