#include "hal_data.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // 가변인자 함수
#include <ctype.h> // isdigit()

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER


/* 전역변수■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
// [volatile] https://luna-archive.tistory.com/2
// volatile 변수 사용 이유 : 컴파일러 최적화 하지 않음
// 멀티스레드 환경이나 인터럽트 서비스 루틴, 하드웨어와 직접 상호작용할 때
// 변수 값이 예측할 수 없이 변경될 수 있는 상황에서 사용됨

// (1) 같은 메모리 주소에 반복 write 할 때,
// 컴파일러가 최적화 수행하면, 마지막 write 만 실행
// mcu 의 경우, 모든 write 수행 해야 함
// (메모리 주소에 연결된 하드웨어 레지스터에 값을 쓰는 프로그램의 경우 이므로)
// 각각의 쓰기가 하드웨어에 특정 명령을 전달하는 것이므로,
// 주소가 같다는 이유만으로 중복되는 쓰기 명령을 없애 버리면, 하드웨어가 오동작

// volatile 키워드는 크게 3가지 경우에 흔히 사용된다.
//(1) MIMO(Memory-mapped I/O)
//(2) 인터럽트 서비스 루틴(Interrupt Service Routine)의 사용
//(3) 멀티 쓰레드 환경

// FSP 라이브러리의 함수 반환 타입 (성공: FSP_SUCCESS)
fsp_err_t err;

/*** UART (Serial 통신) ***/
#define UART_TX_BUF_SIZE 50
#define UART_RX_BUF_SIZE 30
char g_tx_buffer[UART_TX_BUF_SIZE];
volatile uint8_t g_uart_index = 0;  // 버퍼에 데이터가 쌓이는 위치

#define END_CHARACTER   '\r'        // 명령어 종료를 나타내는 문자
#define HEADER          "HDR"       // 헤더: 패킷 시작
#define TAIL            "TAIL"      // 테일: 패킷 끝
volatile uint8_t g_rx_buffer[UART_TX_BUF_SIZE] = {0};
volatile uint16_t g_rx_index = 0;

volatile _Bool g_uart_tx_complete = false;  // 비동기 전송 플래그 (초기값: true)
volatile _Bool g_uart_rx_complete = false;  // 비동기 전송 플래그 (초기값: true)
volatile _Bool g_scan_complete = false;     // ADC SCAN 완료 플래그

volatile _Bool g_uart_tick_output_flag = false;
volatile int g_new_minutes = 0;
volatile int g_new_seconds = 0;
#define NO_VAR 65535 // UART 변수 출력 여부 (uint16_t 의 최댓값:65535 이면 변수 출력 X)

/*** ADC (Analog to Digital Converter ***/
uint16_t g_adc_data; // ADC 조도센서 데이터
#define ADC_BUFFER_SIZE 60
#define ADC_THRESHOLD_HIGH 3000
#define ADC_THRESHOLD_LOW 1000

typedef struct {
    uint16_t buffer[ADC_BUFFER_SIZE];
    int head;       // 가장 오래된 데이터의 인덱스
    int tail;       // 다음 데이터가 저장될 인덱스
    int count;      // 현재 버퍼에 저장된 데이터 수
    uint32_t sum;   // 버퍼의 데이터 합계
} ring_buf_t;
ring_buf_t g_adc_buffer;

/*** USER BUTTON ***/
bsp_io_level_t g_color_btn_level; // 색깔 변경 버튼 상태 (HIGH or LOW)
bsp_io_level_t g_brightness_btn_level; // 밝기 변경 버튼 상태 (HIGH or LOW)
typedef enum {
    WAITING_FOR_PRESS,  // Click 을 기다림 : 버튼을 누른 상태       (LOW)
    WAITING_FOR_RELEASE // 손 떼기를 기다림  : 버튼을 누르지 않은 상태 (HIGH)
} ButtonState;
ButtonState g_color_btn_state = WAITING_FOR_PRESS; // 누르지 않은 상태
ButtonState g_brightness_btn_state = WAITING_FOR_PRESS; // 누르지 않은 상태

int g_color_btn_cnt = 0; // 색상 변경 버튼 클릭 횟수
int g_brightness_btn_cnt = 0;   // 밝기 변경 버튼 클릭 횟수

/*** RGB LED : GPT ***/
uint32_t g_R_LED_duty_cycle = 0;
uint32_t g_G_LED_duty_cycle = 0;
uint32_t g_B_LED_duty_cycle = 0;
_Bool g_is_RGB_LED_ON_by_cmd = false;
#define GAMMA 2.2 // 감마 보정 의 일반적인 감마 값

// [데이터 시트] RGB LED 0V ~ 5V
// http://wiki.sunfounder.cc/index.php?title=RGB_LED_Module
//#define MAX_VOLTAGE 5.0

#define RGB_PWM_PERIOD 1000 // 단위: micro seconds (FSP Configuration)
volatile _Bool g_timer_set = false;
volatile uint64_t g_timer_target_ticks = 0;
volatile uint64_t g_new_tick = 0; // 타이머 설정 틱
volatile uint64_t g_old_tick = 0; // 프로그램 실행부터 지금까지 전체 틱
#define TICK_PER_ONE_SEC 100000
#define TICK_PER_ONE_MIN TICK_PER_ONE_SEC * 60
/*** hal_entry() delay 값 ***/
#define HAL_ENTRY_DELAY 100 // hal_entry() 내부 while 문의 delay
#define SEC_UNIT 1000 / HAL_ENTRY_DELAY // 반복문 내에서 1초 단위로 맞춰주기 위함

/*** 버튼/명령어 수동 제어 여부 ***/
volatile _Bool g_manual_control = false;
/* 소자들■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
/*** Light Sensor : 조도 센서 ***/
#define LIGHT_SENSOR_PIN BSP_IO_PORT_00_PIN_00

/*** RGB LED ***/
#define RGB_LED_R_PIN BSP_IO_PORT_01_PIN_11
#define RGB_LED_G_PIN BSP_IO_PORT_02_PIN_05
#define RGB_LED_B_PIN BSP_IO_PORT_04_PIN_07


/* 함수들■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
void delay_us(uint32_t delay);
void delay_ms(uint32_t ms);
_Bool is_RGB_LED_ON();
void RGB_LED_OFF();
void RGB_LED_ON();
void RGB_HALF_ON();
void Device_Init();
void uart_init();
void adc_init();
void adc_callback(adc_callback_args_t * p_args);
void ring_buf_init(ring_buf_t *rb);
void ring_buf_push(ring_buf_t *rb, uint16_t data);
uint16_t ring_buf_avg(ring_buf_t *rb);
int adc_read();
void gpt_open();
void set_period(timer_ctrl_t * const p_ctrl, uint32_t const period_counts);
void set_duty_cycle(timer_ctrl_t * const p_ctrl, uint32_t const duty_cycle);
void start_gpt(timer_ctrl_t * const p_ctrl);
void pwm_init();
void uart_callback(uart_callback_args_t *p_args);
fsp_err_t uart_ep_demo(void); // 주의
void uart_write(char *message, uint16_t var);
void uart_read();
void parse_command(char* data);
void set_brightness(int level);
void auto_on_off();
uint32_t gamma_correct_duty_cycle(uint32_t duty_cycle);
void set_duty_cycles_by_ratio(int n);
void handle_btn_click(uint16_t btn_num);
void write_duty_cycle();
void check_btn_clicked(bsp_io_port_pin_t BUTTON, bsp_io_level_t g_btn_level, ButtonState* g_btn_state);
uint32_t convert_brightness_to_duty_cycle(uint32_t brightness);
void process_command();
void command_err_handle();
void g_timer_callback(timer_callback_args_t *p_args);
void set_timer(uint32_t minutes, _Bool led_on);

// ■ Delay function in microseconds
void delay_us(uint32_t delay) {
    R_BSP_SoftwareDelay(delay, BSP_DELAY_UNITS_MICROSECONDS);
}

// ■ Delay function in milliseconds
void delay_ms(uint32_t ms) {
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}


// ■ UART 송수신 콜백
void uart_callback(uart_callback_args_t *p_args){
    switch(p_args->event){
        // 데이터 송신 (Transmit)
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart_tx_complete = true; // 송신 완료
            break;
        }

        // 데이터 수신 (Receive)
        // 한 문자 수신 event
        case UART_EVENT_RX_CHAR:
        {
            g_rx_buffer[g_rx_index++] = (uint8_t)p_args->data;
            if ((uint8_t)p_args->data == END_CHARACTER || g_rx_index >= UART_RX_BUF_SIZE) {
                g_rx_buffer[g_rx_index] = '\0'; // 종료 문자 추가
                g_rx_index = 0;  // 인덱스 초기화
                g_uart_rx_complete = true; // 수신 완료 플래그
            }
            break;
        }

        // 아래 event는 버퍼가 모두 찼을 때만 호출된다.
        case UART_EVENT_RX_COMPLETE:
            g_uart_rx_complete = true;
            break;

        // 이후 사용 안 할 이벤트 (컴파일 경고-e2studio 노란 줄-때문에 추가함)
        case UART_EVENT_ERR_PARITY:
            break;
        case UART_EVENT_ERR_FRAMING:
            break;
        case UART_EVENT_ERR_OVERFLOW:
            break;
        case UART_EVENT_BREAK_DETECT:
            break;
        case UART_EVENT_TX_DATA_EMPTY:
            break;
    }
}


// ■ UART 송신 (MCU > PC, Transmit)
void uart_write(char *message, uint16_t var)
{
    if(var == NO_VAR) sprintf(g_tx_buffer, "%s\r\n\033[0m", message);
    else snprintf(g_tx_buffer, UART_TX_BUF_SIZE, "%s: %u\r\n\033[0m", message, var);


    g_uart_tx_complete = false; // 플래그 초기화
    R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t *)g_tx_buffer, strlen(g_tx_buffer));
    while (!g_uart_tx_complete) {} // 전송 완료될 때까지 대기
}



// ■ UART 초기화
void uart_init() {
    R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    R_SCI_UART_CallbackSet(&g_uart0_ctrl, uart_callback, NULL, NULL); // 콜백 함수 등록

}


// ■ ADC 콜백 함수 구현
void adc_callback(adc_callback_args_t * p_args)
{
    if (p_args->event == ADC_EVENT_SCAN_COMPLETE) {
        g_scan_complete = true;
    }
}


// ■ ADC 초기화
void adc_init(){
    // ADC OPEN
    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    //    if(err == FSP_SUCCESS) uart_write("\033[34mADC OPEN 성공", NO_VAR);
    //    else uart_write("ADC OPEN 성공", err);

    // ADC Scan Config
    err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
    //    if(err == FSP_SUCCESS) uart_write("ADC SCAN 설정 성공했습니다.", NO_VAR);
    //    else uart_write("\033[37;41mADC SCAN 설정 실패했습니다.", err); // \033[37;41m: 빨간배경 흰색 글씨

}



void ring_buf_init(ring_buf_t *rb) {
    memset(rb->buffer, 0, sizeof(rb->buffer));
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->sum = 0;
}

void ring_buf_push(ring_buf_t *rb, uint16_t data) {
    // 새 데이터를 버퍼에 추가하고 합계 업데이트
    if(data != 0){
        // 기존 위치의 데이터를 합계에서 제거
        if (rb->count == ADC_BUFFER_SIZE) {
            rb->sum -= rb->buffer[rb->head];
            rb->head = (rb->head + 1) % ADC_BUFFER_SIZE;
        }

        rb->buffer[rb->tail] = data;
            rb->sum += data;
            rb->tail = (rb->tail + 1) % ADC_BUFFER_SIZE;

            // 버퍼 카운트 조정
            if (rb->count < ADC_BUFFER_SIZE) {
                rb->count++;
            }
    }
}

uint16_t ring_buf_avg(ring_buf_t *rb) {
    // 평균을 계산하여 반환
    return rb->count > 0 ? rb->sum / rb->count : 0;
}


// ■ ADC 조도센서 읽어오기 (반환값: adc 평균 데이터)
int adc_read(){
    // ADC SCAN
    err = R_ADC_ScanStart(&g_adc0_ctrl);
    //    if(err == FSP_SUCCESS) uart_write("ADC SCAN START 성공했습니다.", NO_VAR);
    //    else uart_write("\033[37;41mADC SCAN START실패했습니다.", err);


    // ADC READ
    // [참고] adc_data  -> 전압 으로 바꾸고 싶으면, 4095.0으로 나누고 5 곱하기
    err = R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_0, &g_adc_data);
    if(err == FSP_SUCCESS) {
//        uart_write("ADC READ 성공했습니다.", NO_VAR);
//        uart_write("ADC 데이터", (uint16_t)g_adc_data); // 흰색: \033[0m

        ring_buf_push(&g_adc_buffer, g_adc_data);
        uint16_t average = ring_buf_avg(&g_adc_buffer);
        return average;
    }
    else {
        uart_write("\033[37;41mADC READ 실패했습니다", err);
        return 0;
    }

}

// ■ GPT OPEN
void gpt_open(timer_ctrl_t * const p_ctrl, timer_cfg_t const * const p_cfg) {
    err = R_GPT_Open(p_ctrl, p_cfg);
    //    if (err == FSP_SUCCESS) uart_write("GPT OPEN 성공했습니다", NO_VAR);
    //    else uart_write("\033[37;41mGPT OPEN 실패했습니다", err);

}

// ■ GPT SET Period
void set_period(timer_ctrl_t * const p_ctrl, uint32_t const period_counts) {
    err = R_GPT_PeriodSet(p_ctrl, period_counts);
    //    if (err == FSP_SUCCESS) uart_write("\033[35mPeriod Set 성공했습니다", NO_VAR);
    //    else uart_write("\033[37;41mPeriod Set 실패했습니다", err);
}


// ■ GPT Duty Cycle 변경
void set_duty_cycle(timer_ctrl_t * const p_ctrl, uint32_t const duty_cycle) {
    err = R_GPT_DutyCycleSet(p_ctrl, duty_cycle, GPT_IO_PIN_GTIOCA);
    //    if (err == FSP_SUCCESS) uart_write("\033[35mDuty Cycle 설정에 성공했습니다", (uint16_t)duty_cycle); // \033[35m : 보라색
    //    else uart_write("\033[37;41mDuty Cycle 설정에 실패했습니다", err);

    if (p_ctrl == &g_timer3_ctrl) g_R_LED_duty_cycle = duty_cycle;
    else if (p_ctrl == &g_timer4_ctrl) g_G_LED_duty_cycle = duty_cycle;
    else if (p_ctrl == &g_timer6_ctrl) g_B_LED_duty_cycle = duty_cycle;
}



// ■ GPT START
void start_gpt(timer_ctrl_t * const p_ctrl) {
    err = R_GPT_Start(p_ctrl);
    //    if (err == FSP_SUCCESS) uart_write("START GPT 성공했습니다", NO_VAR); // 성공 메시지
    //    else uart_write("\033[37;41mSTART GPT 실패했습니다", err); // 실패 메시지와 오류 코드
}


// ■ PWM 초기화
void pwm_init(){
    // (1) GPT OPEN : 3, 4, 6
    gpt_open(&g_timer3_ctrl, &g_timer3_cfg);
    gpt_open(&g_timer4_ctrl, &g_timer4_cfg);
    gpt_open(&g_timer6_ctrl, &g_timer6_cfg);

    // (2) SET Period
    set_period(&g_timer3_ctrl, RGB_PWM_PERIOD);
    set_period(&g_timer4_ctrl, RGB_PWM_PERIOD);
    set_period(&g_timer6_ctrl, RGB_PWM_PERIOD);

    // (3) SET Duty Cycle
    set_duty_cycle(&g_timer3_ctrl, RGB_PWM_PERIOD);
    set_duty_cycle(&g_timer4_ctrl, RGB_PWM_PERIOD);
    set_duty_cycle(&g_timer6_ctrl, RGB_PWM_PERIOD);

    // (4) GPT Start
    start_gpt(&g_timer3_ctrl);
    start_gpt(&g_timer4_ctrl);
    start_gpt(&g_timer6_ctrl);

    // GPT Callback 등록
    R_GPT_CallbackSet(&g_timer3_ctrl, g_timer_callback , NULL, NULL);
    R_GPT_CallbackSet(&g_timer4_ctrl, g_timer_callback , NULL, NULL);
    R_GPT_CallbackSet(&g_timer6_ctrl, g_timer_callback , NULL, NULL);

}

// ■ RGB_LED 점등 여부
_Bool is_RGB_LED_ON(){
    if(g_R_LED_duty_cycle>0 || g_G_LED_duty_cycle >0 || g_B_LED_duty_cycle >0) return true;
    else return false;
}

// ■ RGB_LED 끄기 (모든 LED OFF)
void RGB_LED_OFF(){
    g_R_LED_duty_cycle = 0;
    g_G_LED_duty_cycle = 0;
    g_B_LED_duty_cycle = 0;

    set_duty_cycle(&g_timer3_ctrl, g_R_LED_duty_cycle);
    set_duty_cycle(&g_timer4_ctrl, g_G_LED_duty_cycle);
    set_duty_cycle(&g_timer6_ctrl, g_B_LED_duty_cycle);
}

// ■ RGB_LED 켜기 (모든 LED ON)
void RGB_LED_ON(){
    g_R_LED_duty_cycle = RGB_PWM_PERIOD;
    g_G_LED_duty_cycle = RGB_PWM_PERIOD;
    g_B_LED_duty_cycle = RGB_PWM_PERIOD;

    set_duty_cycle(&g_timer3_ctrl, g_R_LED_duty_cycle);
    set_duty_cycle(&g_timer4_ctrl, g_G_LED_duty_cycle);
    set_duty_cycle(&g_timer6_ctrl, g_B_LED_duty_cycle);
}

// ■ RGB_LED 반만 켜기 (약간 어둡게)
void RGB_HALF_ON(){
    g_R_LED_duty_cycle = gamma_correct_duty_cycle(RGB_PWM_PERIOD/2);
    g_G_LED_duty_cycle = gamma_correct_duty_cycle(RGB_PWM_PERIOD/2);
    g_B_LED_duty_cycle = gamma_correct_duty_cycle(RGB_PWM_PERIOD/2);

    set_duty_cycle(&g_timer3_ctrl, g_R_LED_duty_cycle);
    set_duty_cycle(&g_timer4_ctrl, g_G_LED_duty_cycle);
    set_duty_cycle(&g_timer6_ctrl, g_B_LED_duty_cycle);
}


// ■ 디바이스 초기화(Device initialization)
void Device_Init() {
    // UART
    uart_init();

    // ADC ( Analog to Digital )
    adc_init();

    // PWM ( Pulse Width Modulation )
    pwm_init();

    // ring buffer init
    ring_buf_init(&g_adc_buffer);
}


// ■ 밝기에 따라 자동 RGB LED 점등
void auto_on_off(int adc_avg){
    if(g_manual_control) return; // 수동제어 활성화 동안, 자동제어 비활성화
    switch(is_RGB_LED_ON()){
        if(g_manual_control) return;
        // LED가 켜져있을 때
        case true:
            // 주변이 밝으면,
            if(adc_avg >= ADC_THRESHOLD_HIGH) {
//                uart_write("\033[31mLED를 꺼야 해요", (uint16_t)adc_avg); // \033[31m : 빨강
                RGB_LED_OFF();
            }
            // 주변이 어두우면,
            else if(adc_avg <= ADC_THRESHOLD_LOW){
                // 유지
            }
            // 밝음과 어두움 중간,
            else {
//                uart_write("\033[31mLED를 조금 어둡게 켜야 해요.", (uint16_t)adc_avg); // \033[31m : 빨강
                RGB_HALF_ON();
            }
            break;

            // LED가 꺼져있을 때
        case false:
            // 주변이 어두우면,
            if(adc_avg <= ADC_THRESHOLD_LOW){
//                uart_write("\033[31mLED를 켜야 해요", (uint16_t)adc_avg);
                RGB_LED_ON();
            }
            // 주변이 밝으면,
            else if(adc_avg >= ADC_THRESHOLD_HIGH){
                // 유지
            }
            // 밝음과 어두움 중간,
            else {
                uart_write("\033[31mLED를 조금 어둡게 켜야 해요.", (uint16_t)adc_avg); // \033[31m : 빨강
                RGB_HALF_ON();
            }
            break;
    }
}


/***
 RGB LED 감마 보정
 [새로 알게 됨] LED 인지 비선형성, 감마보정
 사람의 눈이 LED의 밝기를 "비선형적"으로 인식
 듀티 사이클을 50%에서 60%로 증가시킬 때, 눈으로 느끼는 밝기 변화나
 듀티 사이클을 10%에서 20%로 증가시킬 때, 눈으로 느끼는 밝기 변화가
 사람 눈에는 다르게 인지됨
 이런 "비선형성을 보정"하기 위해, "감마 보정"을 사용
 감마보정: 듀티사이클에 비선형 함수를 적용하여, 밝기가 선형적으로 느껴지도록 보정
         공식 : L_out = L_in ^ γ(gamma), L_in은 [0,1]로 정규화된 데이터
 ***/
// ■ 감마 보정
uint32_t gamma_correct_duty_cycle(uint32_t duty_cycle) {
    // 듀티 사이클을 [0, 1] 범위로 정규화
    double normalized_duty_cycle = (double)duty_cycle / RGB_PWM_PERIOD;

    // 감마 보정
    double corrected_intensity = pow(normalized_duty_cycle, 1/GAMMA);

    // 보정된 듀티 사이클을 다시 0-255 범위로 조정
    uint32_t corrected_duty_cycle = (uint32_t)(corrected_intensity * RGB_PWM_PERIOD);
    return corrected_duty_cycle;
}

// ■ RGB_LED Duty Cycle 변경을 통한, 밝기 조절
void set_duty_cycles_by_ratio(int n) {
//    // 전체 듀티 사이클 합을 계산
//    uint32_t total_duty = g_R_LED_duty_cycle + g_G_LED_duty_cycle + g_B_LED_duty_cycle;
//    uart_write("total duty", (uint16_t)total_duty);
    // LED OFF 상태이면,
    if(g_R_LED_duty_cycle == 0 && g_G_LED_duty_cycle == 0 && g_B_LED_duty_cycle == 0){
        // 아직 생각 중
        uart_write("No duty cycle set", NO_VAR);
        set_duty_cycle(&g_timer3_ctrl, RGB_PWM_PERIOD);
        set_duty_cycle(&g_timer6_ctrl, RGB_PWM_PERIOD);
    }
    else{
        // 새로운 듀티 사이클 계산
        uint32_t new_r_duty = gamma_correct_duty_cycle(g_R_LED_duty_cycle/ 3 * (uint32_t)n);
        uint32_t new_g_duty = gamma_correct_duty_cycle(g_G_LED_duty_cycle/ 3 * (uint32_t)n);
        uint32_t new_b_duty = gamma_correct_duty_cycle(g_B_LED_duty_cycle/ 3 * (uint32_t)n);


        // 새로운 듀티 사이클로 설정
        set_duty_cycle(&g_timer3_ctrl, new_r_duty);
        set_duty_cycle(&g_timer4_ctrl, new_g_duty);
        set_duty_cycle(&g_timer6_ctrl, new_b_duty);

        // 새로운 듀티 사이클을 전역 변수에 업데이트
        g_R_LED_duty_cycle = new_r_duty;
        g_G_LED_duty_cycle = new_g_duty;
        g_B_LED_duty_cycle = new_b_duty;
    }
}


// ■ 버튼으로 LED 수동 점등
void handle_btn_click(uint16_t btn_num){

    // 색상 변경 버튼 이면,
    if(btn_num == 1){
        g_brightness_btn_cnt = 0; // 밝기 버튼 클릭 횟수 초기화
        uart_write("1번 버튼,밝기 버튼 클릭횟수 초기화", (uint16_t)g_brightness_btn_cnt);
        g_color_btn_cnt++;
        g_color_btn_cnt = (g_color_btn_cnt)%3;
        switch(g_color_btn_cnt){
            case 1: // R
                set_duty_cycle(&g_timer3_ctrl, RGB_PWM_PERIOD);
                set_duty_cycle(&g_timer4_ctrl, 0);
                set_duty_cycle(&g_timer6_ctrl, 0);
                break;
            case 2: // G
                set_duty_cycle(&g_timer3_ctrl, 0);
                set_duty_cycle(&g_timer4_ctrl, RGB_PWM_PERIOD);
                set_duty_cycle(&g_timer6_ctrl, 0);
                break;
            case 0: // B
                set_duty_cycle(&g_timer3_ctrl, 0);
                set_duty_cycle(&g_timer4_ctrl, 0);
                set_duty_cycle(&g_timer6_ctrl, RGB_PWM_PERIOD);
                break;
        }
    }

    // 밝기 변경 버튼이면,
    else if(btn_num == 2){
        g_brightness_btn_cnt++;
        g_brightness_btn_cnt = (g_brightness_btn_cnt) % 4;
        switch(g_brightness_btn_cnt){
            case 1: // 어두움
                uart_write("밝기 변경 버튼, 어두움", 1);
                set_duty_cycles_by_ratio(1);
                write_duty_cycle();
                break;
            case 2: // 중간
                uart_write("밝기 변경 버튼, 중간", 2);
                set_duty_cycles_by_ratio(2);
                write_duty_cycle();
                break;
            case 3: // 밝음
                uart_write("밝기 변경 버튼, 밝음", 3);
                set_duty_cycles_by_ratio(3);
                write_duty_cycle();
                break;
            case 0: // 꺼짐
                uart_write("밝기 변경 버튼, 꺼짐", (uint16_t)g_brightness_btn_cnt);
                R_GPT_DutyCycleSet(&g_timer3_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_GPT_DutyCycleSet(&g_timer4_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_GPT_DutyCycleSet(&g_timer6_ctrl, 0, GPT_IO_PIN_GTIOCA);
                write_duty_cycle();
                break;
        }
    }
}

// ■ RGB LED Duty Cycle 출력
void write_duty_cycle() {
    uart_write("R DutyCycle", (uint16_t)g_R_LED_duty_cycle);
    uart_write("G DutyCycle", (uint16_t)g_G_LED_duty_cycle);
    uart_write("B DutyCycle", (uint16_t)g_B_LED_duty_cycle);
}

// ■ 색상 변경 클릭 여부 확인
void check_btn_clicked(bsp_io_port_pin_t BUTTON, bsp_io_level_t g_btn_level, ButtonState* g_btn_state){
    uint16_t btn_num = BUTTON - 4; // S1은 핀 5번, S2는 핀 6번 (BUTTON은 핀 번호)

    // 버튼 상태 읽어오기
    err = R_IOPORT_PinRead(&g_ioport_ctrl, BUTTON, &g_btn_level);
    if(err == FSP_SUCCESS){
        _Bool button_pressed = !g_btn_level; // PRESS 되면, BTN_LEVEL = 0
        switch(*g_btn_state){
            case WAITING_FOR_PRESS:
                if(button_pressed){
                    uart_write("\033[32m버튼이 PRESS 되었습니다", btn_num); // 초록: \033[32m
                    *g_btn_state = WAITING_FOR_RELEASE;
                }
                break;

                // PRESS 해제 되기를 기다리는 상태 (PRESS된 상태)
            case WAITING_FOR_RELEASE:
                if(!button_pressed){
                    uart_write("\033[32m버튼이 RELEASE 되었습니다", btn_num);
                    *g_btn_state = WAITING_FOR_PRESS;
                    // 버튼 클릭 시, on/off 처리
                    g_manual_control = true;  // 수동 제어 활성화
                    handle_btn_click(btn_num);
                }
                break;
        }
    }

}

// ■ 밝기를 Duty Cycle로 변경 (밝기 명령어 범위: 0~100), (DutyCycle 범위: 0~RGB_PWM_PERIOD)
uint32_t convert_brightness_to_duty_cycle(uint32_t brightness) {
    // 범위 초과 방지
    if (brightness > 100) brightness = 100;

    // 밝기 > Duty Cycle 은 선형 변환, 이후 감마 보정
    uint32_t duty_cycle = (brightness * RGB_PWM_PERIOD / 100);
    return gamma_correct_duty_cycle(duty_cycle);
}


// ■ 명령어 처리: 수신 확인 및 처리
void process_command(){
    // 수신 완료 했으면
    if(g_uart_rx_complete){
        // 다음의 새로운 데이터 입력 확인을 위해, 수신 미완료로 변경
        g_uart_rx_complete = false;

        // HDR명령어TAIL 형식인지 확인
        if (strncmp((char *)g_rx_buffer, "HDR", 3) == 0 && strstr((char *)g_rx_buffer, "TAIL") != NULL) {
            // 명령어 추출 (HDR와 TAIL 사이의 부분)
            volatile uint8_t *start = g_rx_buffer + 3;      // "HDR" 이후 예) T10ONTAIL | B100TAIL
            char *end = strstr((char *)g_rx_buffer, "TAIL");  // TAIL의 시작위치
            *end = '\0';  // start에서 "TAIL"이 떨어져 나가서, 사용자 입력 명령어만 남음

            // 이제 start는 사용자 입력 명령어 임
            // 명령어의 첫 글자 확인
            uint8_t first_cmd = start[0]; // R | G | B | T | A
            volatile char* secnd_cmd;
            uint32_t brightness; // 밝기
            uint32_t minutes;    // 시간
            switch(first_cmd){
                case 'R':
                    g_manual_control = true;  // 수동 제어 활성화
                    brightness = convert_brightness_to_duty_cycle((uint32_t)atoi(start + 1));
                    uart_write("\033[33mR LED 밝기 변경 명령어", (uint16_t)brightness);
                    set_duty_cycle(&g_timer3_ctrl, brightness);
                    write_duty_cycle();
                    break;
                case 'G':
                    g_manual_control = true;  // 수동 제어 활성화
                    brightness = convert_brightness_to_duty_cycle((uint32_t)atoi(start + 1));
                    uart_write("\033[33mG LED 밝기 변경 명령어", (uint16_t)brightness);
                    set_duty_cycle(&g_timer4_ctrl, brightness);
                    write_duty_cycle();
                    break;
                case 'B':
                    g_manual_control = true;  // 수동 제어 활성화
                    brightness = convert_brightness_to_duty_cycle((uint32_t)atoi(start + 1)); // "B" 다음 숫자 추출
                    uart_write("\033[33mB LED 밝기 변경 명령어", (uint16_t)brightness);
                    set_duty_cycle(&g_timer6_ctrl, brightness);
                    write_duty_cycle();
                    break;
                case 'T': {
                    g_manual_control = true;  // 수동 제어 활성화
                    uint8_t temp[3] = {0};  // 숫자 부분을 저장할 임시 버퍼 (최대 "99"까지)
                    int i = 1;  // start[1]부터 숫자 부분 확인

                    // 숫자 부분 추출 (최대 2자리)
                    while (isdigit(start[i]) && i < 3) {
                        temp[i - 1] = start[i];
                        i++;
                    }
                    temp[i - 1] = '\0';  // 문자열 종료
                    minutes = (uint32_t)atoi(temp);  // 문자열을 정수로 변환
                    secnd_cmd = start + i;  // "ON" 또는 "OFF" 부분


                    // 명령어가 ON 일 때,
                    if (strcmp(secnd_cmd, "ON") == 0) {
                        // LED가 이미 켜져있으면,
                        if (is_RGB_LED_ON()) {
                            uart_write("\033[37;41m이미 LED가 켜져 있습니다.", NO_VAR);
                            break;
                        }
                        // LED가 꺼져있으면, LED ON 타이머 실행
                        else set_timer(minutes, true);
                    }

                    // 명령어가 OFF 일 때,
                    else if (strcmp(secnd_cmd, "OFF") == 0) {
                        // LED가 이미 꺼져있으면,
                        if (!is_RGB_LED_ON()) {
                            uart_write("\033[37;41m이미 LED가 꺼져 있습니다.", NO_VAR);
                            break;
                        }
                        // LED가 켜져있으면, LED OFF 타이머 실행
                        else set_timer(minutes, false);

                    }

                    // ON/OFF 가 아니면, 잘못된 명령어
                    else command_err_handle();
                    for (i = 0; i < UART_RX_BUF_SIZE; i++) g_rx_buffer[i] = 0;
                    break;
                }

                case 'S':
                    g_timer_set = false;
                    g_new_tick = 0;
                    R_GPT_Reset(&g_timer3_ctrl);
                    R_GPT_Reset(&g_timer4_ctrl);
                    R_GPT_Reset(&g_timer6_ctrl);
                    uart_write("타이머가 리셋되었습니다.", NO_VAR);
                    for (uint8_t i = 0; i < UART_RX_BUF_SIZE; i++) g_rx_buffer[i] = 0;
                    break;
                case 'A':
                {
                    secnd_cmd = start + 1;  // "ON" 또는 "OFF" 부분
                    if(strncmp(secnd_cmd, "ON", 2) == 0){
                        uart_write("\033[35m자동모드+수동모드로 변환합니다.", NO_VAR);
                        g_manual_control = false; // auto mode ON
                    }
                    else if(strncmp(secnd_cmd, "OFF", 3) == 0){
                        uart_write("\033[35m수동모드로 변환합니다.", NO_VAR);
                        g_manual_control = true; // auto mode OFF
                    }
                    else command_err_handle();

                    break;
                }

                // LED ON / OFF (백색)
                case 'O':
                    secnd_cmd = start + 1;  // O -N 또는 FF
                    if(strncmp(secnd_cmd, "N", 1) == 0) {
                        RGB_LED_ON();
                        g_is_RGB_LED_ON_by_cmd = true;
                    }
                    else if(strncmp(secnd_cmd, "FF", 2)==0) {
                        RGB_LED_OFF();
                        g_is_RGB_LED_ON_by_cmd = false;
                    }
                    else command_err_handle();
                    break;

                // 프로그램 종료 (EXIT)
                case 'E':
                    secnd_cmd = start + 1; // XIT
                    if(strncmp(secnd_cmd, "XIT", 3) == 0) while(1);
                    else command_err_handle();
                    break;


                default:
                    command_err_handle();
            }
        }
        // 명령어 형식과 다르면... (HDR명령어TAIL이 아니면)
        else{
            command_err_handle();
        }

    }
}

// ■ 명령어 에러 처리: 형식에 맞지 않는 명령어 처리
void command_err_handle() {
    uart_write("\033[37;41m명령어 형식을 확인해주세요.", NO_VAR);
//    uart_write("\033[37현재 입력된 명령어 : ", NO_VAR);
//    uart_write((char *)g_rx_buffer, NO_VAR);
    uart_write("\033[37mHDR명령어TAIL", NO_VAR);
    uart_write("\033[37m[명령어] LED제어: R50 | G10 | B40", NO_VAR);
    uart_write("\033[37m[명령어] 타이머 : T10", NO_VAR);
    g_rx_index = 0;  // 인덱스 초기화
}


// ■ GPT 콜백 함수
void g_timer_callback(timer_callback_args_t *p_args) {
    (void)p_args; // 이벤트 사용하지 않음

    // 타이머가 설정되어 있고, 남은 시간이 있는 경우
    if (g_timer_set && (g_new_minutes > 0 || g_new_seconds > 0)) {
        g_new_tick++; // 타이머 카운트 증가

        // 1초마다
        if (g_new_tick >= TICK_PER_ONE_SEC) {
            g_new_tick = 0;
            if (g_new_seconds > 0) {
                g_new_seconds--;
            } else if (g_new_minutes > 0) {
                g_new_minutes--;
                g_new_seconds = 59;
            }

            g_uart_tick_output_flag = true; // UART 출력을 위한 플래그 설정
        }
    }

    // 남은 시간이 0이 되었을 때
    if (g_new_minutes == 0 && g_new_seconds == 0 && g_timer_set) {
        g_timer_set = false; // 타이머 종료
        if (g_is_RGB_LED_ON_by_cmd) {
            RGB_LED_ON(); // LED 점등
        } else {
            RGB_LED_OFF(); // LED 소등
        }
    }
}

// ■ 타이머 설정을 초기화하는 함수
void set_timer(uint32_t minutes, _Bool led_on) {
    uart_write(led_on ? "[타이머] LED ON 예약" : "[타이머] LED OFF 예약", (uint16_t)minutes);
    g_timer_target_ticks = minutes * TICK_PER_ONE_MIN;  // 타이머 목표 설정
    g_new_tick = 0; // 타이머 카운트 초기화
    g_timer_set = true;
    g_is_RGB_LED_ON_by_cmd = led_on;

    // 타이머 설정 시 초기 시간 설정
    g_new_minutes = minutes;  // 분으로 설정
    g_new_seconds = 0;        // 초는 0으로 시작
}

void write_time() {
    if (g_uart_tick_output_flag) {
        snprintf(g_tx_buffer, sizeof(g_tx_buffer), "\033[A\r\033[K%02d:%02d", g_new_minutes, g_new_seconds);
        R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t *)g_tx_buffer, strlen(g_tx_buffer));
        g_uart_tick_output_flag = false;
    }
}

/* hal_entry()■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */
    Device_Init();

    while (1) {
        // (1) 데이터 읽기 : READ ADC data
        int adc_avg = adc_read();

        // (2) 자동 조명 ON/OFF : Auto Light On/Off
        auto_on_off(adc_avg);

        // (3) 색상 변경 버튼 클릭?
        check_btn_clicked(BUTTON_S1, g_color_btn_level, &g_color_btn_state);

        // (4) 밝기 조절 버튼 클릭?
        check_btn_clicked(BUTTON_S2, g_brightness_btn_level, &g_brightness_btn_state);

        // (5) Command 수신 확인? (밝기 조절 Command | 타이머 설정 Command )
        process_command();
        write_time();

        // (6) DELAY
        delay_ms(HAL_ENTRY_DELAY);
    }

#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}




/* ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/




/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

#if BSP_CFG_SDRAM_ENABLED

        /* Setup SDRAM and initialize it. Must configure pins first. */
        R_BSP_SdramInit(true);
#endif
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
