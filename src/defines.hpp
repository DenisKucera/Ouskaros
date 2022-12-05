#pragma once

#define DRIVER_0_ADDRES           0
#define DRIVER_1_ADDRES           1
#define DRIVER_2_ADDRES           2
#define DRIVER_3_ADDRES           3

#define DRIVER_0_ENABLE           GPIO_NUM_23   // H= disable motor output
#define DRIVER_1_ENABLE           GPIO_NUM_23
#define DRIVER_2_ENABLE           GPIO_NUM_23
#define DRIVER_3_ENABLE           GPIO_NUM_23

#define VCC_IO                  GPIO_NUM_33  // L = reset driver 0, H = driver0 on
//#define VCC_IO_1                  GPIO_NUM_33  // L = reset driver 0, H = driver0 on
//#define VCC_IO_2                  GPIO_NUM_33  // L = reset driver 0, H = driver0 on
//#define VCC_IO_3                  GPIO_NUM_33  // L = reset driver 0, H = driver0 on

#define SW_CTRL                   GPIO_NUM_32  // L = transistor Q3 off -> motor power off, H = all drivers on

#define KONCOVY_DOJEZD_0          GPIO_NUM_35
#define KONCOVY_DOJEZD_1          GPIO_NUM_34
#define KONCOVY_DOJEZD_2          GPIO_NUM_39
#define KONCOVY_DOJEZD_3          GPIO_NUM_36

/*#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      10
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO0   4  // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO1     // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO2     // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO3     // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down*/

#define LEDC_OUTPUT_IO0     GPIO_NUM_5  //inicializace STEP pinu
#define DIR_OUTPUT0         GPIO_NUM_4  //inicializace DIR pinu
#define LEDC_OUTPUT_IO1      GPIO_NUM_26  //inicializace STEP pinu
#define DIR_OUTPUT1         GPIO_NUM_21  //inicializace DIR pinu
#define LEDC_OUTPUT_IO2      GPIO_NUM_27  //inicializace STEP pinu
#define DIR_OUTPUT2         GPIO_NUM_2  //inicializace DIR pinu  //MUST BE LOW OR UNCONNECTED!!!
#define LEDC_OUTPUT_IO3      GPIO_NUM_14  //inicializace STEP pinu
#define DIR_OUTPUT3         GPIO_NUM_19  //inicializace DIR pinu
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_INPUT_0        GPIO_NUM_12 //BOOT FAILS IF PULLED HIGH!!!
#define PCNT_INPUT_1        GPIO_NUM_13
#define PCNT_INPUT_2        GPIO_NUM_15
#define PCNT_INPUT_3        GPIO_NUM_18
#define PCNT_H_LIM_VAL      1
#define PCNT_L_LIM_VAL     -1
#define PCNT_THRESH1_VAL    1000
#define PCNT_THRESH0_VAL   -1000
volatile int pcnt0_count = 0;

#define GPIO_BIT_MASK_INPUTS ((1ULL<<KONCOVY_DOJEZD_0) | (1ULL<<KONCOVY_DOJEZD_1) | (1ULL<<KONCOVY_DOJEZD_2) | (1ULL<<KONCOVY_DOJEZD_3))

#define DRIVERS_UART              UART_NUM_1
#define DRIVERS_UART_TXD          GPIO_NUM_17  // doma 27
#define DRIVERS_UART_RXD          GPIO_NUM_16  // doma 26
#define DRIVERS_UART_BUF_SIZE     256
#define DRIVERS_RX_TIMEOUT        (20 / portTICK_RATE_MS)
#define DRIVERS_UART_START_BYTE   0x05

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<DRIVER_0_ENABLE) | (1ULL<<SW_CTRL) | (1ULL<<VCC_IO) | (1ULL<<DIR_OUTPUT0) | (1ULL<<DIR_OUTPUT2)  | (1ULL<<DIR_OUTPUT1) | (1ULL<<DIR_OUTPUT3))

#define MOTOR_SPEED_COEFICIENT    71608    // 71608 = 1RPS VACTUAL 0x22= 2*23 uSTEPS/t

#define ENCODER_H_LIM_VAL         1000
#define ENCODER_L_LIM_VAL        -1000

// globální proměnné pro pokusy s grafickým rozhraním
volatile int motor_speed0;
volatile int motor_speed1;
volatile int motor_speed2;
volatile int motor_speed3;
volatile int hodnota0;
volatile int hodnota1; 
volatile int hodnota2; 
volatile int hodnota3;
uint32_t position0;
uint32_t position1;
uint32_t position2;
uint32_t position3;
volatile int motor_load = 0;
volatile int motor_stop_sensitivity = 100;
volatile int potenciometr = 0;
volatile int i_run = 8;
volatile int i_hold = 0;
volatile bool start_stop = true;
volatile bool x = false;
volatile uint mot_load[2048];
volatile uint mot_pos[2048];
volatile int count=0;