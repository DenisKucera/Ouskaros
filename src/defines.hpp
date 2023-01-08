#pragma once

#define DRIVER_0_ADDRES           0
#define DRIVER_1_ADDRES           1
#define DRIVER_2_ADDRES           2
#define DRIVER_3_ADDRES           3

//#define VCC_IO                      GPIO_NUM_14

#define VCC_IO                   GPIO_NUM_32  // L = transistor Q3 off -> motor power off, H = all drivers on

#define ENN_PIN0                   GPIO_NUM_33
#define ENN_PIN1                   GPIO_NUM_25
#define ENN_PIN2                   GPIO_NUM_26
#define ENN_PIN3                   GPIO_NUM_27

#define SILOVKA                    GPIO_NUM_4

#define KONCOVY_DOJEZD_0          GPIO_NUM_35
#define KONCOVY_DOJEZD_1          GPIO_NUM_34
#define KONCOVY_DOJEZD_2          GPIO_NUM_39
#define KONCOVY_DOJEZD_3          GPIO_NUM_36

#define DIAG_PIN0                 GPIO_NUM_19
#define DIAG_PIN1                 GPIO_NUM_21
#define DIAG_PIN2                 GPIO_NUM_22
#define DIAG_PIN3                 GPIO_NUM_23

#define PCNT_INPUT_0        GPIO_NUM_12 //BOOT FAILS IF PULLED HIGH!!!
#define PCNT_INPUT_1        GPIO_NUM_18
#define PCNT_INPUT_2        GPIO_NUM_15
#define PCNT_INPUT_3        GPIO_NUM_13

volatile int pcnt0_count = 0;
volatile int pcnt1_count = 0;
volatile int pcnt2_count = 0;
volatile int pcnt3_count = 0;

#define GPIO_BIT_MASK_INPUTS ((1ULL<<KONCOVY_DOJEZD_0) | (1ULL<<KONCOVY_DOJEZD_1) | (1ULL<<KONCOVY_DOJEZD_2) | (1ULL<<KONCOVY_DOJEZD_3) | (1ULL<<SILOVKA) | (1ULL<<DIAG_PIN0) | (1ULL<<DIAG_PIN1) | (1ULL<<DIAG_PIN2) | (1ULL<<DIAG_PIN3))

#define DRIVERS_UART              UART_NUM_1
#define DRIVERS_UART_TXD          GPIO_NUM_17 
#define DRIVERS_UART_RXD          GPIO_NUM_16 
#define DRIVERS_UART_BUF_SIZE     256
#define DRIVERS_RX_TIMEOUT        (20 / portTICK_RATE_MS)
#define DRIVERS_UART_START_BYTE   0x05

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<VCC_IO))

#define MOTOR_SPEED_COEFICIENT    71608    // 71608 = 1RPS VACTUAL 0x22= 2*23 uSTEPS/t

#define ENCODER_H_LIM_VAL         1000
#define ENCODER_L_LIM_VAL        -1000

// globální proměnné pro pokusy s grafickým rozhraním
volatile int q=0;
volatile int driver_stdby=0;
volatile int motor_speed0;
volatile int motor_speed1;
volatile int motor_speed2;
volatile int motor_speed3;
volatile int hodnota0;
volatile int hodnota1; 
volatile int hodnota2; 
volatile int hodnota3;
int axis0_max;
int axis1_max;
int axis2_max;
int axis3_max;
uint16_t count0=0;
uint16_t count1=0;
uint16_t count2=0;
uint16_t count3=0;
uint32_t mscurrent0;
uint32_t drvstatus0;
uint32_t pwmconf0;
uint32_t gconf0;
uint32_t sgresult0;
volatile int loop=0;

volatile uint16_t position0=0;
volatile uint16_t position1=0;
volatile uint16_t position2=0;
volatile uint16_t position3=0;

volatile bool motor0_cal=false;
volatile bool motor1_cal=false;
volatile bool motor2_cal=false;
volatile bool motor3_cal=false;

volatile bool motor0_cal_done=false;
volatile bool motor1_cal_done=false;
volatile bool motor2_cal_done=false;
volatile bool motor3_cal_done=false;

volatile int driver0_const=75; //139,149
volatile int driver1_const=275; //558,568
volatile int driver2_const=500; //1003,993
volatile int driver3_const=450; //955,962

volatile bool motor0_done=false;
volatile bool motor1_done=false;
volatile bool motor2_done=false;
volatile bool motor3_done=false;

volatile bool set_motors_done=true;

volatile int read_pos0=0;
volatile int read_pos1=0;
volatile int read_pos2=0;
volatile int read_pos3=0;