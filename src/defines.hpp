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

/*#define SWITCH_0        GPIO_NUM_25
#define SWITCH_1        GPIO_NUM_22*/


/*#define LEDC_OUTPUT_IO0     GPIO_NUM_5  //inicializace STEP pinu
#define DIR_OUTPUT0         GPIO_NUM_4  //inicializace DIR pinu
#define LEDC_OUTPUT_IO1      GPIO_NUM_26  //inicializace STEP pinu
#define DIR_OUTPUT1         GPIO_NUM_21  //inicializace DIR pinu
#define LEDC_OUTPUT_IO2      GPIO_NUM_27  //inicializace STEP pinu
#define DIR_OUTPUT2         GPIO_NUM_2  //inicializace DIR pinu  //MUST BE LOW OR UNCONNECTED!!!
#define LEDC_OUTPUT_IO3      GPIO_NUM_14  //inicializace STEP pinu
#define DIR_OUTPUT3         GPIO_NUM_19  //inicializace DIR pinu*/
#define PCNT_INPUT_0        GPIO_NUM_12 //BOOT FAILS IF PULLED HIGH!!!
#define PCNT_INPUT_1        GPIO_NUM_18
#define PCNT_INPUT_2        GPIO_NUM_15
#define PCNT_INPUT_3        GPIO_NUM_13
//#define PCNT_H_LIM_VAL      1
volatile int pcnt0_count = 0;
volatile int pcnt1_count = 0;
volatile int pcnt2_count = 0;
volatile int pcnt3_count = 0;

#define GPIO_BIT_MASK_INPUTS ((1ULL<<KONCOVY_DOJEZD_0) | (1ULL<<KONCOVY_DOJEZD_1) | (1ULL<<KONCOVY_DOJEZD_2) | (1ULL<<KONCOVY_DOJEZD_3) | (1ULL<<SILOVKA)/*| (1ULL<<SWITCH_0) | (1ULL<<SWITCH_1)*/)

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
//volatile int motor_load = 0;
//volatile int motor_stop_sensitivity = 100;
//volatile int potenciometr = 0;
//volatile int i_run = 8;
//volatile int i_hold = 0;
//volatile bool start_stop = true;
//volatile bool loop = false;
//volatile uint mot_load[2048];
//volatile uint mot_pos[2048];
//volatile int count=0;

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