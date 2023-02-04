#define GRIDUI_LAYOUT_DEFINITION
#include "layout.hpp"     //  pro grafické rozhraní
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "defines.hpp"
#include "utils.hpp"
#include "init.hpp"
#include "uart.hpp"
#include "driver.hpp"
#include "gridui.h"         //  pro grafické rozhraní          
#include "rbprotocol.h"     //  pro grafické rozhraní
#include "rbwebserver.h"    //  pro grafické rozhraní
#include "rbwifi.h"         //  pro grafické rozhraní
#include "driver/ledc.h"
#include "soc/gpio_sig_map.h"
#include "soc/pcnt_reg.h"
#include "soc/pcnt_struct.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc_wdt.h"
#include "parser.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <atomic>

using namespace rb;
using namespace std;

static void initGridUi() {
    using namespace gridui;
    // Initialize WiFi
    WiFi::startAp("Oskar95_Denis","oskaroskar");     //esp vytvoří wifi sít
    // WiFi::connect("Jmeno wifi", "Heslo");    //připojení do místní sítě
    
    // Initialize RBProtocol
    auto *protocol = new Protocol("burda", "Oskar_Denis", "Compiled at " __DATE__ " " __TIME__, [](const std::string& cmd, rbjson::Object* pkt) {
        UI.handleRbPacket(cmd, pkt);
    });
    protocol->start();
    // Start serving the web page
    rb_web_start(80);
    // Initialize the UI builder
    UI.begin(protocol);
    // Build the UI widgets. Positions/props are set in the layout, so most of the time,
    // you should only set the event handlers here.
    auto builder = gridui::Layout.begin();
    //builder.MotorSpeed.min(MOTOR_SPEED_MIN);
    //builder.MotorSpeed.max(MOTOR_SPEED_MAX);
   /* builder.Arm1.onPositionChanged([](Arm &s)){

    }*/

   /* builder.ButtonStart.onPress([](Button &s){
        gpio_set_level(GPIO_NUM_32, 1);
        printf("Zapnuti\n");
    });
    builder.ButtonStop.onPress([](Button &s){
        gpio_set_level(GPIO_NUM_32, 0);
        printf("Vypnuti\n");
    });*/
    builder.MotorSpeed0.onChanged([](Slider &s) {
        motor_speed[0] = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed0: %f -> %d\n", s.value(), motor_speed[0].load());   
    });
    builder.MotorSpeed1.onChanged([](Slider &s) {
        motor_speed[1] = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed1: %f -> %d\n", s.value(), motor_speed[1].load());   
    });
    builder.MotorSpeed2.onChanged([](Slider &s) {
        motor_speed[2] = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed2: %f -> %d\n", s.value(), motor_speed[2].load());   
    });
    builder.MotorSpeed3.onChanged([](Slider &s) {
        motor_speed[3] = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed3: %f -> %d\n", s.value(), motor_speed[3].load());   
    }); 

    /* builder.StopSensitivity.onChanged([](Slider &s) {
        motor_stop_sensitivity = int(s.value());
        printf("stop sensitivity:%f -> %d\n",s.value(), motor_stop_sensitivity);
    });

    builder.IRun.onChanged([](Slider &s) {
        i_run = int(s.value());
        printf("I_Run / 32:%f -> %d\n",s.value(), i_run);
    });
    */

    // Commit the layout. Beyond this point, calling any builder methods on the UI is invalid.
    builder.commit();
}

    
   static void initDriver(Driver& driver, const int iRun, const int iHold) {
    driver.init();
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    uint32_t data =0;

    int result = driver.get_PWMCONF(data);
    if (result != 0)
        printf("PWMCONF driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("PWMCONF driveru %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    result = driver.get_DRV_STATUS(data);
    if (result != 0)
        printf("DRV_STATUS driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("DRV_STATUS driveru %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    result = driver.read_gconf(data);
    if (result != 0)
        printf("GCONF %d : ERROR  %d\n", driver.address(), result);
    else
        printf("GCONF %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    result = driver.get_CHOPCONF(data);
    if (result != 0)
        printf("CHOPCONF %d : ERROR  %d\n", driver.address(), result);
    else
        printf("CHOPCONF %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    result = driver.get_GSTAT(data);
    if (result != 0)
        printf("GSTAT %d : ERROR  %d\n", driver.address(), result);
    else
        printf("GSTAT %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    driver.set_speed(0);                      // otáčení motoru se nastavuje zápisem rychlosti do driveru přes Uart
    driver.set_IHOLD_IRUN (16, 32);             // proud IHOLD (při stání) =8/32, IRUN (při běhu)= 8/32 (8/32 je minimum, 16/32 je maximum pro dluhodobější provoz) 
    if(gpio_get_level(SILOVKA)){
        driver.enable();
        //driver_stdby = 1;
        printf("\n");
        printf("Zapnuti driveru!\n");
        printf("\n");
    }
    else{
        driver.disable();
        printf("\n");
        printf("Napajeni nepripojeno!\n");
        printf("\n");
    }                
    vTaskDelay(300 / portTICK_PERIOD_MS);     //doba stání pro nastavení automatiky driveru
    driver.set_IHOLD_IRUN (iRun, iHold);             //proud IHOLD =0, IRUN = 8/32 (při stání je motor volně otočný)
   }
   ////////////////////////BUZZER INIT//////////////////////////
   void beep(uint beeps,uint delay){
        step_pulse_init(2000,LEDC_TIMER_0,BUZZER,LEDC_CHANNEL_0);
        for(int i=0; i<beeps; i++){
            ledc_timer_pause(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
            vTaskDelay(delay / portTICK_PERIOD_MS);
            ledc_timer_resume(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
            vTaskDelay(delay / portTICK_PERIOD_MS);
        } 
        ledc_timer_pause(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
   }

extern "C" void app_main(void)
{   
    beep(1, 1000);
    // zapnuti siloveho napajeni do driveru*/
    driver_stdby = gpio_get_level(SILOVKA);
    gpio_set_level(VCC_IO, 1);
    printf("Oskar95 start \n\tbuild %s %s\n", __DATE__, __TIME__);
    check_reset();
    iopins_init();
    //optozávory inicializace + tlačítka
    gpio_config_t silovka_conf = {
        .pin_bit_mask = (1ULL<<SILOVKA),   //inicializuje pin silovka
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&silovka_conf);

    gpio_config_t opto_pin_conf = {
        .pin_bit_mask = GPIO_BIT_MASK_KONCOVE_DOJEZDY,   //inicializuje piny 35,34,36,39
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&opto_pin_conf);

    gpio_config_t diag_pin_conf = {};
        diag_pin_conf.pin_bit_mask = GPIO_BIT_MASK_DIAG_PINS;  //inicializuje piny 19,21,22,23
        diag_pin_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        diag_pin_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        diag_pin_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        diag_pin_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&diag_pin_conf);

    if(gpio_get_level(SILOVKA)){
        diag_pin_conf.mode = GPIO_MODE_INPUT;  
        diag_pin_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; 
        gpio_config(&diag_pin_conf); 
        gpio_set_level(VCC_IO, 0);              // reset driveru
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(VCC_IO, 1); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    nvs_init();                             //inicializace pro zápis do flash paměti
    initGridUi();

    Uart drivers_uart {
        DRIVERS_UART,
        Uart::config_t {
            .baud_rate = 750000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .use_ref_tick = false
        },
        Uart::pins_t {
            .pin_txd = DRIVERS_UART_TXD,
            .pin_rxd = DRIVERS_UART_RXD,
            .pin_rts = UART_PIN_NO_CHANGE,
            .pin_cts = UART_PIN_NO_CHANGE
        },
        Uart::buffers_t {
            .rx_buffer_size = DRIVERS_UART_BUF_SIZE,
            .tx_buffer_size = 0,
            .event_queue_size = 0
        }
    };

    pcnt_init(PCNT_UNIT_0,PCNT_INPUT_0);
    pcnt_init(PCNT_UNIT_1,PCNT_INPUT_1);
    pcnt_init(PCNT_UNIT_2,PCNT_INPUT_2);
    pcnt_init(PCNT_UNIT_3,PCNT_INPUT_3);

    printf("\n");
    Driver driver0 { drivers_uart, DRIVER_0_ADDRES, ENN_PIN0, PCNT_UNIT_0};
    initDriver(driver0, 31, 10);
    printf("\n");
    Driver driver1 { drivers_uart, DRIVER_1_ADDRES, ENN_PIN1, PCNT_UNIT_1};
    initDriver(driver1, 31, 10); 
    printf("\n");
    Driver driver2 { drivers_uart, DRIVER_2_ADDRES, ENN_PIN2, PCNT_UNIT_2};
    initDriver(driver2, 31, 10);
    printf("\n");
    Driver driver3 { drivers_uart, DRIVER_3_ADDRES, ENN_PIN3, PCNT_UNIT_3};
    initDriver(driver3, 31, 10);
    printf("\n");

    xTaskCreate(g_code_parser,"parsing task",2048, NULL, tskIDLE_PRIORITY,NULL);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_control_task, "Intr_Control_Task", 2048, NULL, tskIDLE_PRIORITY, NULL);
    gpio_install_isr_service(0);

    gpio_isr_handler_add(DIAG_PIN0, gpio_intr_isr, (void*) DIAG_PIN0);
    gpio_isr_handler_add(DIAG_PIN1, gpio_intr_isr, (void*) DIAG_PIN1);
    gpio_isr_handler_add(DIAG_PIN2, gpio_intr_isr, (void*) DIAG_PIN2);
    gpio_isr_handler_add(DIAG_PIN3, gpio_intr_isr, (void*) DIAG_PIN3);

    TaskHandle_t pulse_count;
    xTaskCreatePinnedToCore(pulse,"pulse counter",4096,NULL,tskIDLE_PRIORITY,&pulse_count,1);

    TaskHandle_t silovka_init;
    xTaskCreatePinnedToCore(silovka,"silovka check",4096,NULL,tskIDLE_PRIORITY,&silovka_init,1);

    /*pcnt_set_event_value(PCNT_UNIT_0,PCNT_EVT_H_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_1,PCNT_EVT_H_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_3,PCNT_EVT_H_LIM,0);

    pcnt_set_event_value(PCNT_UNIT_0,PCNT_EVT_L_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_1,PCNT_EVT_L_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_L_LIM,0);
    pcnt_set_event_value(PCNT_UNIT_3,PCNT_EVT_L_LIM,0);*/

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);

    for(int i=0; i<4; i++){
        motor_speed[i]=17902;
    }

    uint32_t SG0=0;
    uint32_t SG1=0;
    uint32_t SG2=0;
    uint32_t SG3=0;

    driver0.set_SGTHRS(1);
    driver1.set_SGTHRS(1);
    driver2.set_SGTHRS(1);
    driver3.set_SGTHRS(1);

    driver0.set_TCOOLTHRS(0xFFFFF);
    driver1.set_TCOOLTHRS(0xFFFFF);
    driver2.set_TCOOLTHRS(0xFFFFF);
    driver3.set_TCOOLTHRS(0xFFFFF);

    beep(5, 200);
   
    //ledc_timer_pause();
    //ledc_timer_resume();
    
    //driver2.set_speed(motor_speed2);

    /*while(gpio_get_level(SILOVKA)){

       if(h_limits[2]>x){
        driver2.set_speed(-motor_speed2);
        x=h_limits[2];
       }
       else if(l_limits[2]>y){
        driver2.set_speed(motor_speed2);
        y=l_limits[2];
       }
        if(h_limits[2]==1){
            //driver2.set_speed(speed);
            pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,100);
        }
        if(h_limits[2]==2){
           pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,100); 
        }
        if(h_limits[2]==3){
           pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,100); 
        }
        if(h_limits[2]==4){
           pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,100); 
        }
        if(h_limits[2]==5
        ){
            driver2.set_speed(0);
        }
        
       printf("%d\n",h_limits[2]);
        
        driver0.get_SG(SG0);
        driver1.get_SG(SG1);
        driver2.get_SG(SG2);
        driver3.get_SG(SG3);
        printf("SG0: %d  SG1: %d  SG2: %d  SG3: %d\n",SG0,SG1,SG2,SG3);
        
        printf("konc dojezd2: %d\n",gpio_get_level(KONCOVY_DOJEZD_2));

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }*/

    while(gpio_get_level(SILOVKA)){
        driver0.set_speed(motor_speed[0]);
        driver1.set_speed(motor_speed[1]);
        driver2.set_speed(motor_speed[2]);
        driver3.set_speed(motor_speed[3]);
        
        vTaskDelay(10/portTICK_PERIOD_MS);
    } 
}
    
    