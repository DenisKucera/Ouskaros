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
#include "pcnt.hpp"
#include "soc/rtc_wdt.h"

using namespace rb;

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
        motor_speed0 = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed0: %f -> %d\n", s.value(), motor_speed0);   
    });
    builder.MotorSpeed1.onChanged([](Slider &s) {
        motor_speed1 = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed1: %f -> %d\n", s.value(), motor_speed1);   
    });
    builder.MotorSpeed2.onChanged([](Slider &s) {
        motor_speed2 = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed2: %f -> %d\n", s.value(), motor_speed2);   
    });
    builder.MotorSpeed3.onChanged([](Slider &s) {
        motor_speed3 = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed3: %f -> %d\n", s.value(), motor_speed3);   
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

    result = driver.get_MSCURACT(data);
    if (result != 0)
        printf("MSCURACT %d : ERROR  %d\n", driver.address(), result);
    else
        printf("MSCURACT %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    result = driver.read_gconf(data);
    if (result != 0)
        printf("GCONF %d : ERROR  %d\n", driver.address(), result);
    else
        printf("GCONF %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    result = driver.get_SG(data);
    if (result != 0)
        printf("STALLGUARD RESULT %d : ERROR  %d\n", driver.address(), result);
    else
        printf("STALLGUARD RESULT %d =  %08X\n", driver.address(), data);
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

    result = driver.get_MSCNT(data);
    if (result != 0)
        printf("MSCNT %d : ERROR  %d\n", driver.address(), result);
    else
        printf("MSCNT %d =  %08X\n", driver.address(), data);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    driver.set_speed(0);                      // otáčení motoru se nastavuje zápisem rychlosti do driveru přes Uart
    driver.set_IHOLD_IRUN (16, 32);             // proud IHOLD (při stání) =8/32, IRUN (při běhu)= 8/32 (8/32 je minimum, 16/32 je maximum pro dluhodobější provoz)
    driver.enable();                          //zapnutí mptoru
    vTaskDelay(300 / portTICK_PERIOD_MS);     //doba stání pro nastavení automatiky driveru
    driver.set_IHOLD_IRUN (iRun, iHold);             //proud IHOLD =0, IRUN = 8/32 (při stání je motor volně otočný)
   }
   /*void driver_sel(const int driver_address){
    
   }
   int driver_cal_speed(){
    int speed=17902;
    return speed;
   }*/

extern "C" void app_main(void)
{   
    // zapnuti siloveho napajeni do driveru*/

    if(gpio_get_level(SILOVKA)){
        gpio_set_level(VCC_IO, 1);
        gpio_set_level(ENN_PIN0, 0);
        gpio_set_level(ENN_PIN1, 0);
        gpio_set_level(ENN_PIN2, 0);
        gpio_set_level(ENN_PIN3, 0);
        printf("\n");
        printf("Zapnuti driveru!\n");
        printf("Oskar95 start \n\tbuild %s %s\n", __DATE__, __TIME__);
        printf("\n");
    }
    else{
        gpio_set_level(ENN_PIN0, 1);
        gpio_set_level(ENN_PIN1, 1);
        gpio_set_level(ENN_PIN2, 1);
        gpio_set_level(ENN_PIN3, 1);
        driver_stdby++;
        printf("\n");
        printf("Napajeni nepripojeno!\n");
        printf("Oskar95 start \n\tbuild %s %s\n", __DATE__, __TIME__);
        printf("\n");
    };
    
    check_reset();
    iopins_init();
    //optozávory inicializace + tlačítka
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_BIT_MASK_INPUTS,   //inicializuje piny 35,34,36,39,22,25
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    if(gpio_get_level(SILOVKA)){
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

    printf("\n");
    Driver driver0 { drivers_uart, DRIVER_0_ADDRES, ENN_PIN0};
    initDriver(driver0, 16, 32);
    printf("\n");
    Driver driver1 { drivers_uart, DRIVER_1_ADDRES, ENN_PIN1};
    initDriver(driver1, 16, 32); 
    driver1.set_speed(motor_speed1);
    printf("\n");
    Driver driver2 { drivers_uart, DRIVER_2_ADDRES, ENN_PIN2};
    initDriver(driver2, 16, 32);
    driver2.set_speed(motor_speed2);
    printf("\n");
    Driver driver3 { drivers_uart, DRIVER_3_ADDRES, ENN_PIN3};
    initDriver(driver3, 16, 32);
    driver3.set_speed(motor_speed3);
    printf("\n");
	
    pcnt_init_fan1();
    pcnt_init_fan2();
    pcnt_init_fan3();
    pcnt_init_fan4(); 

    TaskHandle_t  pulse_count;
    xTaskCreatePinnedToCore(pulse,"pulse counter",4096,NULL,tskIDLE_PRIORITY,&pulse_count,1); 
    
    while(1){
        if(gpio_get_level(SILOVKA) && driver_stdby){
            driver_stdby=0;
            printf("Připojeno 12V:\n");
            esp_restart();
        }
        else if(gpio_get_level(SILOVKA)==0 && driver_stdby==0){
            printf("Stand by:\n");
            esp_restart();
        }
        
        /*printf("KONCOVY_DOJEZD_0 %d\n", gpio_get_level(KONCOVY_DOJEZD_0));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_1 %d\n", gpio_get_level(KONCOVY_DOJEZD_1));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_2 %d\n", gpio_get_level(KONCOVY_DOJEZD_2));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_3 %d\n", gpio_get_level(KONCOVY_DOJEZD_3));*/
        
       
        int speed=-100000;

        if(!gpio_get_level(KONCOVY_DOJEZD_0) && position0==0){
            driver0.set_speed(speed);
        }
        else if(!gpio_get_level(KONCOVY_DOJEZD_0) && count0>=driver0_const){
            driver0.set_speed(speed);
            position0=driver0_const-position0;
            count0=0;
            motor0_cal=true;
        }
        else if(gpio_get_level(KONCOVY_DOJEZD_0) && position0==0){
            driver0.set_speed(speed*(-1));
            position0=count0;
            count0=0;
            //vTaskDelay(500/portTICK_PERIOD_MS);
        }
        else if(motor0_cal && (count0>=position0) && (position0!=driver0_const)){
            driver0.set_speed(0);
            printf("Motor0 zkalibrovan na pozici: %d\n",position0);
            motor0_cal=false;
            motor0_cal_done=true;
            count0=0;
        }
        //
        if(!gpio_get_level(KONCOVY_DOJEZD_1) && (position1==0) && motor0_cal_done){
            driver1.set_speed(speed*(-1));
        }
        else if(!gpio_get_level(KONCOVY_DOJEZD_1) && (count1>=driver1_const)){
            driver1.set_speed(speed*(-1));
            position1=driver1_const-position1;
            count1=0;
            motor1_cal=true;
        }
        else if(gpio_get_level(KONCOVY_DOJEZD_1) && (position1==0) && motor0_cal_done){
            driver1.set_speed(speed);
            position1=count1;
            count1=0;
            //vTaskDelay(500/portTICK_PERIOD_MS);
        }
        else if(motor1_cal && (count1>=position1) && (position1!=driver1_const)){
            driver1.set_speed(0);
            printf("Motor1 zkalibrovan na pozici: %d\n",position1);
            motor1_cal=false;
            motor1_cal_done=true;
            count1=0;
        }
        //
        if(!gpio_get_level(KONCOVY_DOJEZD_2) && (position2==0) && motor0_cal_done && motor1_cal_done){
            driver2.set_speed(speed);
            //vTaskDelay(5/portTICK_PERIOD_MS);
        }
        else if(!gpio_get_level(KONCOVY_DOJEZD_2) && (count2>=driver2_const)){
            driver2.set_speed(speed);
            //vTaskDelay(5/portTICK_PERIOD_MS);
            position2=driver2_const-position2;
            count2=0;
            motor2_cal=true;
        }
        else if(gpio_get_level(KONCOVY_DOJEZD_2) && (position2==0) && motor0_cal_done && motor1_cal_done){
            driver2.set_speed(speed*(-1));
            //vTaskDelay(5/portTICK_PERIOD_MS);
            position2=count2;
            count2=0;
        }
        else if(motor2_cal && (count2>=position2) && (position2!=driver2_const)){
            driver2.set_speed(0);
            //vTaskDelay(5/portTICK_PERIOD_MS);
            printf("Motor2 zkalibrovan na pozici: %d\n",position2);
            motor2_cal=false;
            motor2_cal_done=true;
            count2=0;
        }
        //
        if(!gpio_get_level(KONCOVY_DOJEZD_3) && (position3==0) && motor0_cal_done && motor1_cal_done && motor2_cal_done){
            driver3.set_speed(speed*(-1));
        }
        else if(!gpio_get_level(KONCOVY_DOJEZD_3) && count3>=driver3_const){
            driver3.set_speed(speed*(-1));
            position3=driver3_const-position3;
            count3=0;
            motor3_cal=true;
        }
        else if(gpio_get_level(KONCOVY_DOJEZD_3) && (position3==0) && motor0_cal_done && motor1_cal_done && motor2_cal_done){
            driver3.set_speed(speed);
            position3=count3;
            count3=0;
        }
        else if(motor3_cal && (count3>=position3) && (position3!=driver3_const)){
            driver3.set_speed(0);
            printf("Motor3 zkalibrovan na pozici: %d\n",position3);
            motor3_cal=false;
            motor3_cal_done=true;
            count3=0;
            break;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    while(1){
        bool done0=false;
        bool done1=false;
        bool done2=false;
        bool done3=false;
        //Zastaveni na koncovych dojezdech
        if(gpio_get_level(KONCOVY_DOJEZD_0)){
        driver0.set_speed(0);
        count0=0;
        done0=true;
       }
       else{
        driver0.set_speed(-71608);
       }
       if(gpio_get_level(KONCOVY_DOJEZD_1)){
        driver1.set_speed(0);
        count1=0;
        done1=true;
       }
       else{
        driver1.set_speed(71608);
       }
       if(gpio_get_level(KONCOVY_DOJEZD_2)){
        driver2.set_speed(0);
        count2=0;
        done2=true;
       }
       else{
        driver2.set_speed(-71608);
       }
       if(gpio_get_level(KONCOVY_DOJEZD_3)){
        driver3.set_speed(0);
        count3=0;
        done3=true;
       }
       else{
        driver3.set_speed(71608);
       }
       if(done0 && done1 && done2 && done3){
        break;
       }
       vTaskDelay(10/portTICK_PERIOD_MS);
    }    
    while(1){
        //function();

       // driver0.set_speed(driver_cal_speed());
    }





        //pulse();
       /* printf("pocet pulzu: %d\n",pcnt0_count);
        vTaskDelay(1000/portTICK_PERIOD_MS);*/
        //
      //  vTaskDelay(5/portTICK_PERIOD_MS);
       // printf("procesor: %d\n", xPortGetCoreID());
       
        
}


    
    