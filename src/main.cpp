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

    uint32_t position;
    driver.get_MSCNT(position);
    printf("POCATECNI POZICE MOTORU %d\n", position);

    int result = driver.get_PWMCONF(data);
    if (result != 0)
        printf("PWMCONF driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("PWMCONF driveru %d =  %08X\n", driver.address(), data);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    result = driver.get_DRV_STATUS(data);
    if (result != 0)
        printf("DRV_STATUS driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("DRV_STATUS driveru  %d =  %08X\n", driver.address(), data);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    driver.set_speed(0);                      // otáčení motoru se nastavuje zápisem rychlosti do driveru přes Uart
    driver.set_IHOLD_IRUN (16, 32);             // proud IHOLD (při stání) =8/32, IRUN (při běhu)= 8/32 (8/32 je minimum, 16/32 je maximum pro dluhodobější provoz)
    driver.enable();                          //zapnutí mptoru
    vTaskDelay(300 / portTICK_PERIOD_MS);     //doba stání pro nastavení automatiky driveru
    driver.set_IHOLD_IRUN (iRun, iHold);             //proud IHOLD =0, IRUN = 8/32 (při stání je motor volně otočný)
}


extern "C" void app_main(void)
{   
    gpio_set_level(VCC_IO_0, 1); // zapnuti napajeni do driveru0 
    gpio_set_level(VCC_IO_1, 1); // zapnuti napajeni do driveru1
    gpio_set_level(VCC_IO_2, 1); // zapnuti napajeni do driveru2
    gpio_set_level(VCC_IO_3, 1); // zapnuti napajeni do driveru3
    gpio_set_level(GPIO_NUM_32, 1);// zapnuti siloveho napajeni do driveru
    printf("Zapnuti driveru\n");
    printf("Simple Motor \n\tbuild %s %s\n", __DATE__, __TIME__);
    check_reset();
    iopins_init();
    //optozávory inicializace
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_BIT_MASK_INPUTS;   //inicializuje piny 35,34,36,39
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(VCC_IO_0, 0);              // reset driveru
    gpio_set_level(VCC_IO_1, 0);
    gpio_set_level(VCC_IO_2, 0);
    gpio_set_level(VCC_IO_3, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(VCC_IO_0, 1);              //zapíná VCCIO driveru
    gpio_set_level(VCC_IO_1, 1);
    gpio_set_level(VCC_IO_2, 1);
    gpio_set_level(VCC_IO_3, 1);
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
 
    Driver driver0 { drivers_uart, DRIVER_0_ADDRES, DRIVER_0_ENABLE };
    initDriver(driver0, 32, 32);
    driver0.get_MSCNT(position0);
    driver0.set_speed(motor_speed0);
    vTaskDelay(200/portTICK_PERIOD_MS);
    Driver driver1 { drivers_uart, DRIVER_1_ADDRES, DRIVER_1_ENABLE };
    initDriver(driver1, 32, 32); 
    driver1.set_speed(motor_speed1);
    driver1.get_MSCNT(position1);
    vTaskDelay(200/portTICK_PERIOD_MS);
    Driver driver2 { drivers_uart, DRIVER_2_ADDRES, DRIVER_2_ENABLE };
    initDriver(driver2, 32, 32);
    driver2.set_speed(motor_speed2);
    driver2.get_MSCNT(position2);
    vTaskDelay(200/portTICK_PERIOD_MS);
    Driver driver3 { drivers_uart, DRIVER_3_ADDRES, DRIVER_3_ENABLE };
    initDriver(driver3, 32, 32);
    driver3.get_MSCNT(position3);
    driver3.set_speed(motor_speed3);
    vTaskDelay(200/portTICK_PERIOD_MS);
    
    ledc_init(); //zapnuti STEP pulzů
    gpio_set_level(DIR_OUTPUT0, 1);  //DIR
    //pcnt();
	/*
	bool otevrena_celist = 0;
     
        if (gpio_get_level(KONCOVY_DOJEZD_0)*motor_speed2!=0)   //kontroluje optozavory (aktivovana = 1, deaktivovana = 0)
        {
            double xmotor_speed2 = 0;
            for (double i = 0.25; i <= 1; i+=0.25)  //najezdova rampa
            {
            xmotor_speed2 = -motor_speed2*i;           
            driver2.set_speed(xmotor_speed2);
            if(gpio_get_level(KONCOVY_DOJEZD_3) || gpio_get_level(KONCOVY_DOJEZD_1) || gpio_get_level(KONCOVY_DOJEZD_2)){
                break;
            }
            vTaskDelay(500/portTICK_PERIOD_MS);
            printf("Hodnota0 %f\n", i);
            }
            vTaskDelay(250/portTICK_PERIOD_MS);
            motor_speed2 = -motor_speed2;
        }
        if (gpio_get_level(KONCOVY_DOJEZD_1)*motor_speed3!=0)  //kontroluje optozavory (aktivovana = 1, deaktivovana = 0)
        {
            double xmotor_speed3 = 0;
            for (double i = 0.25; i <= 1; i+=0.25)  //najezdova rampa
            {
            xmotor_speed3 = -motor_speed3*i;           
            driver3.set_speed(xmotor_speed3);
            if(gpio_get_level(KONCOVY_DOJEZD_3) || gpio_get_level(KONCOVY_DOJEZD_0) || gpio_get_level(KONCOVY_DOJEZD_2)){
                break;
            }
            vTaskDelay(500/portTICK_PERIOD_MS);
            printf("Hodnota1 %f\n", i);
            }
            vTaskDelay(250/portTICK_PERIOD_MS);
            motor_speed3 = -motor_speed3;
        }        
        if (motor_speed1*gpio_get_level(KONCOVY_DOJEZD_2)!=0)  //kontroluje optozavory (aktivovana = 1, deaktivovana = 0)
        {
            double xmotor_speed1 = 0;
            for (double i = 0.25; i <= 1; i+=0.25)  //najezdova rampa
            { 
            xmotor_speed1 = -motor_speed1*i;           
            driver1.set_speed(xmotor_speed1);
            if(gpio_get_level(KONCOVY_DOJEZD_3) || gpio_get_level(KONCOVY_DOJEZD_1) || gpio_get_level(KONCOVY_DOJEZD_0)){
                break;
            }
            vTaskDelay(500/portTICK_PERIOD_MS);
            printf("Hodnota2 %f\n", i);
            }
            vTaskDelay(250/portTICK_PERIOD_MS);
            motor_speed1 = -motor_speed1;
        }
        
        if ( gpio_get_level(KONCOVY_DOJEZD_3) && !otevrena_celist )  //kontroluje optozavory (aktivovana = 1, deaktivovana = 0)
        {
            motor_speed0=0;
            driver0.set_speed(motor_speed0);
            otevrena_celist = 1;
            
            //double xmotor_speed0 = 0;
            //for (double i = 0.25; i <= 1; i+=0.25)  //najezdova rampa
            //{
            //xmotor_speed0 = -motor_speed0*i;           
            //driver0.set_speed(xmotor_speed0);
            //if(gpio_get_level(KONCOVY_DOJEZD_0) || gpio_get_level(KONCOVY_DOJEZD_1) || gpio_get_level(KONCOVY_DOJEZD_2)){
            //    break;
            //}
            //vTaskDelay(500/portTICK_PERIOD_MS); 
            //printf("Hodnota3 %f\n", i);
            //}
            //vTaskDelay(250/portTICK_PERIOD_MS);
            //motor_speed0 = -motor_speed0;  //zapise puvodni hodnotu motor_speed
        }
		if(!gpio_get_level(KONCOVY_DOJEZD_3))
		{
			otevrena_celist = 0;
		}
        vTaskDelay(5/portTICK_PERIOD_MS);  */
    TaskHandle_t  pulse_count;
    xTaskCreatePinnedToCore(pulse,"pulse counter",10000,NULL,1,&pulse_count,1); 
    vTaskDelay(1/portTICK_PERIOD_MS);  
    while(1){
        printf("KONCOVY_DOJEZD_0 %d\n", gpio_get_level(KONCOVY_DOJEZD_0));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_1 %d\n", gpio_get_level(KONCOVY_DOJEZD_1));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_2 %d\n", gpio_get_level(KONCOVY_DOJEZD_2));
        vTaskDelay(5/portTICK_PERIOD_MS);
        printf("KONCOVY_DOJEZD_3 %d\n", gpio_get_level(KONCOVY_DOJEZD_3));
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver0.set_speed(motor_speed0);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver1.set_speed(motor_speed1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver2.set_speed(motor_speed2);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver3.set_speed(motor_speed3);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver0.get_MSCNT(position0);
        printf("POZICE MOTORU0 %d\n", position0);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver1.get_MSCNT(position1);
        printf("POZICE MOTORU1 %d\n", position1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver2.get_MSCNT(position2);
        printf("POZICE MOTORU2 %d\n", position2);
        vTaskDelay(5/portTICK_PERIOD_MS);
        driver3.get_MSCNT(position3);
        printf("POZICE MOTORU3 %d\n", position3);
        vTaskDelay(5/portTICK_PERIOD_MS);
      /* pcnt();
        printf("pocet pulzu: %d\n",pcnt0_count);
        vTaskDelay(1000/portTICK_PERIOD_MS);*/
        printf("pocet pulzu: %d\n",pcnt0_count);
        vTaskDelay(5/portTICK_PERIOD_MS);
       // printf("procesor: %d\n", xPortGetCoreID());
        if(gpio_get_level(KONCOVY_DOJEZD_0)){
            ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}


    
    