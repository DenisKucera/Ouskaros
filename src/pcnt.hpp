#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "soc/pcnt_reg.h"
#include "soc/pcnt_struct.h"
#include "driver/periph_ctrl.h"
#include "defines.hpp"

#include "driver/pcnt.h"                                                
#include "soc/pcnt_struct.h"

extern "C" {

// ------------------------------------------------------------

bool            flag_counter0          = true;                                                                                                                                            
volatile uint16_t frequency_counter0     = 0;
uint16_t result_counter0 = 0;

bool            flag_counter1          = true;                                                                                                                                         
volatile uint16_t frequency_counter1     = 0;
uint16_t result_counter1 = 0;

bool            flag_counter2          = true;                                                                                                                                              
volatile uint16_t frequency_counter2     = 0;
uint16_t result_counter2 = 0;

bool            flag_counter3          = true;                                                                                                                                             
volatile uint16_t frequency_counter3    = 0;
uint16_t result_counter3 = 0;

//volatile double average_fans = 0;

// ------------------------------------------------------------

esp_timer_create_args_t timer_args_counter0;                                   
esp_timer_handle_t timer_handle_counter0;                                       
portMUX_TYPE timer_mux_counter0 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_counter1;                                    
esp_timer_handle_t timer_handle_counter1;                                          
portMUX_TYPE timer_mux_counter1 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_counter2;                                    
esp_timer_handle_t timer_handle_counter2;                                          
portMUX_TYPE timer_mux_counter2 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_counter3;                                    
esp_timer_handle_t timer_handle_counter3;                                          
portMUX_TYPE timer_mux_counter3 = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------------------

pcnt_config_t pcnt_config_counter0 = {
  .pulse_gpio_num    = PCNT_INPUT_0,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_DIS,
  .counter_h_lim     = 1000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_0, 
  .channel           = PCNT_CHANNEL_0
  };

  pcnt_config_t pcnt_config_counter1 = 
  {
  .pulse_gpio_num    = PCNT_INPUT_1,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_DIS,
  .counter_h_lim     = 1000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_1, 
  .channel           = PCNT_CHANNEL_0
  };

  pcnt_config_t pcnt_config_counter2 = 
  {
  .pulse_gpio_num    = PCNT_INPUT_2,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_DIS,
  .counter_h_lim     = 1000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_2, 
  .channel           = PCNT_CHANNEL_0
  };

  pcnt_config_t pcnt_config_counter3 = 
  {
  .pulse_gpio_num    = PCNT_INPUT_3,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_DIS,
  .counter_h_lim     = 1000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_3, 
  .channel           = PCNT_CHANNEL_0
  };

// ------------------------------------------------------------

void IRAM_ATTR pcnt_event_handler_counter0(void *arg)
{
  portENTER_CRITICAL_ISR(&timer_mux_counter0);
  PCNT.int_clr.val = BIT(PCNT_UNIT_0);
  portEXIT_CRITICAL_ISR(&timer_mux_counter0);
}     

void IRAM_ATTR pcnt_event_handler_counter1(void *arg) 
{
  portENTER_CRITICAL_ISR(&timer_mux_counter1); 
  PCNT.int_clr.val = BIT(PCNT_UNIT_1); 
  portEXIT_CRITICAL_ISR(&timer_mux_counter1); 
}  

void IRAM_ATTR pcnt_event_handler_counter2(void *arg)
{
  portENTER_CRITICAL_ISR(&timer_mux_counter2); 
  PCNT.int_clr.val = BIT(PCNT_UNIT_2); 
  portEXIT_CRITICAL_ISR(&timer_mux_counter2);
}

void IRAM_ATTR pcnt_event_handler_counter3(void *arg)
{
  portENTER_CRITICAL_ISR(&timer_mux_counter3); 
  PCNT.int_clr.val = BIT(PCNT_UNIT_3); 
  portEXIT_CRITICAL_ISR(&timer_mux_counter3);
}

void pcnt_get_counter_counter0(void *p) 
{ 
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*) &result_counter0); 
  flag_counter0 = true;
}

void pcnt_get_counter_counter1(void *p) 
{ 
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_get_counter_value(PCNT_UNIT_1, (int16_t*) &result_counter1); 
  flag_counter1 = true;
}

void pcnt_get_counter_counter2(void *p) 
{ 
  pcnt_counter_pause(PCNT_UNIT_2);
  pcnt_get_counter_value(PCNT_UNIT_2, (int16_t*) &result_counter2); 
  flag_counter2 = true;
}

void pcnt_get_counter_counter3(void *p) 
{ 
  pcnt_counter_pause(PCNT_UNIT_3);
  pcnt_get_counter_value(PCNT_UNIT_3, (int16_t*) &result_counter3); 
  flag_counter3 = true;
}

// ------------------------------------------------------------ 

void pcnt_init_counter0(void)                                                     
{  

  pcnt_unit_config(&pcnt_config_counter0);
  pcnt_isr_register(pcnt_event_handler_counter0, NULL, 0, NULL);                   
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0); 
  pcnt_counter_pause(PCNT_UNIT_0);                                       
  pcnt_counter_clear(PCNT_UNIT_0);                                       
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);                        
  pcnt_counter_resume(PCNT_UNIT_0);                                       

  timer_args_counter0.callback = pcnt_get_counter_counter0;
  timer_args_counter0.arg      = NULL;
  timer_args_counter0.name     = "one shot timer";

  /*if(esp_timer_create(&timer_args_fan1, &timer_handle_fan1) != ESP_OK) 
  {
    ESP_LOGE(TAG,"timer create");
  }*/

  timer_args_counter0.callback = pcnt_get_counter_counter0; 
  esp_timer_create(&timer_args_counter0, &timer_handle_counter0);                           
}

void pcnt_init_counter1(void)                                                     
{ 

  pcnt_unit_config(&pcnt_config_counter1);
  pcnt_isr_register(pcnt_event_handler_counter1, NULL, 0, NULL);               
  pcnt_intr_enable(PCNT_UNIT_1);  
  pcnt_set_filter_value(PCNT_UNIT_1, 1000);
  pcnt_filter_enable(PCNT_UNIT_1); 
  pcnt_counter_pause(PCNT_UNIT_1);                                      
  pcnt_counter_clear(PCNT_UNIT_1);                                      
  pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);                        
  pcnt_counter_resume(PCNT_UNIT_1);                                    

  timer_args_counter1.callback = pcnt_get_counter_counter1;
  timer_args_counter1.arg      = NULL;
  timer_args_counter1.name     = "one shot timer";

  timer_args_counter1.callback = pcnt_get_counter_counter1;
  esp_timer_create(&timer_args_counter1, &timer_handle_counter1); 
}

void pcnt_init_counter2(void)                                                     
{  

  pcnt_unit_config(&pcnt_config_counter2);
  pcnt_isr_register(pcnt_event_handler_counter2, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_UNIT_2);  
  pcnt_set_filter_value(PCNT_UNIT_2, 1000);
  pcnt_filter_enable(PCNT_UNIT_2); 
  pcnt_counter_pause(PCNT_UNIT_2);                                       
  pcnt_counter_clear(PCNT_UNIT_2);                                        
  pcnt_event_enable(PCNT_UNIT_2, PCNT_EVT_H_LIM);                        
  pcnt_counter_resume(PCNT_UNIT_2);                                       

  timer_args_counter2.callback = pcnt_get_counter_counter2;
  timer_args_counter2.arg      = NULL;
  timer_args_counter2.name     = "one shot timer";

  timer_args_counter2.callback = pcnt_get_counter_counter2;
  esp_timer_create(&timer_args_counter2, &timer_handle_counter2); 
}

void pcnt_init_counter3(void)                                                     
{  

  pcnt_unit_config(&pcnt_config_counter3);
  pcnt_isr_register(pcnt_event_handler_counter3, NULL, 0, NULL);
  pcnt_intr_enable(PCNT_UNIT_3);  
  pcnt_set_filter_value(PCNT_UNIT_3, 1000);
  pcnt_filter_enable(PCNT_UNIT_3); 
  pcnt_counter_pause(PCNT_UNIT_3);                                       
  pcnt_counter_clear(PCNT_UNIT_3);                                        
  pcnt_event_enable(PCNT_UNIT_3, PCNT_EVT_H_LIM);                        
  pcnt_counter_resume(PCNT_UNIT_3);                                       

  timer_args_counter3.callback = pcnt_get_counter_counter3;
  timer_args_counter3.arg      = NULL;
  timer_args_counter3.name     = "one shot timer";

  timer_args_counter3.callback = pcnt_get_counter_counter3;
  esp_timer_create(&timer_args_counter3, &timer_handle_counter3); 
}

// ------------------------------------------------------------

void pulse(void*pvParameters){
   while(1){ 

    if (flag_counter0 == true)
  {
    flag_counter0 = false;
    frequency_counter0 =  result_counter0 /*+ (overflow_cnt_fan1*20000)*/; 
    pcnt_counter_clear(PCNT_UNIT_0); 
    pcnt_counter_resume(PCNT_UNIT_0); 
    count0=frequency_counter0+count0;
    esp_timer_start_once(timer_handle_counter0, 1000);
    pcnt_counter_clear(PCNT_UNIT_0);
  }

   /*else if(flag_fan1 == true && gpio_get_level(DIR_OUTPUT0)==0){
    flag_fan1 = false;
    frequency_fan1 =  result_fan1; 
    pcnt_counter_clear(PCNT_UNIT_0); 
    pcnt_counter_resume(PCNT_UNIT_0);    
    //printf("RPM FAN1: %d\n",frequency_fan1);
    count0=count0-frequency_fan1;
    esp_timer_start_once(timer_handle_fan1, 1000000);
    pcnt_counter_clear(PCNT_UNIT_0);
   }*/

  if (flag_counter1 == true)
  {
    flag_counter1 = false;
    frequency_counter1 =  result_counter1 /*+ (overflow_cnt_fan2 * 20000)*/; 
    pcnt_counter_clear(PCNT_UNIT_1); 
    pcnt_counter_resume(PCNT_UNIT_1); 
   count1=frequency_counter1+count1;
    esp_timer_start_once(timer_handle_counter1, 1000);
    pcnt_counter_clear(PCNT_UNIT_1);
  }
   /*else if(flag_fan2 == true && gpio_get_level(DIR_OUTPUT1)==0){flag_fan2 = false;
    frequency_fan2 =  result_fan2; 
    pcnt_counter_clear(PCNT_UNIT_1); 
    pcnt_counter_resume(PCNT_UNIT_1);     
   //printf("RPM FAN2: %d\n",frequency_fan2);
   count1=count1-frequency_fan2;
    esp_timer_start_once(timer_handle_fan2, 1000000);
    pcnt_counter_clear(PCNT_UNIT_1);
    
   }*/

  if (flag_counter2 == true)
  {
    flag_counter2 = false;
    frequency_counter2 =  result_counter2 /*+ (overflow_cnt_fan3 * 20000)*/; 
    pcnt_counter_clear(PCNT_UNIT_2); 
    pcnt_counter_resume(PCNT_UNIT_2); 
    count2=frequency_counter2+count2;
    esp_timer_start_once(timer_handle_counter2, 1000);
    pcnt_counter_clear(PCNT_UNIT_2);
  }
   /*else if(flag_fan3 == true && gpio_get_level(DIR_OUTPUT2)==0){
    flag_fan3 = false;
    frequency_fan3 =  result_fan3; 
    pcnt_counter_clear(PCNT_UNIT_2); 
    pcnt_counter_resume(PCNT_UNIT_2);    
    //printf("RPM FAN3: %d\n",frequency_fan3);
    count2=count2-frequency_fan3;
    esp_timer_start_once(timer_handle_fan3, 1000000);
    pcnt_counter_clear(PCNT_UNIT_2);
   }*/
    

  if (flag_counter3 == true)
  {
    flag_counter3 = false;
    frequency_counter3 =  result_counter3 /*+ (overflow_cnt_fan4 * 20000)*/; 
    pcnt_counter_clear(PCNT_UNIT_3); 
    pcnt_counter_resume(PCNT_UNIT_3); 
    count3=frequency_counter3+count3;
    esp_timer_start_once(timer_handle_counter3, 1000);
    pcnt_counter_clear(PCNT_UNIT_3);
  }
/*else if(flag_fan4 == true && gpio_get_level(DIR_OUTPUT3)==0){
    flag_fan4 = false;
    frequency_fan4 =  result_fan4; 
    pcnt_counter_clear(PCNT_UNIT_3); 
    pcnt_counter_resume(PCNT_UNIT_3);    
    //printf("RPM FAN4: %d\n",frequency_fan4);
    count3=count3-frequency_fan4;
    esp_timer_start_once(timer_handle_fan4, 1000000);
    pcnt_counter_clear(PCNT_UNIT_3);
}*/
   }
}
/*
void step_pulse_init(const uint32_t freq_hz, const ledc_timer_t timer_num, const gpio_num_t ledc_output, const ledc_channel_t channel)
{
    // Prepare and then apply the LEDC PWM timer configuration
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = timer_num,
        .freq_hz          = freq_hz,  // set output frequency at 10000 Hz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = ledc_output,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = timer_num,
        .duty       = 128, // set duty at about 50%
        .hpoint     = 0,
    };    
    ledc_channel_config(&ledc_channel);
}*/
}