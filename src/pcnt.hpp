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



extern "C"{

    xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
    pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

    /* A sample structure to pass events from the PCNT
    * interrupt handler to the main program.
    */
    typedef struct {
        int unit;  // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
    } pcnt_evt_t;

    /* Decode what PCNT's unit originated an interrupt
    * and pass this information together with the event type
    * the main program using a queue.
    */
    static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
    {
        uint32_t intr_status = PCNT.int_st.val;
        int i;
        pcnt_evt_t evt;
        portBASE_TYPE HPTaskAwoken = pdFALSE;

        for (i = 0; i < PCNT_UNIT_MAX; i++) {
            if (intr_status & (BIT(i))) {
                evt.unit = i;
                /* Save the PCNT event type that caused an interrupt
                to pass it to the main program */
                evt.status = PCNT.status_unit[i].val;
                PCNT.int_clr.val = BIT(i);
                xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
                if (HPTaskAwoken == pdTRUE) {
                    portYIELD_FROM_ISR();
                }
            }
        }
    }

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
}

 void index_pcnt(const pcnt_unit_t unit, const gpio_num_t pcnt_input, const gpio_num_t pcnt_ctrl){
    /* Prepare configuration for the PCNT unit */
        pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pcnt_input,
        .ctrl_gpio_num = pcnt_ctrl,
        .lctrl_mode = PCNT_MODE_DISABLE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
        };
    pcnt_unit_config(&pcnt_config);
    /* Initialize PCNT unit */
    /* Set threshold 0 and 1 values and enable events to watch */
  //  pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_TH//RES_1, PCNT_THRES//H1_VAL);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
 
    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
    }


void pulse(void*pvParameters)
{
    //vTaskDelay(1/portTICK_PERIOD_MS);
    /* Initialize LEDC to generate sample pulse signal */
   // ledc_init();
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    //pcnt_example_init();
    index_pcnt(PCNT_UNIT_0, PCNT_INPUT_0 , GPIO_NUM_0);
    index_pcnt(PCNT_UNIT_1, PCNT_INPUT_1 , GPIO_NUM_0);
    index_pcnt(PCNT_UNIT_2, PCNT_INPUT_2 , GPIO_NUM_0);
    index_pcnt(PCNT_UNIT_3, PCNT_INPUT_3 , GPIO_NUM_0);
    //int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
           // pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
           // printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
            if (evt.status & PCNT_STATUS_H_LIM_M) {
               // printf("H_LIM EVT\n");
                switch(evt.unit) {
                    case 0:
                        pcnt0_count++;
                        break;
                    case 1:
                        pcnt1_count++;
                        break;
                    case 2:
                        pcnt2_count++;
                        break;
                    case 3:
                        pcnt3_count++;
                        break;
                    default:
                        break;   
                }
            }
        } else {
            //pcnt_get_counter_value(unit, &count);
         //   printf("Current counter value :%d\n", count);
        }
     //   printf("procesor: %d\n", xPortGetCoreID());
    }
    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }    
}
}