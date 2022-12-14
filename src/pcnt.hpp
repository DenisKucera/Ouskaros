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

    static void ledc_init()
{
    // Prepare and then apply the LEDC PWM timer configuration
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_8_BIT;
    ledc_timer.freq_hz          = 5120;  // set output frequency at 10000 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel0 = {
        .gpio_num   = LEDC_OUTPUT_IO0,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128, // set duty at about 50%
        .hpoint     = 0,
    };    
    ledc_channel_config(&ledc_channel0);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num   = LEDC_OUTPUT_IO1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128, // set duty at about 50
        .hpoint     = 0,
    };
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num   = LEDC_OUTPUT_IO2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128, // set duty at about 50%
        .hpoint     = 0,
    };    
    ledc_channel_config(&ledc_channel2);

    ledc_channel_config_t ledc_channel3 = {
        .gpio_num   = LEDC_OUTPUT_IO3,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128, // set duty at about 50%
        .hpoint     = 0,
    };    
    ledc_channel_config(&ledc_channel3);
}

static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
        pcnt_config_t pcnt_config;
        // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = PCNT_INPUT_0;
       // pcnt_config.pulse_gpio_num = PCNT_INPUT_1;
       // pcnt_config.pulse_gpio_num = PCNT_INPUT_2;
       // pcnt_config.pulse_gpio_num = PCNT_INPUT_3;
        //.ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        pcnt_config.channel = PCNT_CHANNEL_0;
        //pcnt_config.channel = PCNT_CHANNEL_1;
        pcnt_config.unit = PCNT_TEST_UNIT;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
        pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_DISABLE; // Reverse counting direction if low
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
        pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;
    
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    
    /* Configure and enable the input filter */
    //pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    //pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

void pulse(void*pvParameters)
{
    vTaskDelay(1/portTICK_PERIOD_MS);
    /* Initialize LEDC to generate sample pulse signal */
   // ledc_init();
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();

    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
           // printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
            if (evt.status & PCNT_STATUS_THRES1_M) {
                printf("THRES1 EVT\n");
            }
            if (evt.status & PCNT_STATUS_THRES0_M) {
                printf("THRES0 EVT\n");
            }
            if (evt.status & PCNT_STATUS_L_LIM_M) {
                printf("L_LIM EVT\n");
            }
            if (evt.status & PCNT_STATUS_H_LIM_M) {
             //   printf("H_LIM EVT\n");
                pcnt0_count++;
            }
            if (evt.status & PCNT_STATUS_ZERO_M) {
             //   printf("ZERO EVT\n");
            }
        } else {
            pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
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