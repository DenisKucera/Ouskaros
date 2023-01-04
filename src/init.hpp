#pragma once
#include <stdio.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include <iostream>
#include "defines.hpp"

#define TAG "spiffs"
using namespace std;

void iopins_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

int * spiffs(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);
    
    /* read data from hello.txt file */
    ESP_LOGE(TAG, "Reading data from file: instruction.txt");
    FILE *file = fopen("/spiffs/instruction.txt", "r");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "File does not exist!");
    }
    else
    {
        int i=0;
        char line[256];
        static int x[256];
        while (fgets(line, sizeof(line), file) != NULL)
        {
            x[i] = int(strtol(line, NULL, 10));
            if(q<i){
                q=i;
            }
            i++;
        }
        x[q+1]=q+1;
        fclose(file);
        return x;
    }
    esp_vfs_spiffs_unregister(NULL);
    return 0;
}

void nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was trun_0cated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}
