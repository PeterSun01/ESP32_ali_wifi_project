#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "Human.h"
#include "Smartconfig.h"


static const char *TAG = "human";




static void Human_Task(void* arg)
{
    while(1)
    {
        Human_status=gpio_get_level(GPIO_HUMAN);
        printf("human =%d\n",Human_status);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}



void Human_Init(void)
{
    //配置GPIO，下降沿触发中断
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = 1 << GPIO_HUMAN;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


   xTaskCreate(Human_Task, "Human_Task", 4096, NULL, 5, NULL);


}


