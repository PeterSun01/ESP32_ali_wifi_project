#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "Smartconfig.h"
#include "Nvs.h"
#include "Mqtt.h"
#include "Json_parse.h"
#include "Uart0.h"

#include "Led.h"
#include "E2prom.h"
#include "RtcUsr.h"
#include "Key.h"
#include "Beep.h"
#include "sht31.h"
#include "PMS7003.h"


extern const int CONNECTED_BIT;

void timer_periodic_cb(void *arg); 
esp_timer_handle_t timer_periodic_handle = 0; //定时器句柄


esp_timer_create_args_t timer_periodic_arg = {
    .callback =
        &timer_periodic_cb, 
    .arg = NULL,            
    .name = "PeriodicTimer" 
};

void timer_periodic_cb(void *arg) //1ms中断一次
{
  static int64_t timer_count = 0;
  timer_count++;
  if (timer_count >= 3000) //1s
  {
    timer_count = 0;
    printf("Free memory: %d bytes\n", esp_get_free_heap_size());

  }
}

static void Uart0_Task(void* arg)
{
    while(1)
    {
        Uart0_read();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }  
}



void app_main(void)
{

  ESP_ERROR_CHECK( nvs_flash_init() );
  ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
  Led_Init();
  i2c_init();
  Uart0_Init();
  key_Init();
  Beep_Init();
  PMS7003_Init();

  xTaskCreate(Uart0_Task, "Uart0_Task", 4096, NULL, 10, NULL);


  /*step1 判断是否有ProductKey/DeviceName/DeviceSecret****/
  E2prom_Read(PRODUCTKEY_ADDR,(uint8_t *)ProductKey,PRODUCTKEY_LEN);
  printf("ProductKey=%s\n", ProductKey);

  E2prom_Read(DEVICENAME_ADDR,(uint8_t *)DeviceName,DEVICENAME_LEN);
  printf("DeviceName=%s\n", DeviceName);

  E2prom_Read(DEVICESECRET_ADDR,(uint8_t *)DeviceSecret,DEVICESECRET_LEN);
  printf("DeviceSecret=%s\n", DeviceSecret); 


  if((strlen(DeviceName)==0)||(strlen(DeviceName)==0)||(strlen(DeviceSecret)==0)) //未获取到ProductKey/DeviceName/DeviceSecret，未烧写序列号
  {
    printf("no ProductKey/DeviceName/DeviceSecret!\n");
    while(1)
    {
      //故障灯闪烁
      Led_Status=LED_STA_NOSER;
      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }

  /*step3 创建WIFI连接****/
  initialise_wifi();
  //阻塞等待wifi连接
  xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT , false, true, portMAX_DELAY); 
  

  /*******************************timer 1s init**********************************************/
  esp_err_t err = esp_timer_create(&timer_periodic_arg, &timer_periodic_handle);
  err = esp_timer_start_periodic(timer_periodic_handle, 1000); //创建定时器，单位us，定时1ms
  if (err != ESP_OK)
  {
    printf("timer periodic create err code:%d\n", err);
  }

  initialise_mqtt();

}
