/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
//usr include
#include "Smartconfig.h"
#include "Led.h"

#define NEEDTOUCH_Y 0X00
#define NEEDTOUCH_N 0X01

static const char *TAG = "sc";
wifi_config_t s_staconf;

uint8_t need_touch=NEEDTOUCH_Y;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        if(need_touch==NEEDTOUCH_Y)
        {
            Led_Status=LED_STA_TOUCH;
            xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
        }
        printf("wifi event start\n");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        printf("wifi conneced\n");
        Led_Status=LED_STA_INIT;
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        printf("wifi disconneced\n");
        Led_Status=LED_STA_WIFIERR;
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void re_touch(void)//用户按键重新配置wifi
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    need_touch=NEEDTOUCH_Y;
    ESP_ERROR_CHECK( esp_wifi_start() );

}

void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &s_staconf));//读取配置状态
    if (s_staconf.sta.ssid[0] != '\0')//当前有连过的wifi直接连接
    {
        need_touch=NEEDTOUCH_N;
        printf("already have wifi message! connect to ap SSID:%s password:%s\r\n",
               s_staconf.sta.ssid, s_staconf.sta.password);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &s_staconf));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_connect();
    }
    else//进入配网
    {
        need_touch=NEEDTOUCH_Y;
        ESP_ERROR_CHECK( esp_wifi_start() );
    }
}

static void sc_callback(smartconfig_status_t status, void *pdata)
{
    switch (status) {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAG, "SC_STATUS_WAIT");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAG, "SC_STATUS_FINDING_CHANNEL");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAG, "SC_STATUS_GETTING_SSID_PSWD");
            break;
        case SC_STATUS_LINK:
            ESP_LOGI(TAG, "SC_STATUS_LINK");
            wifi_config_t *wifi_config = pdata;
            ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
            ESP_LOGI(TAG, "PASSWORD:%s", wifi_config->sta.password);
            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config) );
            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
            if (pdata != NULL) {
                uint8_t phone_ip[4] = { 0 };
                memcpy(phone_ip, (uint8_t* )pdata, 4);
                ESP_LOGI(TAG, "Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
            }
            xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
            //xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
}

void smartconfig_example_task(void * parm)
{
    //EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    ESP_ERROR_CHECK( esp_smartconfig_start(sc_callback) );
    while (1) 
    {
       /* uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) 
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) 
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();

            vTaskDelete(NULL);
        }*/


        xEventGroupWaitBits(s_wifi_event_group, ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        ESP_LOGI(TAG, "smartconfig over");
        esp_smartconfig_stop();

        vTaskDelete(NULL);

    }
}


