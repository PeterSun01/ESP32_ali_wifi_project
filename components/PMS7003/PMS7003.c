#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "Led.h"
#include "PMS7003.h"


#define UART1_TXD  (UART_PIN_NO_CHANGE)
#define UART1_RXD  (GPIO_NUM_35)
#define UART1_RTS  (UART_PIN_NO_CHANGE)
#define UART1_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE    100


static const char *TAG = "PMS7003";

void PMS7003_Read_Task(void* arg);


void PMS7003_Init(void)
{
     //配置GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =  1ULL << UART1_RXD;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    xTaskCreate(&PMS7003_Read_Task, "PMS7003_Read_Task", 2048, NULL, 10, NULL);
}


static uint8_t	Check_PMSensor_DataValid(uint8_t* PM_Sensor_RxBuffer)
{
	uint16_t 	Cal_CheckSum;
	uint16_t 	Buffer_CheckSum;
	uint16_t 	Buffer_Len;
	uint8_t 	i;
	uint8_t     Result = 0;

	if((PM_Sensor_RxBuffer[0] == 'B')&&(PM_Sensor_RxBuffer[1] == 'M'))
	{
		Buffer_Len = (uint16_t)((PM_Sensor_RxBuffer[2] << 8) | PM_Sensor_RxBuffer[3]);

		Buffer_CheckSum = (uint16_t)((PM_Sensor_RxBuffer[Buffer_Len + 2] << 8) | PM_Sensor_RxBuffer[Buffer_Len + 3]);

		Cal_CheckSum = 0;
		for(i=0;i<(Buffer_Len + 2);i++)
		{
			Cal_CheckSum += PM_Sensor_RxBuffer[i];
		}
		if(Cal_CheckSum == Buffer_CheckSum)
			Result = 1;
	}
	return Result;
}


/*
PM2.5浓度与LED指示
优：0~100    			绿色
轻度污染：101~300 	     黄色	   
重度污染：301及以上       红色  
*/


void PMS7003_Read_Task(void* arg)
{
    uint8_t data_u1[BUF_SIZE];
    while(1)
    {
        int len1 = uart_read_bytes(UART_NUM_1, data_u1, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len1!=0)  //读取到传感器数据
        {
            len1=0;
            if(Check_PMSensor_DataValid(data_u1)==1)//数据校验成功
            {
                PM2_5  = 0;
                PM10   = 0;
                PM2_5  = (uint16_t)((data_u1[12]<<8) | data_u1[13]);
                PM10   = (uint16_t)((data_u1[14]<<8) | data_u1[15]);
                //ESP_LOGI(TAG, "PM2_5=%d,PM10=%d", PM2_5,PM10);
                if(PM2_5<=100)//优：0~100绿色
                {
                    Led_STA_G_On();
                }
                else if((PM2_5>100)&&(PM2_5<=300))//轻度污染：101~300 黄色	   
                {
                    Led_STA_Y_On();
                }
                else if(PM2_5>300)//重度污染：301及以上 红色  
                {
                    Led_STA_R_On();
                }
            }
            bzero(data_u1,sizeof(data_u1));                 
        }  
        vTaskDelay(5 / portTICK_RATE_MS);
    }   
}


