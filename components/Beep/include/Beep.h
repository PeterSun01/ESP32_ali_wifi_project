
#ifndef _BEEP_H_
#define _BEEP_H_

#include "freertos/FreeRTOS.h"

extern void Beep_Init(void);
extern void Beep_On(void);
extern void Beep_Off(void);

#define GPIO_BEEP    5

uint8_t Beep_status;
#define BEEP_ON     0X01
#define BEEP_OFF    0X00

#define BEEP_ON_MQTT_SEND   "{\"switch1\":\"on\"}"
#define BEEP_OFF_MQTT_SEND  "{\"switch1\":\"off\"}"

#endif

