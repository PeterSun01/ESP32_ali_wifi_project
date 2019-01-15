#ifndef _LED_H_
#define _LED_H_


#include "freertos/FreeRTOS.h"

extern void Led_Init(void);

extern void Led_CTL_R_On(void);
extern void Led_CTL_B_On(void);
extern void Led_CTL_Off(void);

extern void Led_STA_R_On(void);
extern void Led_STA_G_On(void);
extern void Led_STA_Y_On(void);


uint8_t Led_Status;

#define LED_STA_INIT            0x00
#define LED_STA_TOUCH           0x01//配网
#define LED_STA_SENDDATA        0x02
#define LED_STA_WIFIERR         0x03
#define LED_STA_NOSER           0x04//无序列号
#define LED_STA_SENDDATAOVER    0x05
#define LED_STA_SENDDATAERR     0x06


#endif

