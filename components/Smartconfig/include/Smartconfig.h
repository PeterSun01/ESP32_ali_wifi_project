#ifndef _SMART_H_
#define _SMART_H_

#include "freertos/event_groups.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;




void smartconfig_example_task(void * parm);
void initialise_wifi(void);
void re_touch(void);



#define WIFISTATUS_CONNET       0X01
#define WIFISTATUS_DISCONNET    0X00
uint8_t WifiStatus;


#endif