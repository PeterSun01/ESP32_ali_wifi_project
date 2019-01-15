#ifndef _MQTT_H_
#define _MQTT_H_

//#define TOPIC_POST  "/sys/a148brGI1Gw/A0001AIR1/thing/event/property/post"
//#define Topic_Set  "/sys/a148brGI1Gw/A0001AIR1/thing/service/property/set"

extern void initialise_mqtt(void);
void Mqtt_Send_Msg(char* topic);

#endif