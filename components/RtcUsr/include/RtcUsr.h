
#ifndef _RTCUSR_H_
#define _RTCUSR_H_

extern void Rtc_Set(int year,int mon,int day,int hour,int min,int sec);
extern void Rtc_Read(int* year,int* month,int* day,int* hour,int* min,int* sec);


#endif

