#ifndef ZO_SYSTEM_TIMER_H
#define ZO_SYSTEM_TIMER_H

#include <stdbool.h>

//watchdog timer is used to give an ISR every ~16ms
//this is the system timer resolution
//this isr is used for system timing functionality
//watchdog reset functionality is maintained
void zoSystemTimerInit(void);

//watchdog functionality 
void zoSystemTimerWatchDogInit(const uint16_t timeOutMiliSecond);
void zoSystemTimerWatchDogEnable(void);
void zoSystemTimerWatchDogDisable(void);
void zoSystemTimerWatchDogReset(void);

//timeout functionality 
void zoSystemTimerTimeOutInit(uint16_t *counter);
bool zoSystemTimerTimeOutExpired(uint16_t *counter, const uint16_t timeOutMiliSecond);

//time measuring functionality
//system measured in ms
void zoSystemTimerMeasureStart(uint16_t *counter);
uint16_t zoSystemTimerMeasureGet(uint16_t *counter);

#endif //ZO_SYSTEM_TIMER_H