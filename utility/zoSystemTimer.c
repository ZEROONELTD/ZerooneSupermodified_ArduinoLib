#include <avr/io.h>
#include <avr/interrupt.h>
#include "zoSystemTimer.h"
#include "zoMcu.h"


static volatile uint16_t Miliseconds = 0;
static volatile uint16_t WatchDogMiliseconds = 0;
static volatile uint16_t WatchDogTimeOut = 0xFFFF;
static volatile bool WatchDogResetEnabled = false;

void zoSystemTimerInit(void)
{
	//MCUSR &= ~_BV(WDRF);		//allow for WDE clear
	//WDTCSR |= _BV(WDCE);		//enable watchdog change
	WDTCSR = 0xD8;				//0b11011000:WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0 
								//configure interrupt and system reset mode  
	zoSystemTimerWatchDogDisable();
}

//watchdog functionality 
inline void zoSystemTimerWatchDogInit(const uint16_t timeOutMiliSecond)
{
	enterCritical();
	WatchDogTimeOut = timeOutMiliSecond;
	exitCritical();
}

inline void zoSystemTimerWatchDogEnable(void)
{
	enterCritical();
	WatchDogResetEnabled = true;
	exitCritical();
}

inline void zoSystemTimerWatchDogDisable(void)
{
	enterCritical();
	WatchDogResetEnabled = true;
	exitCritical();
}

inline void zoSystemTimerWatchDogReset(void)
{
	enterCritical();
	WatchDogMiliseconds = 0;
	exitCritical();
}

//timeout functionality 
inline void zoSystemTimerTimeOutInit(uint16_t *counter)
{
	zoSystemTimerMeasureStart(counter);
}

inline bool zoSystemTimerTimeOutExpired(uint16_t *counter, const uint16_t timeOutMiliSecond)
{
	return( ( zoSystemTimerMeasureGet(counter) >= timeOutMiliSecond ) ? true : false );
}

//time measuring functionality
//system measured in ms
inline void zoSystemTimerMeasureStart(uint16_t *counter)
{
	enterCritical();
	*counter = Miliseconds;
	exitCritical();
}

uint16_t zoSystemTimerMeasureGet(uint16_t *counter)
{
	uint16_t curr;
	
	enterCritical();
	curr= Miliseconds;	
	exitCritical();

	if(*counter <= curr)
		return (curr - *counter);
	else
		return (0xFFFF + curr -*counter); 
}

ISR(WDT_vect)
{
	Miliseconds+=16;
	WatchDogMiliseconds+=16;

	if( (WatchDogMiliseconds <= WatchDogTimeOut) || (!WatchDogResetEnabled) )
		WDTCSR |= _BV(WDCE)|_BV(WDIE);	//avoid reseting MCU					
}