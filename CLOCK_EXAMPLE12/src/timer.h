/*******************************************************************************
*
* File Name    : timer.h
*
* Abstract     : Timer routines
*
* History      : jsi	1 June 2011
*			   : jsi    23 FEB 2016 modified for electroclave gen 2
*
*******************************************************************************/

#ifndef TIMER_H
#define TIMER_H

#include "asf.h"

#define MAX_TICK (0x3FFFFFFFL)

#define HOURS			(60*MINUTES)
#define MINUTES			(60*SECONDS)
#define SECONDS			TICKS_PER_SEC
#define TICKS_PER_SEC	1000

extern unsigned long	timerTickCount;

/*
 * Timers normally run for 1 sec or more
 */
#define TIMER_1ST_IDX		0
#define	TMR_DEBUG			0
#define	TMR_DISPLAY			1
#define	TMR_SANITIZE		2
#define	TMR_ONE_MINUTE		3
#define	TMR_CLEAN			4
#define	TMR_ERROR_DISPLAY	5
#define TMR_KEYPAD			6
#define TMR_DOOR_OPEN		7
#define TMR_DIRTY			8
#define TIMER_LAST_SEC_IDX	8

/*
 * Timers normally run for less than one second
 */
#define TIMER_1ST_SUBSEC_IDX	0
#define TIMER_LAST_IDX			8

#define NUM_TIMERS 				(TIMER_LAST_IDX+1)
#define NUM_SEC_TIMERS			(TIMER_LAST_SEC_IDX+1)
#define NUM_SUBSEC_TIMERS		(0)

typedef struct
{
	unsigned short	active;
	unsigned char 	done;
	unsigned char 	overflow;	
	unsigned long 	count;	

}TIMER;

extern TIMER timers[NUM_TIMERS];
extern unsigned char rollover;


#define SOLENOID_ON_COUNT	500		//ms
#define SOLENOID_OFF_COUNT	500		//ms
#define CYCLE_ON			1
#define CYCLE_OFF			0



typedef struct CONTROLS{

	uint8_t		psupply_onn;
	uint8_t		ledoen;
	uint8_t		MFP;
	uint8_t		buzzer_enable;
	uint8_t		buzzer_cycle;
	uint16_t	buzzer_dur_count;
	uint16_t	buzzer_repeat_count;
	uint16_t	buzzer_on_dur;
	uint16_t	buzzer_off_dur;
	uint8_t		buzzer_repeat;
	uint8_t		solenoid_enable;
	uint8_t		solenoid_cycle;
	uint16_t	solenoid_count;

};


void init_timer_vars(void);
void process_timers(void); 
void process_timer(unsigned short);
void start_timer(unsigned short, unsigned long);
void end_timer(unsigned short);
unsigned short timer_running(unsigned short);
unsigned short timer_done(unsigned short);

#endif