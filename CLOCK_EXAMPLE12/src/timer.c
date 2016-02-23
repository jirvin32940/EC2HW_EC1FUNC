/***********************************************************************/
/*                                                                     */
/*  FILE        :timer.c                                               */
/*  DATE        :15 November 2011                                      */
/*  DESCRIPTION :Interrupt routines                                    */
/*                                                                     */
/*  																   */
/*  Copyright (c) 2011 Lighting Science Group Corporation.             */
/*  All rights reserved.											   */
/***********************************************************************/

#include "timer.h"

TIMER timers[NUM_TIMERS];

unsigned long timerTickCount = 0;
unsigned char rollover = 0;
unsigned char rollover_subsec = 0;



void init_timer_vars(void)
{
	unsigned short i;

	for (i=0; i<NUM_TIMERS; i++)
	{
		timers[i].count = 0;
		timers[i].active = 0;
		timers[i].done = 0;
		timers[i].overflow = 0;
	}
}

void start_timer(unsigned short timerID, unsigned long duration)
{
	timers[timerID].count = ((timerTickCount+duration) & MAX_TICK);
	timers[timerID].active = 1;
	timers[timerID].done = 0;
	if (duration <= (MAX_TICK - timerTickCount))
	{
		timers[timerID].overflow = 0;
	}
	else
	{
		timers[timerID].overflow = 1;
	}
}

void process_timers(void)
{
	unsigned short i;

	for (i=0; i<NUM_SEC_TIMERS; i++)
	{
		if (timers[i].active)
		{
			if (rollover) 
			{
				timers[i].overflow = 0;
			}
			
			if (!timers[i].overflow) 	/* don't check for timer count done if the timer count overflowed when it was set  */
			{							/* but timerTickCount hasn't rolled over yet */
				if (timerTickCount > timers[i].count)
				{
					timers[i].active = 0;
						timers[i].done = 1;
				}
			}	
		}
	}
	
	if (rollover)
	{
		rollover = 0; /* set in the interrupt, cleared here after we use it */
	}
}

void process_subsec_timers(void)
{
	unsigned short i;

	for (i=TIMER_1ST_SUBSEC_IDX; i<(TIMER_LAST_IDX+1); i++)
	{
		if (timers[i].active)
		{
			if (rollover_subsec) 
			{
				timers[i].overflow = 0;
			}
			
			if (!timers[i].overflow)		/* don't check for timer count done if the timer count overflowed when it was set  */
			{									/* but timerTickCount hasn't rolled over yet */
				if (timerTickCount > timers[i].count)
				{
					timers[i].active = 0;
						timers[i].done = 1;
				}
			}	
		}
	}
	
	if (rollover_subsec)
	{
		rollover_subsec = 0; /* set in the interrupt, cleared here after we use it */
	}
}

/* 
 * This function is for updating just one timer...useful in situations where you don't want to spend the clock cycles
 * to process all the timers but you need a particular timer updated because it's duration is less than 1.5ms...
 * Note that this doesn't deal with timer tick rollover...on the rare occasion that the timer tick does roll over, a timer
 * processed using this function won't update...this is a deficiency, but it will happen rarely enough that the 
 * complexity required to deal with the rollover for a single timer is not worth managing. 
 */
void process_timer(unsigned short timerID)
{
	if (timers[timerID].active)
	{
		if (!timers[timerID].overflow)
		{
			if (timerTickCount > timers[timerID].count)
			{
				timers[timerID].active = 0;
				timers[timerID].done = 1;
			}
		}
	}
}

unsigned short timer_running(unsigned short timerID)
{
	return (timers[timerID].active);
}

unsigned short timer_done(unsigned short timerID)
{
	if (timers[timerID].done)
	{
		timers[timerID].done = 0;
		return 1;
	}
	else
		return 0;
}

void end_timer(unsigned short timerID)
{
	timers[timerID].count = 0;
	timers[timerID].active = 0;
	timers[timerID].done = 0;
	timers[timerID].overflow = 0;
}

