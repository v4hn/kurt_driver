#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "mytime.h"

long double    mseconds;


unsigned long GetCurrentTimeInMilliSec(void)
{
  struct timeval tv;
  unsigned long milliseconds;
  gettimeofday(&tv, NULL);
  milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
  return milliseconds;
}

// Zeitmessung NR_TIMER -1 special since programm start
// timer 0 is for local use
long double Get_mtime_diff(int timer_number)
{
  static double   prev_clock[NR_TIMER];
  static int      first_time = 1;
  double          new_clock, clock_diff;
  int             i;

  if (first_time) {
    new_clock = GetCurrentTimeInMilliSec();
    for (i = 0; i < NR_TIMER; i++) {
      prev_clock[i] = new_clock;
    }
    clock_diff = new_clock;
    first_time = 0;
  }
  else {
    new_clock = GetCurrentTimeInMilliSec();
    clock_diff = new_clock - prev_clock[timer_number];
    if (timer_number != (NR_TIMER - 1)) prev_clock[timer_number] = new_clock;
  }

  return(clock_diff);
}

/*-------------------------------------------------------------------------*
 * NAME        : mydelay
 * DESCRIPTION
 *  warten und nichts tun
 *  achtung blockiert also nur waehrend der initialisierung verwenden
 *
 * PARAMETERS
 * RESULT
 *    --
 *-------------------------------------------------------------------------*/
void mydelay (unsigned long msec)
{
  unsigned long start_clock, cur_clock;

  start_clock = GetCurrentTimeInMilliSec();

  do  {
    cur_clock  = GetCurrentTimeInMilliSec();
  } while ((cur_clock - start_clock) < msec);

}
