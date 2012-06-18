#ifndef _mytime_
#define _mytime_

#define NR_TIMER 10
// timer 0 is for local use
// wheel encoder use timer 1


long double   Get_mtime_diff(int timer_number);
unsigned long GetCurrentTimeInMilliSec(void);
void          mydelay (unsigned long msec);



#endif
