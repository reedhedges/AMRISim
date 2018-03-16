

#include "replace.h"


#if HAVE_SYS_TIMEB_H
#include <sys/timeb.h>
#else
#error gettimeofday replacement needs sys/timeb.h
#endif

/* Sets tv with seconds and microseconds, but microseconds only 
   has milisecond accuracy on windows. Timezone is not set 
   but it's deprecated and nobody uses it anymore. 
*/
int gettimeofday(struct timeval* tv, struct timezone* tz)
{
#if HAVE__FTIME
  struct _timeb timebuff;
  _ftime(&timebuff);
  tv->tv_sec =  timebuff.time;
  tv->tv_usec = (timebuff.millitm) * 1000.0;
  return 0;
#else
#error The replacement for gettimeofday() needs the windows _ftime function
#endif
}
