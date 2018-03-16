
#include "replace.h"
#include <stdlib.h>


/* A *much* less accurate version of usleep. It may only have accuracy of a few hundred ms.  */
int usleep(unsigned long usec)
{
#ifdef HAVE_WIN32_SLEEP
#       warning using Win32 Sleep() function.
	Sleep(usec/1000.0);
#else
#   ifdef HAVE__SLEEP
#       warning Using private Win32 _sleep function.
	_sleep(usec / 1000.0);
#   else
#      error the usleep() replacement requires the Windows _sleep() or Sleep() function!
#   endif
#endif
}


