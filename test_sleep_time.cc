
/* Test of how accurate ArUtil::sleep() and ArTime end up being with respect to
 * each other in actual use.   Try this on different platforms to see what
 * happens.
 */

#include <stdio.h>
#include "ariaUtil.h"

int main(int argc, char **argv)
{
  ArTime t;
  for(long msec = 10; msec < 400; msec += 20)
  {
    for(int i = 0; i < 10; ++i)
    {
      t.setToNow();
      ArUtil::sleep(msec);
      printf("Sleeping for %ld msec (ArUtil::sleep) took %ld msec according to ArTime.mSecSince()\n", msec, t.mSecSince());
      t.setToNow();
      usleep(msec * 1000);
      printf("Sleeping for %ld msec (usleep) took %ld msec according to ArTime.mSecSince()\n", msec, t.mSecSince());
    }
  }
  return 0;
}
