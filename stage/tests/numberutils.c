
#include <stdio.h>
#include "stage.h"

int main(int argc, char **argv)
{
  double min = -10.0;
  double max = 10.0;
  double min2 = -20.0;
  double err = 100;
  double r;
  puts("This program generates random numbers using Stage's utility macros, then verifies that the result is within the correct range. It runs forever, unless there is an error. It also prints when the edge of a range is ever achieved, so if you ever see a message of that, then you know that works.");
  while(1)
  {
    r = STG_RANDOM_RANGE(min,max);
    assert(r >= min && r <= max);
    printf("within range (%f,%f): %f\n", min, max, r);

    r = STG_RANDOM_RANGE(min2, min);
    assert(r >= min2 && r <= min);
    printf("within range (%f,%f): %f\n", min2, min, r);

    r = STG_RANDOM_ERR(err);
    assert( r >  -(err/2.0) && r < (err/2.0) );
    printf("within error +/- %f: %f\n", err/2.0, r);
  }
  return 0;
}
