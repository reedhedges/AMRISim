
#include "stage.h"
#include <stdio.h>

int main(int argc, char **argv)
{
  GData *list = NULL;
  char value[512];
  const int numitems = 99999;
  const int randomkeyrange = 123456;
  stg_msec_t start = stg_timenow();
  stg_msec_t sum;
  stg_msec_t maxtime = 0;
  int i;
  g_datalist_init(&list);
  puts("Adding items to data list...");
  puts("Iter\tRandKey\tTime");
  for(i = 0; i < numitems; ++i)
  {
    char *s = (char*)malloc(24);
    int n = STG_RANDOM_RANGE(0,randomkeyrange);
    stg_msec_t t = stg_timenow();
    stg_msec_t dur;
    snprintf(s, 24, "dataitem_%d", n);
    g_datalist_set_data(&list, s, (gpointer)&value);
    dur = stg_timenow() - t;
    printf("%d\t%d\t%d ms\n", i, n, dur);
    sum += dur;
    if(dur > maxtime) maxtime = dur;
  }
  printf("Added 9999 items to datalist. Total time=%d, Avg time=%d, Max time=%d\n", stg_timenow() - start, sum/numitems, maxtime);
  puts("Searching for items...");
  puts("Iter\tRandKey\tFound?\tTime");
  start = stg_timenow();
  sum = 0;
  maxtime = 0;
  for(i = 0; i < numitems; ++i)
  {
    char key[24];
    int n = STG_RANDOM_RANGE(0,randomkeyrange);
    stg_msec_t t = stg_timenow();
    char *found;
    stg_msec_t dur;
    snprintf(key, 24, "dataitem_%d", n);
    found = g_datalist_get_data(&list, key);
    dur = stg_timenow() - t;
    printf("%d\t%d\t%s\t%d ms\n", i, n, found?"yes":"no", dur);
    sum += dur;
    if(dur > maxtime) maxtime = dur;
  }
  printf("Done searching. Total time=%d, Avg time=%lu, Max time=%d\n", stg_timenow() - start, sum/numitems, maxtime);
  return 0;
}
