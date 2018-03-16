
#include "blinker.h"
#include "stage.h"

int main(int argc, char **argv)
{
  stg_msec_t start;
  unsigned int prevstate;
  stg_blinker_t blinker;
  stg_blinker_init(&blinker, 1000, 2);
  blinker.active = 1;
  start = stg_timenow();
  prevstate = 999;
  printf("Testing blinker (interval 1000ms, 2 states): ");
  fflush(stdout);
  while(stg_timenow() - start <= 5000)
  {
    unsigned int state = stg_blinker_update(&blinker);
    if(state != prevstate)
    {
      printf("%u... ", state);
      fflush(stdout);
      prevstate = state;
    }
    usleep(10*1000);
  }
  printf("\nChanging interval to 500ms, 4 states: ");
  fflush(stdout);
  blinker.interval = 500;
  blinker.num_states = 4;
  start = stg_timenow();
  while(stg_timenow() - start <= 5000)
  {
    unsigned int state = stg_blinker_update(&blinker);
    if(state != prevstate)
    {
      printf("%u... ", state);
      fflush(stdout);
      prevstate = state;
    }
    usleep(10*1000);
  }
  puts("");
}
