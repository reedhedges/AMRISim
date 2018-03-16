
#include "blinker.h"
#include "stage.h"

void stg_blinker_init(stg_blinker_t *blinker, unsigned long interval, unsigned int num_states)
{
  blinker->state = 0;
  blinker->num_states = num_states;
  blinker->interval = interval;
  blinker->last_change_time = stg_timenow();
  blinker->active = 0;
}

unsigned int stg_blinker_update(stg_blinker_t *blinker)
{
  unsigned long now;
  if(!blinker->active) return 0;
  now = stg_timenow();
  if( now - blinker->last_change_time >= blinker->interval )
  {
    blinker->state = (blinker->state + 1) % blinker->num_states;
    blinker->last_change_time = now;
  }
  return blinker->state;
}

void stg_blinker_activate(stg_blinker_t *blinker)
{
  blinker->active = 1;
}

void stg_blinker_deactivate(stg_blinker_t *blinker)
{
  blinker->active = 0;
}

int stg_blinker_active(stg_blinker_t *blinker)
{
  return blinker->active;
}

