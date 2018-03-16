#ifndef _STG_BLINKER_H_
#define _STG_BLINKER_H_


typedef struct _stg_blinker {
  unsigned int state;
  unsigned int num_states;
  unsigned long interval;
  unsigned long last_change_time;
  int active;
} stg_blinker_t;

void stg_blinker_init(stg_blinker_t *blinker, unsigned long interval, unsigned int num_states);
unsigned int stg_blinker_update(stg_blinker_t *blinker);
void stg_blinker_activate(stg_blinker_t *blinker);
void stg_blinker_deactivate(stg_blinker_t *blinker);
int stg_blinker_active(stg_blinker_t *blinker);

#endif
