/////////////////////////////////
// File: locktime.c
// Desc: Stage library test program. Test lock contention between world update thread and accessors.
// Created: 2006.11.15
// Author: Reed Hedges <reed@mobilerobots.com>
// CVS: $Id$
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <assert.h>
#include <unistd.h>

#define STG_DEBUG_MODEL_LOCK 1
#include "stage.h"

void *threadMain(void *ptr);

struct simdata {
  stg_world_t* world;
  stg_model_t* model;
};


#define NUM_THREADS 15

int main( int argc, char* argv[] )
{ 
  struct simdata stuff;
  pthread_t threads[NUM_THREADS];
  int stat = 0;
  int i;

  stg_init( argc, argv );
  stg_print_msg("createmodel: creating the world...");
  stuff.world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);

  stg_print_msg("createmodel: creating a plain \"model\"...");
  stuff.model = stg_world_new_model(stuff.world, "model", NULL, NULL);
  assert(stuff.model);

  //stg_world_enable_world_lock(stuff.world, TRUE);
  //stg_world_set_model_update_locking_policy(stuff.world, STG_WORLD_LOCK);

  stg_world_start(stuff.world);

  for(i = 0; i < NUM_THREADS; ++i)
  {
	  int err = pthread_create(&threads[i], NULL, &threadMain, &stuff); 
          assert(err == 0);
  }

  while(stat == 0)
  {
    stat = stg_world_update(stuff.world, TRUE);
  }
}


void* threadMain(void *ptr)
{
  struct simdata* sd = (struct simdata*)ptr;
  struct timeval start_tv;
  struct timeval end_tv;
  while(TRUE)
  {
    double start, end;
    gettimeofday(&start_tv, NULL);
    stg_world_lock(sd->world);
    gettimeofday(&end_tv, NULL);
    start = (double)((start_tv.tv_sec * 1000.0) + (start_tv.tv_usec/1000.0));
    end = (double)((end_tv.tv_sec * 1000.0) + (end_tv.tv_usec/1000.0));
    stg_print_msg("Thread %x: It took %.4f ms to lock the world. Holding it for 4s.", pthread_self(), end-start);
    usleep(2*1000);

    gettimeofday(&start_tv, NULL);
    stg_model_lock(sd->model);
    gettimeofday(&end_tv, NULL);
    start = (double)((start_tv.tv_sec * 1000.0) + (start_tv.tv_usec/1000.0));
    end = (double)((end_tv.tv_sec * 1000.0) + (end_tv.tv_usec/1000.0));
    stg_print_msg("Thread %x: It took %.4f ms to lock the model. Holding it for 2s.", pthread_self(), end-start);

    usleep(2*1000);
    stg_model_unlock(sd->model);

    stg_world_unlock(sd->world);
    usleep(10*1000);
  }
  return NULL;
}

