/////////////////////////////////
// File: modeldestroy.c
// Desc: Stage library test program
// Created: 2006.6.15
// Author: Reed Hedges <reed@mobilerobots.com>
// CVS: $Id$
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include "stage.h"

int main( int argc, char* argv[] )
{ 
  stg_world_t* world;
  stg_model_t* model;
  puts( "Stage test program: create and destroy some models. The simulation runs for 5 seconds. All memory should have been deallocated by program end.");
  stg_init( argc, argv );
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);
  model = stg_world_new_model(world, "model", NULL, NULL);
  assert(model);
  /*
  puts("modeldestroy: running world for 5 sec.");
  stg_world_start(world);
  stg_msec_t start = stg_world_get_time(world);
  while(stg_world_get_time(world) - start < 5000)
    stg_world_update(world, 0);
  puts("modeldestroy: no longer updating the world.");
  */
  puts("modeldestroy: removing the model from the world and destroying.");
  stg_world_remove_model(world, model);
  stg_model_destroy(model);
  puts("modeldestroy: destroying the world.");
  stg_world_destroy(world);
  puts("modeldestroy: end of program.");
  return 0;
}
