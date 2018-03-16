
#include <stdio.h>
#include "stage.h"

int main( int argc, char* argv[] )
{ 
  stg_world_t* world;
  stg_msec_t start;

  puts( "Stage test program: turn on the busy cursor in the world. The world will run for 5 seconds. The busy cursor should display twice for one second each time.");
  if(argc > 1 && strcmp(argv[1], "--help") == 0)
    return 0;
  stg_init( argc, argv );
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);
  puts("busycursor: running world for 5 sec.");
  stg_world_start(world);
  start = stg_world_get_time(world);
  while(stg_world_get_time(world) - start < 1000)
    stg_world_update(world, TRUE, FALSE);
  stg_world_display_message(world, 0, "busycursor", 3, "setting cursor to busy.");
  stg_world_set_cursor_busy(world);
  while(stg_world_get_time(world) - start < 2000)
    stg_world_update(world, TRUE, FALSE);
  stg_world_display_message(world, 0, "busycursor", 3, "setting cursor to normal.");
  stg_world_set_cursor_normal(world);
  while(stg_world_get_time(world) - start < 3000)
    stg_world_update(world, TRUE, FALSE);
  stg_world_display_message(world, 0, "busycursor", 3, "setting cursor to busy.");
  stg_world_set_cursor_busy(world);
  while(stg_world_get_time(world) - start < 4000)
    stg_world_update(world, TRUE, FALSE);
  stg_world_display_message(world, 0, "busycursor", 3, "setting cursor to normal.");
  stg_world_set_cursor_normal(world);
  while(stg_world_get_time(world) - start < 5000)
    stg_world_update(world, TRUE, FALSE);
  stg_world_display_message(world, 0, "busycursor", 3, "no longer updating the world.");
  stg_world_display_message(world, 0, "busycursor", 3, "destroying the world.");
  stg_world_destroy(world);
  puts("busycursor: end of program.");
  return 0;
}
