#include <stdio.h>
#include "stage.h"

int main( int argc, char* argv[] )
{ 
  stg_world_t *world;
  stg_model_t *model1, *model2;
  //stg_pose_t pose;

  puts( "Stage test program: create a world containing some models, then \"resize\" the world, then destroy it.  All memory should have been deallocated by program end.");
  stg_init( argc, argv );
  puts("resizeworld: creating world and models...");
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);
  stg_world_start(world);
  model1 = stg_model_create(world, NULL, 0, "test_model_1", "model", "model", 0, NULL, FALSE);
  stg_world_add_model(world, model1);
  model2 = stg_model_create(world, NULL, 1, "test_model_2", "model", "model", 1, NULL, FALSE);
  stg_model_set_pose_position(model2, 0.5, 0.5);
  stg_world_add_model(world, model2);
  printf("resizeworld: resizing the world from %fX%f to %fX%f.\n", STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT,
    2.0*STG_DEFAULT_WORLD_WIDTH, 2.0*STG_DEFAULT_WORLD_HEIGHT);
  stg_world_resize(world, 2.0*STG_DEFAULT_WORLD_WIDTH, 2.0*STG_DEFAULT_WORLD_HEIGHT);
  puts("resizeworld: removing the models from the world and destroying.");
  stg_world_remove_model(world, model1);
  stg_model_destroy(model1);
  stg_world_remove_model(world, model2);
  stg_model_destroy(model2);
  puts("resizeworld: destroying the world.");
  stg_world_destroy(world);
  puts("resizeworld: end of program.");
  return 0;
}
