
#include <stdio.h>
#include "stage.h"

int main( int argc, char* argv[] )
{ 
  stg_world_t *world;
  stg_model_t *model1, *model2, *model3;
  stg_meters_t o = -0.05;

  puts( "Stage test program: create some models with height properties and examine them.");
  stg_init( argc, argv );
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);
  model1 = stg_world_new_model(world, "model", NULL, NULL);
  model2 = stg_world_new_model(world, "model", model1, NULL);
  model3 = stg_world_new_model(world, "model", model2, NULL);

  stg_model_set_height(model1, 0.5);
  printf("height test: set model1 height to 0.5m. height_in_world is now %0.4f. (shoud be 0.5)\n", stg_model_get_height_in_world(model1));
  stg_model_set_height(model2, 0.25);
  printf("height test: set model2 height to 0.25m. height_in_world is now %0.4f. (should be 0.5+0.25 = 0.75)\n", stg_model_get_height_in_world(model2));
  stg_model_set_property(model2, "height_offset", &o, sizeof(o));
  stg_model_recalc_height_in_world(model2);
  printf("height test: set model2 height_offset to -0.05m and recalculated height_in_world. height_in_world is now %0.4f. (should be 0.5+0.25-0.05 = 0.7)\n", stg_model_get_height_in_world(model2));
  stg_model_set_height(model3, 1.0);
  printf("height test: set model3 height to 1.0m. height_in_world is now %0.4f. (should be 0.5+0.25-0.05+1.0 = 1.7)\n", stg_model_get_height_in_world(model3));

  puts("height test: end of program.");
  return 0;
}
