/////////////////////////////////
// File: createmodel.c
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
  stg_print_msg( "Stage test program: to load a world file named \"createmodel_worldfile.world\", then creates a model with type \"test_model_def\". The model should appear in the GUI with the properties given in the macro definition for \"test_model_def\" in the world file. The simulation then runs until exit.");
  stg_init( argc, argv );
  stg_print_msg("createmodel: creating the world...");
  world = stg_world_create(0, "test_world", STG_DEFAULT_INTERVAL_SIM, STG_DEFAULT_INTERVAL_REAL, 
    1.0/STG_DEFAULT_RESOLUTION, STG_DEFAULT_WORLD_WIDTH, STG_DEFAULT_WORLD_HEIGHT);

  stg_print_msg("createmodel: creating  plain \"model\"...");
  model = stg_world_new_model(world, "model", NULL, NULL);
  assert(model);

  stg_print_msg("createmodel: creating three plain \"model\" models with the same requested name (eventual name token should be different for each)...");
  model = stg_world_new_model(world, "model", NULL, "testmodel");
  assert(model);
  model = stg_world_new_model(world, "model", NULL, "testmodel");
  assert(model);
  model = stg_world_new_model(world, "model", NULL, "testmodel");
  assert(model);

  stg_print_msg("createmodel: creating a model using type \"test_model_def\" which is not yet defined. this should return NULL (assertion fails if not NULL).");
  model = stg_world_new_model(world, "test_model_def", NULL, NULL);
  assert(model == NULL);

  stg_print_msg("createmodel: loading world file \"createmodel_worldfile.world\"");
  if(!stg_world_load_world_file(world, "createmodel_worldfile.world"))
  {
    stg_print_error("Error: could not load worldfile \"createmodel_worldfile.world\"!");
    exit(1);
  }


  stg_print_msg("createmodel: creating a model using type \"test_model_def\"...");
  model = stg_world_new_model(world, "test_model_def", NULL, NULL);
  if(!model)
  {
    stg_print_error("Error: could not create a model of type \"test_model_def\".");
    exit(2);
  }

  stg_print_msg("createmodel: running world...");
  stg_world_start(world);
  while( stg_world_update(world, 1, 0) == 0)
    stg_world_update(world, 0, 0);
  
  stg_print_msg("createmodel: exited.");
  stg_print_msg("createmodel: removing the model from the world and destroying.");
  stg_world_remove_model(world, model);
  stg_model_destroy(model);

  stg_print_msg("createmodel: destroying the world.");
  stg_world_destroy(world);
  stg_print_msg("createmodel: end of program.");
  return 0;
}
