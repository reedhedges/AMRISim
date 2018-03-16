///////////////////////////////////////////////////////////////////////////
//
// File: example_custom_model.c
// Author: Reed Hedges <reed@activmedia.com>
// Date: 13 October 2005
//
// CVS info:
//  $Source$
//  $Author$
//  $Revision$
//
///////////////////////////////////////////////////////////////////////////
//
// This is an example model type implementation for Stage applications.
// Add it with stg_model_register_custom_type().
//
// Start at the examplemodel_init function, that's where it all begins.
//
///////////////////////////////////////////////////////////////////////////


#include "stage.h"
#include "stage/src/stage_internal.h"
#include "stage/src/gui.h"
#include "stage/src/rtk.h"

#include "stage/stage_internal.h"

// Callback called when loaded from the world file. Parse parameters here.
void examplemodel_load( stg_model_t* mod, int wf_id )
{
  // If your model wanted to try reading any information from the worldfile, it
  // could do so using the wf_ functions defined in stage_internal.h, using
  // wf_id as the "section ID" in the worldfile.
  stg_print_message("example model load function!\n");
}

// Callback called during the first subscription 
int examplemodel_startup( stg_model_t* mod )
{ 
  // If a model is only useful if a client is subscribed, then you can set the
  // update and render callbacks here.
  stg_print_message( "examplemodel startup function" );
  return 0; // no error
}


// Callback called at last subscription
int examplemodel_shutdown( stg_model_t* mod ) {
  // If a model is only useful if a client is subscribed, then you can remove the
  // update and render callbacks here.
  stg_print_message("example model shutdown function!\n");
  return 0; // no error
}



// Callback called every simulation loop
int examplemodel_update( stg_model_t* mod ) {
  // Just increment our test property
  int *i = (int*) stg_model_get_property_fixed(mod, "testproperty", sizeof(int));
  assert(i);
  (*i)++;
  stg_model_property_refresh(mod, "testproperty");
}

// Our callback to draw the model
int examplemodel_render( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp )
{
  stg_rtk_fig_t* fig = stg_model_get_fig(mod, "testfig");
  int* val = (int*) stg_model_get_property_fixed(mod, name, sizeof(int));
  assert(val);
  char text[32];
  snstg_print_message(text, 32, "Test Fig for property \"%s\"!\nProperty value=%d", name, *val);
  stg_rtk_fig_text(fig, 1.0, 0.1, 0, text);
  return 0; // ok
}


// Our callback to clear the model's display
int examplemodel_unrender( stg_model_t* mod, char* name, 
			 void* data, size_t len, void* userp )
{
  stg_model_fig_clear( mod, "testfig" );
  return 1; // callback just runs one time
}

// Called before model is destroyed. Properties are automatically
// deallocated, but if this model allocated any memory or added
// property data toggle items to the GUI with stg_model_add_property_toggles
// (e.g. in the init function belowe), then that stuff can be cleaned up here.
void examplemodel_destroy(stg_model_t *mod)
{
  stg_print_message("example model destroy function!");
}

// Initialization function registered with our custom type in Stage,
// Called when this model is created
int examplemodel_init( stg_model_t* mod ) {
  stg_print_message("example model init function!");

  // Set our callbacks for various purposes
  mod->f_load = examplemodel_load;
  mod->f_startup = examplemodel_startup;
  mod->f_shutdown = examplemodel_shutdown;
  mod->f_update = examplemodel_update;
  mod->f_destroy = examplemodel_destroy;

  // set initial property value
  int *i = (int*) malloc(sizeof(int));
  *i = 23;
  stg_model_set_property(mod, "testproperty", i, sizeof(int));

  // Set the render callback to be called when the property changes
  stg_model_add_property_callback(mod, "testproperty", examplemodel_render, 0);

  // This is how you prevent Stage from drawing default shapes for a model
  // (which it does for any model that does not set its own figure)
  //stg_model_set_property( mod, "polygons", NULL, 0 );
  //stg_geom_t geom;
  //memset( &geom, 0, sizeof(geom));
  //stg_model_set_property( mod, "geom", &geom, sizeof(geom) );

  // ... But we'll make a figure. This is turned on/off in the startup/shutdown
  // callbacks and modified in the "render" property callback
  stg_model_fig_create(mod, "testfig", NULL, 100);

  // You could call these in a property callback if you wanted them to reflect
  // variable properties of the model instead of constants like we do here:
  stg_color_t col = stg_lookup_color("cyan");
  stg_rtk_fig_color_rgb32(fig, col);
  stg_rtk_fig_origin(fig, -2, 0, 0);
  stg_rtk_fig_rectangle(fig, 0, 0, 0, 0.1, 0.1, 0);

  return 0;
}

