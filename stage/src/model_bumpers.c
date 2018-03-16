///////////////////////////////////////////////////////////////////////////
//
// File: model_bumpers.c
// Author: Reed Hedges <reed@activmedia.com>
//
///////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <sys/time.h>
#include <stdlib.h>

//#define DEBUG


#include "stage_internal.h"
#include "gui.h"

/**
@ingroup model
@defgroup model_bumpers Bumpers model 
The bumpers model simulates an array of outward-facing switches triggered by collision.
Its sort of a simplified ranger model, really.

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
bumpers
(
  # bumpers properties
  count 16
  size [0.01 0.03]
  autostall false
  autostop false
  pose[0] [? ? ?]
  pose[1] [? ? ?]
  pose[2] [? ? ?]
  pose[3] [? ? ?]
  pose[4] [? ? ?]
  pose[5] [? ? ?]
  pose[6] [? ? ?]
  pose[7] [? ? ?]
  pose[8] [? ? ?]
  spose[9] [? ? ?]
  pose[10] [? ? ?]
  pose[11] [? ? ?]
  pose[12] [? ? ?]
  pose[13] [? ? ?]
  pose[14] [? ? ?]
  pose[15] [? ? ?]
)
@endverbatim

@par Notes

The bumpers model allows configuration of the pose and size of each switch seperately 
(using pose[index] and size[index]). However, most users will set a common size (using size)
and sview), and just specify individual switch poses.

@par Details
- count int 
  - the number of switches
- pose[\<switch index\>] [float float float]
  - [x y theta] 
  - pose of a switch relative to its parent.
- size [float float]
  - [x y] 
  - size of each switch in meters.
- size[\<switch index\>] [float float]
  - per-switch version of the size property. Overrides the common setting.
- autostop [boolean]
  - if true, and the parent model is a position model, clear position model's commanded
    velocity as soon as a switch is triggered.
- autostall [boolean]
  - if true, set the parent model's 'collision' flag as soon as a switch is triggered.
*/

  
/** 
@ingroup stg_model_bumpers
@ingroup stg_model_props
@defgroup stg_model_bumpers_props Bumper Properties

- "bumpers_cfg" stg_bumpers_config_t
- "bumpers_data" stg_bool_t[] (see model overview description)
*/


void bumpers_load( stg_model_t* mod, int wf_id )
{
  // Load the geometry of a bumpers array
  stg_bumpers_config_t cfg;
  cfg->count = wf_read_int(wf_id, "count", 0);
  cfg->autostop = wf_read_bool(wf_id, "autostop", 0);
  cfg->autostall = wf_read_bool(wf_id, "autostall", 0);
  if (count > 0)
  {
    char key[256];
    stg_pose_t* switch_cfg = (stg_pose_t*) malloc( sizeof(stg_pose_t), count );
    
    cfg->dd
    stg_size_t common_size;
    common_size.x = wf_read_tuple_length(wf_id, "size", 0, 0.020);
    common_size.y = wf_read_tuple_length(wf_id, "size", 1, 0.10);
    
    // allow individual configuration of switchs (some parameters)
    for(i = 0; i < count; i++)
    {
      snprintf(key, sizeof(key), "pose[%d]", i);
      switch_cfg[i].pose.x = wf_read_tuple_length(wf_id, key, 0, 0);
      switch_cfg[i].pose.y = wf_read_tuple_length(wf_id, key, 1, 0);
      switch_cfg[i].pose.a = wf_read_tuple_angle(wf_id, key, 2, 0);
      
      snprintf(key, sizeof(key), "size[%d]", i);
      switch_cfg[i].size.x = wf_read_tuple_length(wf_id, key, 0, common_size.x);
      switch_cfg[i].size.y = wf_read_tuple_length(wf_id, key, 1, common_size.y);
    }
      
    PRINT_DEBUG1( "loaded %d bumpers configs", count );	  

    stg_model_set_property(mod, "bumpers_switch_cfg", switch_cfg, count * sizeof(stg_bumpers_switch_config_t));
    free(switch_cfg);
  }

  stg_model_set_property(mod, "bumpers_cfg", &cfg, sizeof(cfg));
}


int bumpers_update( stg_model_t* mod );
int bumpers_startup( stg_model_t* mod );
int bumpers_shutdown( stg_model_t* mod );

int bumpers_render_data( stg_model_t* mod, char* name, 
			void* data, size_t len, void* userp );
int bumpers_unrender_data( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp );
int bumpers_render_cfg( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );

int bumpers_init( stg_model_t* mod )
{
  // override the default methods
  mod->f_startup = bumpers_startup;
  mod->f_shutdown = bumpers_shutdown;
  mod->f_update = NULL; // installed on startup & shutdown
  mod->f_load = bumpers_load;
  
  stg_model_set_property( mod, "bumpers_data", NULL, 0 );
  
  // Set up sensible defaults
  stg_geom_t geom;
  memset( &geom, 0, sizeof(geom)); // no size
  stg_model_set_property( mod, "geom", &geom, sizeof(geom) );
  
  stg_color_t col = stg_lookup_color( STG_BUMPERS_COLOR );
  stg_model_set_property( mod, "color", &col, sizeof(col) );
  
  // remove the polygon: bumpers has no body
  stg_model_set_property(mod, "polygons", NULL, 0 );

  // Initialize 
  stg_model_set_property(mod, "bumpers_cfg", NULL, 0);
  
  // Draw active bumpers
  stg_model_add_property_callback(mod, "bumpers_data", bumpers_render_data, NULL);

  return 0;
}

int bumpers_startup( stg_model_t* mod )
{
  PRINT_DEBUG( "bumpers startup" );
  mod->f_update = bumpers_update;
  return 0;
}


int bumpers_shutdown( stg_model_t* mod )
{
  PRINT_DEBUG( "bumpers shutdown" );
  mod->f_update = NULL;
  // clear the data - this will unrender it too.
  stg_model_set_property( mod, "bumpers_data", NULL, 0 );
  return 0;
}


int bumpers_update( stg_model_t* mod )
{     
  if( mod->subs < 1 )
    return 0;

  //PRINT_DEBUG1( "[%d] updating bumpers", mod->world->sim_time );
  
  size_t cfg_len = 0;
  stg_bumpers_config_t *cfg = stg_model_get_property( mod, "bumpers_cfg", &cfg_len );
  
  if( cfg_len < sizeof(stg_bumpers_config_t) )
    return 0; // nothing to see here

  if (cfg->count == 0)
    return 0;  

  stg_bumpers_switch_cfg_t *switch_cfg  = stg_model_get_property(mod, "bumpers_switch_cfg", &cfg_len);
  if(cfg_len == 0)
    return 0;
  
  size_t data_len;
  stg_bool_t* data = (stg_bool_t*) stg_model_get_property(mod, "bumpers_data", &data_len);

  // data was unset
  // TODO: config callback that resizes the array if the count changes.
  if(data == NULL)
  {
    data = malloc(cfg->count);
  }

  // Test each switch for collision
  int i;
  for( i = 0; i < cfg->count; i++ )
  {
    // get the switch's pose in global coords
    stg_pose_t p;
    memcpy( &p, &switch_cfg[i].pose, sizeof(p) ); 
    stg_model_local_to_global( mod, &p );
     
    // test collision
    // TODO
    data[i] = FALSE;
    
  }
        
  stg_model_property_changed(mod, "bumpers_data");
  return 0;
}


int bumpers_unrender_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  stg_model_fig_clear( mod, "bumpers_data_fig" );
  return 1; // quit callback
}


int bumpers_render_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  PRINT_DEBUG( "bumpers render data" );
  
  stg_rtk_fig_t* fig = stg_model_get_fig( mod, "bumpers_data_fig" );

  if( !fig )
    fig = stg_model_fig_create( mod, "bumpers_data_fig", "top", STG_LAYER_BUMPERSDATA );

  stg_rtk_fig_clear(fig);

  size_t clen=0;
  stg_bumpers_config_t* cfg = stg_model_get_property(mod, "bumpers_cfg", &clen );
  
  // any data at all?
  if( clen < sizeof(stg_bumpers_config_t) )
    return 0;
  
  int rcount = clen / sizeof( stg_bumpers_config_t );
  
  if( rcount < 1 ) // no data 
    return 0; 
  
  size_t dlen = 0;
  stg_bumpers_sample_t *data = 
    stg_model_get_property( mod, "bumpers_data", &dlen );
  
  if(data == NULL)
  {
    stg_bumpers_unrender_data(...);
    return 0;
  }

  // iff we have the right amount of data
  if( dlen == rcount * sizeof(stg_bumpers_sample_t) )
    {       
      stg_geom_t geom;
      stg_model_get_geom(mod,&geom);
      
      stg_rtk_fig_color_rgb32(fig, stg_lookup_color(STG_BUMPERS_BRIGHT_COLOR) );
      stg_rtk_fig_origin( fig, geom.pose.x, geom.pose.y, geom.pose.a );	  
      
      // draw the range  beams
      int s;
      for( s=0; s<rcount; s++ )
	{
	  if( data[s].range > 0.0 )
	    {
	      stg_bumpers_config_t* rngr = &cfg[s];
	      
	      stg_rtk_fig_arrow( fig, 
				 rngr->pose.x, rngr->pose.y, rngr->pose.a, 	
				 data[s].range, 0.02 );
	    }
	}
    }
  else
    if( dlen > 0 )
      PRINT_WARN2( "data size doesn't match configuation (%d/%d bytes)",
		   dlen,  rcount * sizeof(stg_bumpers_sample_t) );
  
  return 0; // keep running
}

