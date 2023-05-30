///////////////////////////////////////////////////////////////////////////
//
// File: model_ranger.c
// Author: Richard Vaughan
// Date: 10 June 2004
//
///////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <sys/time.h>
#include <stdlib.h>


//#define DEBUG 1

#include "stage_internal.h"
#include "gui.h"


extern stg_rtk_fig_t* fig_debug_rays;

#define STG_RANGER_DATA_MAX 64

#define STG_RANGER_WATTS 2.0 // ranger power consumption

/**
@ingroup model
@defgroup model_ranger Ranger model 
The ranger model simulates an array of sonar or infra-red (IR) range sensors.

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
ranger
(
  # ranger properties
  scount 16
  spose[0] [? ? ?]
  spose[1] [? ? ?]
  spose[2] [? ? ?]
  spose[3] [? ? ?]
  spose[4] [? ? ?]
  spose[5] [? ? ?]
  spose[6] [? ? ?]
  spose[7] [? ? ?]
  spose[8] [? ? ?]
  spose[9] [? ? ?]
  spose[10] [? ? ?]
  spose[11] [? ? ?]
  spose[12] [? ? ?]
  spose[13] [? ? ?]
  spose[14] [? ? ?]
  spose[15] [? ? ?]
   
  ssize [0.01 0.03]
  sview [0.0 5.0 5.0]
  no_return_indicator "max"
  noise 0.01 

  projection_type "single"   # Or "closest", "farthest", "random"
  projection_res 6  # One ray every 6 degrees (but irrelevant to "single" projection type) 
  throwaway_thresh  0.6    # Percent of farthest range reading that (farthest-closest) must be before the ranger model considers discarding the reading, based on throwaway_prob
  throwaway_prob    0.7    # Probability that a reading might be discarded, (but only if it passes the condition described for throwaway_thresh)


)
@endverbatim

@par Notes

The ranger model allows configuration of the pose, size and view parameters of each transducer seperately (using spose[index], ssize[index] and sview[index]). However, most users will set a common size and view (using ssize and sview), and just specify individual transducer poses.

@par Details
- scount int 
  - the number of range transducers
- spose[\<transducer index\>] [float float float]
  - [x y theta] 
  - pose of the transducer relative to its parent.
- ssize [float float]
  - [x y] 
  - size in meters. Has no effect on the data, but controls how the sensor looks in the Stage window.
- ssize[\<transducer index\>] [float float]
  - per-transducer version of the ssize property. Overrides the common setting.
- sview [float float float]
   - [range_min range_max fov] 
   - minimum range and maximum range in meters, field of view angle in degrees. Currently fov has no effect on the sensor model, other than being shown in the confgiuration graphic for the ranger device.
- sview[\<transducer index\>] [float float float]
  - per-transducer version of the sview property. Overrides the common setting.
- noise [double]
  - error (meters) randomly added to each reading ranges from 0-noise to 0+noise
- noise[\<transducer index\>] [double]
  - per-transducer version of the noise property.
- projection_type [string]
  - Method of casting rays into the environment to simulate a full-field range sensor.
    If "single" then simply cast one ray in the middle of the sensor. If "closest" then
    cast several rays over the field of view and use the one with the closest range. If
    "farthest" then cast several rays and use the farthest range. If "random" then cast
    a randomly chosen ray within the field. If "mean" then cast several rays and use
    the mean average.
- projection_res [double]
  - If projection_type is not "single" then this determines how many rays to cast.
    a ray is cast for every projection_res degrees in the field of view.
- enable_thowaway [int]
  - If 1, the enable a quick and dirty simulation of the problem of a signal 
    being reflected off a sufficiently angled surface and never returning to the 
    transducer.
- throwaway_thresh [double]
  - If enable_throwaway is 1, then this value gives the percent over the farthest range 
    reading that (farthest-closest) must be before the ranger model considers 
    discarding the reading.
- throwaway_prob [double]   
  - If enable_throwaway is 1, and the test described in throwaway_thresh passes, then this
    is the probability that a reading might be discarded and instead a "no return" 
    indicator is used.
*/

  
/** 
@ingroup stg_model_ranger
@ingroup stg_model_props
@defgroup stg_model_ranger_props Ranger Properties

- "ranger_cfg" stg_ranger_config_t
- "ranger_data" stg_ranger_sample_t[]
*/


void ranger_load( stg_model_t* mod, int wf_id )
{
  //PRINT_DEBUG1("ranger_load called. wf_id=%d\n", wf_id);
  // Load the geometry of a ranger array
  int scount = wf_read_int( wf_id, "scount", 0);
  PRINT_DEBUG1("ranger_load: scount=%d\n", scount);
  if (scount > 0)
    {
      int i;
      char key[256];
      stg_size_t common_size;
      double common_min, common_max, common_fov, common_noise;
      double resolution, throwaway_thresh, throwaway_prob;
      int enable_throwaway;
      const char* tmp;
      stg_ranger_projection_t projection_type;
    
      stg_ranger_config_t* configs = (stg_ranger_config_t*)
	calloc( sizeof(stg_ranger_config_t), scount );
      
      common_size.x = wf_read_tuple_length(wf_id, "ssize", 0, 0.01 );
      common_size.y = wf_read_tuple_length(wf_id, "ssize", 1, 0.03 );
      
      common_min = wf_read_tuple_length(wf_id, "sview", 0, 0.0);
      common_max = wf_read_tuple_length(wf_id, "sview", 1, 5.0);
      common_fov = wf_read_tuple_angle(wf_id, "sview", 2, 5.0);

#ifdef ENABLE_RANGER_NOISE
      common_noise = wf_read_float(wf_id, "noise", 0.0);
#endif

      tmp = wf_read_string(wf_id, "projection_type", "single");
      if(strcmp(tmp, "single") == 0)
        projection_type = STG_RANGER_SINGLE_RAY;
      else if(strcmp(tmp, "closest") == 0)
        projection_type = STG_RANGER_CLOSEST_RAY;
      else
      {
        PRINT_WARN1("Ranger (Sonar) configuration: unknown projection type \"%s\". Using default \"single\".", tmp);
        projection_type = STG_RANGER_SINGLE_RAY;
      }
      resolution = wf_read_angle(wf_id, "projection_res", 6);
      enable_throwaway = wf_read_int(wf_id, "enable_throwaway", 0);
      throwaway_thresh = wf_read_float(wf_id, "throwaway_thresh", 0.75);
      throwaway_prob = wf_read_float(wf_id, "throwaway_prob", 0.4);

      // set all transducers with the common settings
      for(i = 0; i < scount; i++)
	{
	  configs[i].size.x = common_size.x;
	  configs[i].size.y = common_size.y;
	  configs[i].bounds_range.min = common_min;
	  configs[i].bounds_range.max = common_max;
	  configs[i].fov = common_fov;
    configs[i].noise = common_noise;
    configs[i].projection_type = projection_type;
    configs[i].resolution = resolution;
    configs[i].enable_throwaway = enable_throwaway;
    configs[i].throwaway_thresh = throwaway_thresh;
    configs[i].throwaway_prob = throwaway_prob;
	}

      // allow individual configuration of transducers (some parameters)
      for(i = 0; i < scount; i++)
	{
	  snprintf(key, sizeof(key), "spose[%d]", i);
	  configs[i].pose.x = wf_read_tuple_length(wf_id, key, 0, 0);
	  configs[i].pose.y = wf_read_tuple_length(wf_id, key, 1, 0);
	  configs[i].pose.a = wf_read_tuple_angle(wf_id, key, 2, 0);
	  
	  snprintf(key, sizeof(key), "ssize[%d]", i);
	  configs[i].size.x = wf_read_tuple_length(wf_id, key, 0, common_size.x);
	  configs[i].size.y = wf_read_tuple_length(wf_id, key, 1, common_size.y);
	  
	  snprintf(key, sizeof(key), "sview[%d]", i);
	  configs[i].bounds_range.min = 
	    wf_read_tuple_length(wf_id, key, 0, common_min);
	  configs[i].bounds_range.max =   // set up sensible defaults

	    wf_read_tuple_length(wf_id, key, 1, common_max);
	  configs[i].fov = wf_read_tuple_angle(wf_id, key, 2, common_fov );

#ifdef ENABLE_RANGER_NOISE
    snprintf(key, sizeof(key), "noise[%d]", i);
    configs[i].noise = wf_read_float(wf_id, key, common_noise);
#else
    configs[i].noise = 0;
#endif
	}
      
      PRINT_DEBUG1( "loaded %d ranger configs", scount );	  

      stg_model_set_property( mod, "ranger_cfg", configs, scount * sizeof(stg_ranger_config_t) );
      
      free( configs );
    }
}

//void ranger_render_cfg( stg_model_t* mod );
//void ranger_render_data( stg_model_t* mod ) ;

int ranger_update( stg_model_t* mod );
int ranger_startup( stg_model_t* mod );
int ranger_shutdown( stg_model_t* mod );
void ranger_destroy(stg_model_t *mod);

int ranger_render_data( stg_model_t* mod, char* name, 
			void* data, size_t len, void* userp );
int ranger_unrender_data( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp );
int ranger_render_cfg( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );


int ranger_init( stg_model_t* mod )
{
  stg_color_t col;
  stg_geom_t geom;
  stg_ranger_config_t cfg[16];
  size_t cfglen;
  double offset;
  int c;
  gboolean debug;

  // seed random number generator
  // this is now done in stg_init().
  //static int first_time = 1;
  //if(first_time)
  //{
  //  first_time = 0;
  //  srand(time(NULL));
  //}

  // override the default methods
  mod->f_startup = ranger_startup;
  mod->f_shutdown = ranger_shutdown;
  mod->f_update = NULL; // installed on startup & shutdown
  mod->f_destroy = ranger_destroy;
  mod->f_load = ranger_load;
  
  stg_model_set_property( mod, "ranger_data", NULL, 0 );
  
  // Set up sensible defaults
  memset( &geom, 0, sizeof(geom)); // no size
  stg_model_set_property( mod, "geom", &geom, sizeof(geom) );
  
  col = stg_lookup_color( STG_RANGER_CONFIG_COLOR );
  stg_model_set_property( mod, "color", &col, sizeof(col) );

  debug = FALSE;
  stg_model_set_property(mod, "ranger_debug", &debug, sizeof(debug));
  
  // remove the polygon: ranger has no body
  stg_model_set_property( mod, "polygons", NULL, 0 );

  
  // Look up color used to render data
  col = stg_lookup_color(STG_RANGER_COLOR);
  stg_model_set_property(mod, "ranger_data_color", &col, sizeof(col));

  // create default ranger config
  cfglen = 16*sizeof(cfg[0]);
  memset( cfg, 0, cfglen);
  
  offset = MIN(geom.size.x, geom.size.y) / 2.0;

  for( c=0; c<16; c++ )
    {
      cfg[c].pose.a = M_PI/8 * c;
      cfg[c].pose.x = offset * cos( cfg[c].pose.a );
      cfg[c].pose.y = offset * sin( cfg[c].pose.a );
      
      
      cfg[c].size.x = 0.01; // a small device
      cfg[c].size.y = 0.04;
      
      cfg[c].bounds_range.min = 0;
      cfg[c].bounds_range.max = 5.0;
      cfg[c].fov = M_PI/6.0;
      cfg[c].noise = 0.0;
    }
  stg_model_set_property( mod, "ranger_cfg", cfg, cfglen );
  
  // adds a menu item and associated on-and-off callbacks
  if(mod->world->gui_enabled)
    stg_model_add_property_toggles( mod, "ranger_data", 
				  ranger_render_data, // called when toggled on
				  NULL,
				  ranger_unrender_data,
				  NULL,
				  "Sonar (Ranger) Data",
				  TRUE );

/*   stg_model_add_property_toggles( mod, "ranger_cfg",  */
/* 				  ranger_render_cfg, // called when toggled on */
/* 				  NULL, */
/* 				  stg_model_fig_clear_cb, // called when toggled off */
/* 				  gui->cfg, // arg to stg_fig_clear_cb */
/* 				  "ranger config", */
/* 				  TRUE ); */
  


  return 0;
}

void ranger_destroy(stg_model_t *mod)
{
  if(mod->world->gui_enabled)
    stg_model_remove_property_toggles(mod, "ranger_data");
}

int ranger_startup( stg_model_t* mod )
{
  PRINT_DEBUG( "ranger startup" );

  mod->f_update = ranger_update;
  //mod->watts = STG_RANGER_WATTS;

  return 0;
}


int ranger_shutdown( stg_model_t* mod )
{
  PRINT_DEBUG( "ranger shutdown" );

  mod->f_update = NULL;
  //mod->watts = 0.0; // stop consuming power
  
  // clear the data - this will unrender it too.
  stg_model_set_property( mod, "ranger_data", NULL, 0 );

  return 0;
}

int ranger_raytrace_match( stg_model_t* mod, stg_model_t* hitmod )
{
  stg_ranger_return_t* rr;
  if(!hitmod) return FALSE;
  rr = stg_model_get_property_fixed( hitmod, 
				  "ranger_return", 
				  sizeof(stg_ranger_return_t));
  if(!rr) return FALSE;
  
  // Ignore myself, my children, and my ancestors.
  return(*rr && !stg_model_is_related(mod,hitmod) );
}	


int ranger_update( stg_model_t* mod )
{     
  size_t len = 0;
  stg_ranger_config_t* cfg;
  stg_ranger_sample_t* ranges;
  int t, rcount;


  if( mod->subs < 1 )
    return 0;

  //PRINT_DEBUG1( "[%d] updating rangers", mod->world->sim_time );
  
  cfg =
    stg_model_get_property( mod, "ranger_cfg", &len );
  

  if( len < sizeof(stg_ranger_config_t) )
    return 0; // nothing to see here
  
  rcount = len / sizeof(stg_ranger_config_t);
  
  ranges = (stg_ranger_sample_t*)
    calloc( sizeof(stg_ranger_sample_t), rcount );
  

  if( fig_debug_rays ) stg_rtk_fig_clear( fig_debug_rays );

  // Find range for each "transducer" in the ranger array:
  for( t=0; t<rcount; t++ )
  {
    stg_meters_t range;
    stg_radians_t obs_angle = 0;

    // get the sensor's pose in global coords
    stg_pose_t pz;
    memcpy( &pz, &cfg[t].pose, sizeof(pz) ); 
    stg_model_local_to_global( mod, &pz );
     
    // if the view is impossible, don't bother casting rays, just return
    // the max range.  this lets you "disable" sensors in the world file
    if(cfg[t].fov == 0)
    {
      ranges[t].range = cfg[t].bounds_range.max;
      ranges[t].intersect_flag = FALSE;
      continue;
    }


    // Based on projection type, cast some rays and determine range and 
    // angle of the thing we hit.
    range = cfg[t].bounds_range.max;
    ranges[t].intersect_flag = FALSE;
    if(cfg[t].projection_type == STG_RANGER_SINGLE_RAY) 
    {
      stg_model_t* hitmod;
      itl_t *itl;
      stg_matrix_lock(mod->world->matrix);
      itl = itl_create(pz.x, pz.y, pz.a, cfg[t].bounds_range.max,
          mod->world->matrix, PointToBearingRange);
      hitmod = itl_first_matching( itl, ranger_raytrace_match, mod);
      stg_matrix_unlock(mod->world->matrix);
      if( hitmod )
      {
        ranges[t].intersect_flag = TRUE;
        obs_angle = itl->obs_angle;
        if(itl->range < cfg[t].bounds_range.min)
          range = cfg[t].bounds_range.min;
        else
          range = itl->range;
      }
      itl_destroy(itl);
    } 
    else if(cfg[t].projection_type == STG_RANGER_CLOSEST_RAY)
    {
      int ray = 0;
      int numrays = ceil(cfg[t].fov / cfg[t].resolution);
      double da = (cfg[t].fov / numrays);
      double a = NORMALIZE( (pz.a - (cfg[t].fov/2.0)) + (da/2.0) ); // start ray
      stg_meters_t maxrange = -1;
      stg_meters_t minrange = -1;
      for(ray = 0; ray < numrays; ray++)
      {
        stg_model_t * hitmod;
	itl_t *itl;
        stg_matrix_lock(mod->world->matrix);
        itl = itl_create( pz.x, pz.y, a, 
          cfg[t].bounds_range.max, 
          mod->world->matrix, 
          PointToBearingRange );
        hitmod = itl_first_matching( itl, ranger_raytrace_match, mod);
        stg_matrix_unlock(mod->world->matrix);
        if( hitmod )
        {
          // If its closer, save it, but restrict to >= min range:
          if(itl->range < range)
          {
            ranges[t].intersect_flag = TRUE;
            obs_angle = itl->obs_angle;
            if(itl->range < cfg[t].bounds_range.min)
              range = cfg[t].bounds_range.min;
            else
              range = itl->range;
          }

          // Update max and min ranges
          if(range > maxrange || maxrange == -1)
            maxrange = range;
          if(range < minrange || minrange == -1)
            minrange = range;
        }
        a = NORMALIZE(a + da);
        itl_destroy(itl);
      }

      // TODO replace this with a "rule" that manually overrides the range for a
      // certain intersection angle.
      
      //printf("\tclosest=%.3f, min=%.3f, max=%.3f, diff=%.3f, thresh=%.2f, prob=%.2f\n", range, minrange, maxrange, (maxrange-minrange), cfg[t].throwaway_thresh, cfg[t].throwaway_prob);
      if(cfg[t].enable_throwaway && (maxrange - minrange) >= cfg[t].throwaway_thresh)
      {
        // throw it away sometimes:
        //double r = (double)rand() / (double)RAND_MAX;
        double r = STG_RANDOM_IN(0.0, 1.0);
        if( ( r ) <= cfg[t].throwaway_prob )
        {
          range = cfg[t].bounds_range.max;
        }
      }
    }
    else
    {
      PRINT_ERR2("WTF: Ranger: unsupported projection_type in cfg[%d]: %d", t, cfg[t].projection_type);
    }
        

#ifdef ENABLE_RANGER_NOISE
    // Add noise except during special max-range condition (or if noise is 0):
    double noise = cfg[t].noise;
    if(noise > 0.0 && range < cfg[t].bounds_range.max)
    {
      //range += ((double)(rand() - RAND_MAX/2) / (double)RAND_MAX) * cfg[t].noise;
      range += STG_RANDOM_IN(-noise, noise);
    }
#endif

    // record the range
    ranges[t].range = range;

    // record the angle of intersection (-90,90 degrees)
    if(ranges[t].intersect_flag)
    {
      ranges[t].intersect_angle = stg_intersection_angle(obs_angle, pz.a);
    }

  }



        
  // Set property
  stg_model_set_property( mod, "ranger_data", ranges, sizeof(stg_ranger_sample_t) * rcount );
  free( ranges );
  return 0;
}

/*
int ranger_noise_test( stg_ranger_sample_t* data, size_t count,  )
{
  int s;
  for( s=0; s<count; s++ )
    {
      // add 10mm random error
      ranges[s].range *= 0.1 * drand48();
    }
}
*/

int ranger_render_cfg( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  stg_rtk_fig_t* fig;
  stg_geom_t geom;
  int s;
  stg_ranger_config_t *cfg = (stg_ranger_config_t*)data;
  int rcount = len / sizeof( stg_ranger_config_t );

  if( cfg == NULL || rcount < 1 )
    return 0; // nothing to draw
  
  fig = stg_model_get_fig( mod, "ranger_cfg_fig" );

  if( !fig )
    fig = stg_model_fig_create( mod, "ranger_cfg_fig", "top", STG_LAYER_RANGERCONFIG );
  
  stg_model_get_geom(mod,&geom);
  
  stg_rtk_fig_clear(fig);
  stg_rtk_fig_origin( fig, geom.pose.x, geom.pose.y, geom.pose.a );  
  
  // add rects showing ranger positions
  for( s=0; s<rcount; s++ )
    {
      stg_ranger_config_t* rngr = &cfg[s];
      double sidelen, da, x1, y1, x2, y2;
      char str[4];
      
      // sensor pose
      stg_rtk_fig_rectangle( fig, 
			     rngr->pose.x, rngr->pose.y, rngr->pose.a,
			     rngr->size.x, rngr->size.y, 
			     mod->world->win->fill_polygons ); 
      
      // TODO - FIX THIS
      
      // sensor FOV
      sidelen = rngr->bounds_range.max;
      da = rngr->fov/2.0;
      
      x1= rngr->pose.x + sidelen*cos(rngr->pose.a - da );
      y1= rngr->pose.y + sidelen*sin(rngr->pose.a - da );
      x2= rngr->pose.x + sidelen*cos(rngr->pose.a + da );
      y2= rngr->pose.y + sidelen*sin(rngr->pose.a + da );
      
      stg_rtk_fig_line( fig, rngr->pose.x, rngr->pose.y, x1, y1 );
      stg_rtk_fig_line( fig, rngr->pose.x, rngr->pose.y, x2, y2 );	
      
      stg_rtk_fig_ellipse_arc( fig, 
			       rngr->pose.x, rngr->pose.y, rngr->pose.a,
			       2.0*cfg->bounds_range.max,
			       2.0*cfg->bounds_range.max, 
			       -da, da );

      // sensor number
      snprintf(str, 4, "%3d", s);
      stg_rtk_fig_text(fig,
          rngr->pose.x + sidelen*cos(rngr->pose.a),
          rngr->pose.y + sidelen*sin(rngr->pose.a),
          rngr->pose.a, str);
    }

  return 0;
}


int ranger_unrender_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  stg_model_fig_clear( mod, "ranger_data_fig" );
  return 1; // quit callback
}


int ranger_render_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  stg_rtk_fig_t* fig;
  size_t clen=0, dlen;
  int rcount;
  stg_ranger_sample_t *samples ;
  stg_ranger_config_t* cfg;

  PRINT_DEBUG( "ranger render data" );
  
  fig = stg_model_get_fig( mod, "ranger_data_fig" );

  if( !fig )
    fig = stg_model_fig_create( mod, "ranger_data_fig", "top", STG_LAYER_RANGERDATA );

  stg_rtk_fig_clear(fig);

  cfg = stg_model_get_property(mod, "ranger_cfg", &clen );
  
  // any samples at all?
  if( clen < sizeof(stg_ranger_config_t) )
    return 0;
  
  rcount = clen / sizeof( stg_ranger_config_t );
  
  if( rcount < 1 ) // no samples 
    return 0; 
  
  dlen = 0;
  samples = 
    stg_model_get_property( mod, "ranger_data", &dlen );
  
  // iff we have the right amount of data
  if( dlen == rcount * sizeof(stg_ranger_sample_t) )
    {       
      stg_geom_t geom;
      int s;
      stg_color_t *col;
      stg_model_get_geom(mod,&geom);
      
      col = stg_model_get_property_fixed(mod, "ranger_data_color", sizeof(stg_color_t));
      stg_rtk_fig_color_rgb32(fig, *col);

      stg_rtk_fig_origin( fig, geom.pose.x, geom.pose.y, geom.pose.a );	  
      
      // draw the range  beams
      for( s=0; s<rcount; s++ )
      {
        stg_ranger_config_t* rngr = &cfg[s];
        if( samples[s].range > 0.0 && rngr->fov != 0) // fov==0 means nonexistant sonar
        {
          stg_rtk_fig_arrow( fig, 
           rngr->pose.x, rngr->pose.y, rngr->pose.a, 	
           samples[s].range, 0.02 );
        }
      }

#if 0
      /* For debugging -- compare against Raytrace Debug */
      {
        for(s = 0; s < rcount; s++)
        {
          if(samples[s].intersect_flag)
          {
            stg_ranger_config_t* rngr = &cfg[s];
            char str[6];
            snprintf(str, 6, "%0.1f", RTOD(samples[s].intersect_angle));
            stg_rtk_fig_text(fig, 
              rngr->pose.x + ( 0.2 + samples[s].range ) * cos(rngr->pose.a),
              rngr->pose.y + ( 0.2 + samples[s].range ) * sin(rngr->pose.a),
              rngr->pose.a, str);
          }
        }
      }
#endif



    }
  else
    if( dlen > 0 )
      PRINT_WARN2( "data size doesn't match configuation (%lu/%lu bytes)",
		   dlen,  rcount * sizeof(stg_ranger_sample_t) );
  
  return 0; // keep running
}


