

#include "gui.h"

//#define DEBUG 1
//#define DEBUG_LASER_RULES 1

#include "stage_internal.h"
#include <sys/time.h>
#include <math.h>

extern stg_rtk_fig_t* fig_debug_rays;

#define TIMING 0
#define LASER_FILLED 1

#define STG_LASER_WATTS 17.5 // laser power consumption

#define STG_LASER_SAMPLES_MAX 1024
#define STG_DEFAULT_LASER_SIZEX 0.15
#define STG_DEFAULT_LASER_SIZEY 0.15
#define STG_DEFAULT_LASER_MINRANGE 0.0
#define STG_DEFAULT_LASER_MAXRANGE 8.0
#define STG_DEFAULT_LASER_FOV M_PI
#define STG_DEFAULT_LASER_SAMPLES 180
#define STG_DEFAULT_LASER_REVERSE_SCAN 0
#define STG_DEFAULT_LASER_NOISE 0.0
#define STG_DEFAULT_LASER_READING_ANGLE_ERROR 0.0

/**
@ingroup model
@defgroup model_laser Laser model 
The laser model simulates a scanning laser rangefinder

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
laser
(
  # Laser properties:
  samples 180
  range_min 0.0
  range_max 8.0
  fov 3.141593
  noise 0.0
  reading_angle_error 0.0

  # Model properties:
  size [0.15 0.15]
  color "blue"
)
@endverbatim

@par Details
- samples int
  - the number of laser samples per scan
- range_min float
  -  the minimum range reported by the scanner, in meters. The scanner will detect objects closer than this, but report their range as the minimum.
- range_max float
  - the maximum range reported by the scanner, in meters. The scanner will not detect objects beyond this range.
- fov float
  - the angular field of view of the scanner, in radians
- noise [double]
  - range of random error (meters) added to each reading (ranges from -noise .. +noise)
- reading_angle_error [double]
  - range of random error (radians) added to the angle at which each reading is taken 
    (error will range from -reading_angle_error .. +reading_angle_error)


- Laser return value modification rules:

  Every model has a laser_return value. The laser_return_rule properties of the laser let 
  you modify that return value based on angle and range.  Each property in the list is a
  rule providing a detected return value based on the model's original laser_return
  value and a range condition (angle of incidence and distance).  If a laser reading
  being taken satisfies a rule's condition, and the intersected model has the rule's required
  return value, then detect_val is used instead of the model's value. 

  For example, the following two rules simulate the SICK LMS-200's reflectance detection: 
  Even if a model's return type is 2 or greater (high reflectance), the SICK will only see it as such
  if its beam hits the object within +/- 30 degrees from normal, and it's within 8 meters.

  laser_return_rules = 2  # Number of rules to try to read from the file.  

  laser_return_rule[0].model_gt 1
  laser_return_rule[0].condition  "outside_range"
  laser_return_rule[0].range  8
  laser_return_rule[0].detect  1  # Detect it as a 1 instead of a 2!
  laser_return_rule[0].stop 1     # 1 to stop evaluating rules, 0 to continue (default)

  laser_return_rule[1].model_gt 1
  laser_return_rule[1].condition  "outside_angle"
  laser_return_rule[1].angle  60
  laser_return_rule[1].detect 1  # Detect it as a 1 instead of a 2!

  The following rules are like the above, but would make a very bright reflector (3 or 4) become a 
  (2) outside +/- 5 degrees or 6 meters. (The SICK doesn't do this really, this is an imaginary example.)

  laser_return_rules = 6  # Number of rules 

  laser_return_rule[0].model_eq 3
  laser_return_rule[0].condition "outside_range"
  laser_return_rule[0].range 6
  laser_return_rule[0].detect 2  # Detect it as a 2

  laser_return_rule[1].model_eq 3
  laser_return_rule[1].condition "outside_angle"
  laser_return_rule[1].angle 10 
  laser_return_rule[1].detect 2  # Detect it as a 2

  laser_return_rule[2].model_eq 4
  laser_return_rule[2].condition "outside_range"
  laser_return_rule[2].range 6 
  laser_return_rule[2].detect 2  # Detect it as a 2

  laser_return_rule[3].model_eq 4
  laser_return_rule[3].condition "outside_angle"
  laser_return_rule[3].angle 10 
  laser_return_rule[3].detect 2  # Detect it as a 2

  laser_return_rule[4].model_eq 2
  laser_return_rule[4].condition "outside_range"
  laser_return_rule[4].range 8
  laser_return_rule[4].detect 1

  laser_return_rule[5].model_eq 2
  laser_return_rule[5].condition "outside_angle"
  laser_return_rule[5].except_region 30
  laser_return_rule[5].detect 1

  The following rule simulates a laser that is unable to see things at very acute angles

  laser_return_rules = 1
  laser_return_rule[0].model_gt 0   # Any value
  laser_return_rule[0].condition "outside_angle"
  laser_return_rule[0].angle 160
  laser_return_rule[0].detect 0


  The following rule simply turns any model with value 2 into 33
  laser_return_rules = 1
  laser_return_rule[0].model_eq 2
  laser_return_rule[0].detect 33

*/

/** 
@ingroup stg_model_laser
@ingroup stg_model_props
@defgroup stg_model_laser_props Laser Properties

- "laser_cfg" stg_laser_config_t
- "laser_data" stg_laser_sample_t[]
- "laser_return" stg_laser_return_t
[DISABLED] - "laser_beam_height" stg_meters_t
*/

void laser_load( stg_model_t* mod, int wf_id )
{
  int nrules;
  char key[32];
  int i;
  stg_laser_rule_t* prevrule;
  const char* cond_str;

  stg_laser_config_t lconf;
  stg_laser_config_t* now = 
    stg_model_get_property_fixed( mod, "laser_cfg", sizeof(stg_laser_config_t)); 

  memset( &lconf, 0, sizeof(lconf));

  lconf.samples   = wf_read_int( wf_id, "samples", now ? now->samples : STG_DEFAULT_LASER_SAMPLES );
  lconf.range_min = wf_read_length( wf_id, "range_min", now ? now->range_min : STG_DEFAULT_LASER_MINRANGE );
  lconf.range_max = wf_read_length( wf_id, "range_max", now ? now->range_max : STG_DEFAULT_LASER_MAXRANGE );
  lconf.fov       = wf_read_angle( wf_id, "fov",  now ? now->fov : STG_DEFAULT_LASER_FOV );
  lconf.reverse_scan = wf_read_int( wf_id, "reverse_scan", now ? now->reverse_scan :  STG_DEFAULT_LASER_REVERSE_SCAN);

#ifdef ENABLE_LASER_NOISE
  lconf.noise = wf_read_float(wf_id, "noise", now ? now->noise : STG_DEFAULT_LASER_NOISE);
  lconf.reading_angle_error = wf_read_float(wf_id, "reading_angle_error", now ? now->reading_angle_error : STG_DEFAULT_LASER_READING_ANGLE_ERROR);
#else
  lconf.noise = 0;
  lconf.reading_angle_error = 0;
#endif


  nrules = wf_read_int(wf_id, "laser_return_rules", 0);
  prevrule = lconf.rules;
  for(i = 0; i < nrules; ++i)
  {
    stg_laser_rule_t* newrule = (stg_laser_rule_t*)malloc(sizeof(stg_laser_rule_t));
    assert(newrule);
    newrule->next = NULL;

    // Model value conditions
    snprintf(key, 32, "laser_return_rule[%i].model_eq", i);
    newrule->model_value_eq = wf_read_int(wf_id, key, -1);
    snprintf(key, 32, "laser_return_rule[%i].model_gt", i);
    newrule->model_value_gt = wf_read_int(wf_id, key, -1);
    snprintf(key, 32, "laser_return_rule[%i].model_gt", i);
    newrule->model_value_lt = wf_read_int(wf_id, key, -1);

    // (backwards compatability:)
    if(newrule->model_value_eq == -1)
    {
      snprintf(key, 32, "laser_return_rule[%i].model", i);
      newrule->model_value_eq = wf_read_int(wf_id, key, -1);
    }

    // Detection value
    snprintf(key, 32, "laser_return_rule[%i].detect", i);
    newrule->result = STG_LASER_RULE_RETURN_VALUE;   // only result currently
    newrule->result_value.detect = wf_read_int(wf_id, key, -1);
    if(newrule->result_value.detect == -1)
    {
      PRINT_ERR1("Error: no detection-value specified in laser_return_rule[%d]! Skipping rule.\n", i);
      free(newrule);
      continue;
    }

    snprintf(key, 32, "laser_return_rule[%i].stop", i);
    newrule->stop = wf_read_int(wf_id, key, 0);
    
    // Condition
    snprintf(key, 32, "laser_return_rule[%i].condition", i);
    cond_str = wf_read_string(wf_id, key, NULL);
    if(cond_str == NULL || strcmp(cond_str, "none") == 0)
      newrule->condition = STG_LASER_RULE_COND_NONE;
    else if(strcmp(cond_str, "outside_range") == 0)
      newrule->condition = STG_LASER_RULE_COND_OUTSIDE_RANGE;
    else if(strcmp(cond_str, "outside_angle") == 0)
      newrule->condition = STG_LASER_RULE_COND_OUTSIDE_ANGLE;
    else
    {
      PRINT_ERR2("Error: Unrecognized condition \"%s\" specificed in laser_return_rule[%d]! Skipping this rule.\n", cond_str, i);
      free(newrule);
      continue;
    }

    // Condition value
    switch(newrule->condition)
    {
      case STG_LASER_RULE_COND_OUTSIDE_RANGE:
        snprintf(key, 32, "laser_return_rule[%i].range", i);
        newrule->condition_value.range = wf_read_float(wf_id, key, -1);
        if(newrule->condition_value.range < 0)
        {
          PRINT_ERR2("Error: Invalid range %f (or not given) in laser_return_rule[%d] (must be >= 0). Skipping this rule.\n", newrule->condition_value.range, i);
          free(newrule);
          continue;
        }
        break;
      case STG_LASER_RULE_COND_OUTSIDE_ANGLE:
        {
          double angle_deg;
          snprintf(key, 32, "laser_return_rule[%i].angle", i);
          angle_deg = wf_read_float(wf_id, key, -1);
          if(angle_deg < 0)
          {
            PRINT_ERR2("Error: Invalid angle %f (or not given) in laser_return_rule[%d] (must be >= 0). Skipping this rule.\n", angle_deg, i);
            free(newrule);
            continue;
          }
          newrule->condition_value.angle = DTOR(angle_deg);
        }
        break;

      case STG_LASER_RULE_COND_NONE:
        break;
    }

    if(prevrule == NULL)
      lconf.rules = newrule;
    else
      prevrule->next = newrule;
    prevrule = newrule;
  }
  
  stg_model_set_property( mod, "laser_cfg", &lconf, sizeof(lconf));

  {
    stg_meters_t lbh = 0.0;
    if(stg_model_has_height(mod))
      lbh = stg_model_get_height(mod) / 2.0;
    lbh = wf_read_float(wf_id, "laser_beam_height", lbh);
    stg_model_set_property(mod, "laser_beam_height", &lbh, sizeof(lbh));
  }
}

int laser_update( stg_model_t* mod );
int laser_startup( stg_model_t* mod );
int laser_shutdown( stg_model_t* mod );
void laser_destroy( stg_model_t* mod );

int laser_render_data( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );
int laser_unrender_data( stg_model_t* mod, char* name,
			 void* data, size_t len, void* userp );
int laser_render_cfg( stg_model_t* mod, char* name, 
		      void* data, size_t len, void* userp );
//int laser_unrender_cfg( stg_model_t* mod, char* name,
//		void* data, size_t len, void* userp );


// not looking up color strings every redraw makes Stage 6% faster! (scanf is slow)
static int init = 0;
static stg_color_t laser_color=0, bright_color=0, fill_color=0, cfg_color=0, geom_color=0;


int laser_init( stg_model_t* mod )
{
  // seed random number generator 
  // this is now done in stg_init().
  //static int first_time = 1;
  //if(first_time)
  //{
  //  first_time = 0;
  //  srand(time(NULL));
  //}

  // we do this just the first time a laser is created
  if( init == 0 )
    {
      laser_color =   stg_lookup_color(STG_LASER_COLOR);
      bright_color = stg_lookup_color(STG_LASER_BRIGHT_COLOR);
      fill_color =  stg_lookup_color(STG_LASER_FILL_COLOR);
      geom_color = stg_lookup_color(STG_LASER_GEOM_COLOR);
      cfg_color = stg_lookup_color(STG_LASER_CFG_COLOR);
      init = 1;
    }

  // we don't consume any power until subscribed
  //mod->watts = 0.0; 
  
  // override the default methods
  mod->f_startup = laser_startup;
  mod->f_shutdown = laser_shutdown;
  mod->f_update =  NULL; // laser_update is installed startup, removed on shutdown
  mod->f_load = laser_load;
  mod->f_destroy = laser_destroy;

  // sensible laser defaults 
  {
    stg_geom_t geom; 
    geom.pose.x = 0.0;
    geom.pose.y = 0.0;
    geom.pose.a = 0.0;
    geom.size.x = STG_DEFAULT_LASER_SIZEX;
    geom.size.y = STG_DEFAULT_LASER_SIZEY;
    stg_model_set_property( mod, "geom", &geom, sizeof(geom) );
  }
      
  
  // create a single rectangle body 
  {
    stg_polygon_t* square = stg_unit_polygon_create();
    stg_model_set_property( mod, "polygons", square, sizeof(stg_polygon_t));
    // set_property copied the data, we can free it; in future,
    free(square);
  }


  // set up a laser-specific config structure
  {
    stg_laser_config_t lconf;
    memset(&lconf,0,sizeof(lconf));  
    lconf.range_min   = STG_DEFAULT_LASER_MINRANGE;
    lconf.range_max   = STG_DEFAULT_LASER_MAXRANGE;
    lconf.fov         = STG_DEFAULT_LASER_FOV;
    lconf.samples     = STG_DEFAULT_LASER_SAMPLES;  
    lconf.reverse_scan = STG_DEFAULT_LASER_REVERSE_SCAN;
    lconf.noise       = STG_DEFAULT_LASER_NOISE;
    lconf.reading_angle_error = STG_DEFAULT_LASER_READING_ANGLE_ERROR;
    stg_model_set_property( mod, "laser_cfg", &lconf, sizeof(lconf) );
  }
  
  
  // set default color
  stg_model_set_property( mod, "color", &geom_color, sizeof(geom_color));
  
  // when data is set, render it to the GUI
  // clear the data - this will unrender it too
  stg_model_set_property( mod, "laser_data", NULL, 0 );

  // adds a menu item and associated on-and-off callbacks
  if(mod->world->gui_enabled)
    stg_model_add_property_toggles( mod, "laser_data", 
				  laser_render_data, // called when toggled on
				  NULL, 
				  laser_unrender_data, // called when toggled off
				  NULL, 
				  "Laser Data",
				  TRUE );

  // TODO
  /* stg_model_add_property_toggles( mod, "laser_cfg",  */
/* 				  laser_render_cfg, // called when toggled on */
/* 				  NULL, */
/* 				  NULL,//stg_fig_clear_cb, */
/* 				  NULL, //gui->cfg,  */
/* 				  "laser config", */
/* 				  TRUE ); */
  
  return 0;
}

void laser_destroy(stg_model_t* mod)
{
  stg_laser_config_t *cfg;

  if(mod->world->gui_enabled)
    stg_model_remove_property_toggles(mod, "laser_data");

  // Destroy laser rules allocated in laser_load:
  cfg = stg_model_get_property_fixed(mod, "laser_cfg", sizeof(stg_laser_config_t));
  if(cfg) 
  {
    int i = 0;
    stg_laser_rule_t *rule = cfg->rules;
    while(rule != NULL)
    {
      stg_laser_rule_t *nextrule = rule->next;
      free(rule);
      rule = nextrule;
      ++i;
    }
  }

  // Destroy scan data array allocated in laser_update
  stg_laser_sample_t *data = stg_model_get_property(mod, "laser_data", NULL);
  if(data)
    free(data);

}

int laser_raytrace_match( stg_model_t* mod, stg_model_t* hitmod )
{           
  stg_laser_return_t* lr;

  if(!mod) return 0;
  if(!hitmod) return 0;

  lr = stg_model_get_property_fixed( hitmod, 
				       "laser_return", 
				       sizeof(stg_laser_return_t));
  
  // Ignore myself, my children, and my ancestors, and undetectable objects
  if( (lr == NULL) || (*lr == 0) || (stg_model_is_related(mod, hitmod)) )
    return 0;
  else
    return 1;
  
  // Stop looking when we see something
  //hisreturn = hitmdmodel_laser_return(hitmod);
  
  return 0; // no match
}	

int laser_raytrace_match_height(stg_model_t *lasermod, stg_model_t *hitmod)
{
  stg_laser_return_t *lr = stg_model_get_property_fixed(hitmod, "laser_return", sizeof(stg_laser_return_t));
  stg_meters_t *laser_beam_height = NULL;

  // Ignore myself, my children, and my ancestors, undetectable objects
  if(stg_model_is_related(lasermod, hitmod) || *lr == 0)
    return 0;

  // ignore objects that are too low
  if( stg_model_has_height(hitmod) && stg_model_has_height(lasermod) && ((laser_beam_height = stg_model_get_property_fixed(lasermod, "laser_beam_height", sizeof(stg_meters_t))) != NULL) )
  {
    stg_meters_t hiw = stg_model_get_height_in_world(lasermod);
    stg_meters_t h = stg_model_get_height(lasermod);
    stg_meters_t bhiw = *laser_beam_height + (hiw - h);
    //printf("laser beam height %0.3f compared against model %s height %0.3f...\n", bhiw, hitmod->token, stg_model_get_height_in_world(hitmod));
    if(stg_model_get_height_in_world(hitmod) < bhiw)
      return 0;
  }

  return 1;
}

int laser_update( stg_model_t* mod )
{   
  stg_laser_config_t* cfg;
  stg_geom_t geom;
  stg_pose_t pz;
  double sample_incr, bearing;
  static stg_laser_sample_t* scan = 0;  // keep samples in a static buffer
  int t;
 
  // reading indices (for reverse_scan)
  int first;
  int next;

  double reading_angle_error;
  itl_t *itl;

  //puts( "laser update" );

  PRINT_DEBUG2( "[%lu] laser update (%d subs)", mod->world->sim_time, mod->subs );
  
  // no work to do if we're unsubscribed
  if( mod->subs < 1 )
    return 0;
  
  cfg = 
    stg_model_get_property_fixed( mod, "laser_cfg", sizeof(stg_laser_config_t));
  assert(cfg);

  if(cfg->samples == 0)
  {
    if( fig_debug_rays ) stg_rtk_fig_clear( fig_debug_rays );
    if(scan)
      free(scan);
    scan = NULL;
    stg_model_set_property( mod, "laser_data", NULL, 0);
    return 0;
  }

  stg_model_get_geom( mod, &geom );

  // get the sensor's pose in global coords
  memcpy( &pz, &geom.pose, sizeof(pz) ); 
  stg_model_local_to_global( mod, &pz );

  PRINT_DEBUG3( "laser origin %.2f %.2f %.2f", pz.x, pz.y, pz.a );
  //stg_print_msg( "%s: laser origin %.2f %.2f %.2f", mod->token, pz.x, pz.y, pz.a );

  sample_incr = cfg->fov / (double)cfg->samples;
  bearing = pz.a - cfg->fov/2.0;
  
#if TIMING
  struct timeval tv1, tv2;
  gettimeofday( &tv1, NULL );
#endif
      
  if( fig_debug_rays ) stg_rtk_fig_clear( fig_debug_rays );


  // make a scan buffer (static for speed, so we only have to allocate
  // memory when the number of samples changes).
  scan = realloc( scan, sizeof(stg_laser_sample_t) * cfg->samples );
  assert(scan);
  
  // only compute every second sample, for speed
  //for( t=0; t<cfg.samples-1; t+=2 )
  

  //memset( scan, 0, sizeof(stg_laser_sample_t) * cfg.samples );
  //for( t=0; t<1; t++ )
  if(cfg->reverse_scan)
  {
    first = (cfg->samples) - 1;
    next = -1;
  }
  else
  {
    first = 0;
    next = 1;
  }


#if ENABLE_LASER_NOISE
  //reading_angle_error = ((double)(rand() - RAND_MAX/2) / (double)RAND_MAX) * cfg->reading_angle_error;
  double err = cfg->reading_angle_error;
  reading_angle_error = STG_RANDOM_IN(-err, err);
#else
  reading_angle_error = 0;
#endif
  stg_matrix_lock(mod->world->matrix);
  itl = itl_create(pz.x, pz.y, bearing+reading_angle_error, cfg->range_max, mod->world->matrix, PointToBearingRange);
  bearing += sample_incr;
  for(t = first; (cfg->reverse_scan  && t >= 0) || (!cfg->reverse_scan && t < cfg->samples) ; t += next)
  {
    stg_model_t* hitmod = NULL;
    double range = 0.0;

    //if(mod->world->use_model_height && stg_model_has_height(mod))
    //{
    //  hitmod = itl_first_matching(itl, laser_raytrace_match_height, mod, mod->current_matrix_cell);
    //}
    //else
    //{
      hitmod = itl_first_matching( itl, laser_raytrace_match, mod);
    //}
    stg_matrix_unlock(mod->world->matrix);


    if( hitmod )
      range = itl->range;
    else
      range = cfg->range_max;

    //printf( "%d:%.2f  ", t, range );

    if( range < cfg->range_min )
      range = cfg->range_min;
          
    
#ifdef ENABLE_LASER_NOISE
    // add some random noise except in special max. range condiion
    if(range < cfg->range_max)
    {
      double noise = cfg->noise;
      range += STG_RANDOM_IN(-noise, noise);
      //range += ((double)(rand() - RAND_MAX/2) / (double)RAND_MAX) * cfg->noise;
    }
#endif

    // record the range in mm
    //scan[t+1].range = 
    scan[t].range = (uint32_t)( range * 1000.0 );

    if( hitmod )
    {
      // evaluate rules to determine the return value (a/k/a reflectance).
      stg_laser_return_t *hitmod_lr_p;
      gboolean stop = FALSE;
      stg_laser_rule_t* rule;
      stg_laser_return_t ret;

      
      // This is the model's "natural" value:
      stg_model_lock(hitmod);
      hitmod_lr_p = stg_model_get_property_fixed( hitmod, "laser_return", sizeof(stg_laser_return_t));
      if(hitmod_lr_p)
        ret = (*hitmod_lr_p);
      else
      {
        stg_print_error ("Internal warning: laser hit model %s, but it has a NULL laser_return property (or model data is corrupted...)! Using return value of 1.", hitmod->token);
        ret = 1;
      }
      stg_model_unlock(hitmod);

#ifdef DEBUG_LASER_RULES
      printf("Stage laser: Hit a model with return value set to %d. Checking our rules...\n", ret);
#endif

      for(rule = cfg->rules; rule != NULL && !stop; rule = rule->next)
      {
        // model_value_* are the primary prerequisites. ignore this rule if none
        // of the given (not -1) model value prerequisites match.
        if(!(  ( rule->model_value_eq != -1 && ret == rule->model_value_eq ) 
            || ( rule->model_value_gt != -1 && ret > rule->model_value_gt )  
            || ( rule->model_value_lt != -1 && ret < rule->model_value_lt ) 
          ))
        {
          continue;
        }

#ifdef DEBUG_LASER_RULES
        printf("Stage laser: Rule cond matches: cond=%d (%s), model_eq=%d,gt=%d,lt=%d, stop=%d, detect_as=%d.\n", 
          rule->condition, ((rule->condition == STG_LASER_RULE_COND_OUTSIDE_RANGE)?"outside range":((rule->condition == STG_LASER_RULE_COND_OUTSIDE_ANGLE)?"outside range":"none")), 
          rule->model_value_eq, rule->model_value_gt,
          rule->model_value_lt, rule->result_value.detect, ret);
#endif

        switch(rule->condition)
        {
          case STG_LASER_RULE_COND_OUTSIDE_RANGE:
            if(range > rule->condition_value.range)
            {
              ret = rule->result_value.detect;
            }
            break;
          case STG_LASER_RULE_COND_OUTSIDE_ANGLE:
            {
              if(fabs(stg_intersection_angle(itl->obs_angle, itl->a)) > (rule->condition_value.angle / 2.0))
              {
                ret = rule->result_value.detect;
              }
            }
            break;
          case STG_LASER_RULE_COND_NONE:
            {
              ret = rule->result_value.detect;
            }
            break;
        }
        stop = rule->stop;
      }

#ifdef DEBUG_LASER_RULES
      printf("Stage laser: Using return value %d\n", ret);
#endif
        
      if(ret > 1)
        scan[t].reflectance = ret - 1; // (*lr >= LaserBright) ? 1 : 0;
      else
        scan[t].reflectance = 0;

    }
    else
    {
      // ray didn't hit any other model.
      scan[t].reflectance = 0;
    }
      

    stg_matrix_lock(mod->world->matrix);

#if ENABLE_LASER_NOISE
    double err = cfg->reading_angle_error;
    //reading_angle_error = ((double)(rand() - RAND_MAX/2) / (double)RAND_MAX) * cfg->reading_angle_error;
    reading_angle_error = STG_RANDOM_IN(-err, err);
#endif  // else, reading_angle_error is already 0 from before
    itl_reset(itl, pz.x, pz.y, bearing+reading_angle_error, cfg->range_max, PointToBearingRange);
    bearing += sample_incr;
  }

#ifdef DEBUG_LASER_RULES
  puts(""); // blank line between sets of scans
#endif

  itl_destroy( itl );
  stg_matrix_unlock(mod->world->matrix);
  
  // new style
  stg_model_set_property( mod, "laser_data", scan, sizeof(stg_laser_sample_t) * cfg->samples);
  

#if TIMING
  gettimeofday( &tv2, NULL );
  printf( " laser data update time %.6f\n",
	  (tv2.tv_sec + tv2.tv_usec / 1e6) - 
	  (tv1.tv_sec + tv1.tv_usec / 1e6) );	    
#endif

  return 0; //ok
}


int laser_unrender_data( stg_model_t* mod, char* name, 
			 void* data, size_t len, void* userp )
{
  stg_model_fig_clear( mod, "laser_data_fig" );
  stg_model_fig_clear( mod, "laser_data_bg_fig" );  
  return 1; // callback just runs one time
}

int laser_render_data( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp )
{
  stg_laser_config_t *cfg;
  stg_laser_sample_t* samples;
  size_t sample_count;
  stg_rtk_fig_t* bg = stg_model_get_fig( mod, "laser_data_bg_fig" );
  stg_rtk_fig_t* fg = stg_model_get_fig( mod, "laser_data_fig" );  
  
  if( ! bg )
    bg = stg_model_fig_create( mod, "laser_data_bg_fig", "top", STG_LAYER_LASERDATA );
  
  if( ! fg )
    fg = stg_model_fig_create( mod, "laser_data_fig", "top", STG_LAYER_LASERDATA );  
  
  stg_rtk_fig_clear( bg );
  stg_rtk_fig_clear( fg );

  
  cfg = 
    stg_model_get_property_fixed( mod, "laser_cfg", sizeof(stg_laser_config_t));
  assert( cfg );
  
  
  samples = (stg_laser_sample_t*)data; 
  sample_count = len / sizeof(stg_laser_sample_t);
  
  if( samples && sample_count )
    {
      // now get on with rendering the laser data
      stg_pose_t pose;
      stg_geom_t geom;
      double sample_incr, bearing;
      stg_point_t* points;
      int s;
      int p;
      int first;
      int next;

      stg_model_get_global_pose( mod, &pose );
      stg_model_get_geom( mod, &geom );
      sample_incr = cfg->fov / sample_count;
      bearing = geom.pose.a - cfg->fov/2.0;
      points = calloc( sizeof(stg_point_t), sample_count + 1 );
      
      //stg_rtk_fig_origin( fg, pose.x, pose.y, pose.a );  
      stg_rtk_fig_color_rgb32( fg, bright_color );
      // TODO: menu item to choose to draw data unflipped?
      if(cfg->reverse_scan)
      {
        first = sample_count - 1;
        next = -1;
      }
      else
      {
        first = 0;
        next = 1;
      }
      for( p=1, s=first; p <= sample_count; s += next, p++ )
	{
	  // useful debug:
	  //stg_rtk_fig_arrow( bg, 0, 0, bearing, (samples[s]->range/1000.0), 0.01 );
 
    double sinb, cosb;
    STG_SINCOS(bearing, sinb, cosb);
    points[p].x = cosb * (samples[s].range/1000.0);
    points[p].y = sinb * (samples[s].range/1000.0);
	  bearing += sample_incr;
	}
      
      
  // give polygons to rtk. note: we assume that stg_rtk_point_t is equivalent to stg_point_t. (both are structs containing pair of doubles x and y.)
  if( mod->world->win->fill_polygons )
	{
	  stg_rtk_fig_color_rgb32( bg, fill_color );
	  stg_rtk_fig_polygon( bg, 0,0,0, sample_count+1, (stg_rtk_point_t*)points, TRUE );
	}
      
  stg_rtk_fig_color_rgb32( fg, laser_color );
  stg_rtk_fig_polygon( fg, 0,0,0, sample_count+1, (stg_rtk_point_t*)points, FALSE ); 	
      
  // loop through again, drawing bright boxes on top of the polygon
  for( s=0; s<sample_count; s++ )
	{      
	  // if this hit point is bright, we draw a little box
	  if( samples[s].reflectance > 0 )
	    {
	      stg_rtk_fig_color_rgb32( fg, bright_color );
	      stg_rtk_fig_rectangle( fg, 
				     points[1+s].x, points[1+s].y, 0,
				     0.10, 0.10, 1 );
	      stg_rtk_fig_color_rgb32( fg, laser_color );
	    }
	}
      
      free( points );
    }
  return 0; // callback runs until removed
}

int laser_render_cfg( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  //puts( "laser render cfg" );
  stg_laser_config_t* cfg;
  double mina, maxa, sinmina, cosmina, sinmaxa, cosmaxa;
  double leftfarx, leftfary, rightfarx, rightfary, leftnearx, leftneary,
    rightnearx, rightneary;

  stg_rtk_fig_t* fig = stg_model_get_fig( mod, "laser_cfg_fig" );
  
  if( !fig )
    fig = stg_model_fig_create( mod, "laser_config_fig", "top", STG_LAYER_LASERCONFIG );
  
  stg_rtk_fig_clear(fig);
  
  // draw the FOV and range lines
  stg_rtk_fig_color_rgb32( fig, cfg_color );
  
  cfg = (stg_laser_config_t*)data;
  assert( cfg );
  
  mina = cfg->fov / 2.0;
  maxa = -cfg->fov / 2.0;
  
  STG_SINCOS(mina, sinmina, cosmina);
  STG_SINCOS(maxa, sinmaxa, cosmaxa);
  leftfarx = cfg->range_max * cosmina;
  leftfary = cfg->range_max * sinmina;
  rightfarx = cfg->range_max * cosmaxa;
  rightfary = cfg->range_max * sinmaxa;  
  leftnearx = cfg->range_min * cosmina;  
  leftneary = cfg->range_min * sinmina;
  rightnearx = cfg->range_min * cosmaxa;
  rightneary = cfg->range_min * sinmaxa;
  
  stg_rtk_fig_line( fig, leftnearx, leftneary, leftfarx, leftfary );
  stg_rtk_fig_line( fig, rightnearx, rightneary, rightfarx, rightfary );
  
  stg_rtk_fig_ellipse_arc( fig,0,0,0, 
			   2.0*cfg->range_max,
			   2.0*cfg->range_max, 
			   mina, maxa );      
  
  stg_rtk_fig_ellipse_arc( fig,0,0,0, 
			   2.0*cfg->range_min,
			   2.0*cfg->range_min, 
			   mina, maxa );      
  return 0;
}


/* int stg_model_unrender_cfg( stg_model_t* mod, char* name,  */
/* 			    void* data, size_t len, void* userp ) */
/* { */
/*   gui_model_t* gui =  */
/*     stg_model_get_property_fixed( mod, "gui", sizeof(gui_model_t)); */
  
/*   if( gui && gui->cfg  ) */
/*     stg_rtk_fig_clear(gui->cfg); */
  
/*   return 1; // callback just runs one time */
/* } */

int laser_startup( stg_model_t* mod )
{ 
  PRINT_DEBUG( "laser startup" );
  
  // start consuming power
  //mod->watts = STG_LASER_WATTS;
  
  // install the update function
  mod->f_update = laser_update;

  return 0; // ok
}

int laser_shutdown( stg_model_t* mod )
{ 
  PRINT_DEBUG( "laser shutdown" );
  
  // uninstall the update function
  mod->f_update = NULL;

  // stop consuming power
  //mod->watts = 0.0;
  
  // clear the data - this will unrender it too
  stg_model_set_property( mod, "laser_data", NULL, 0 );

  return 0; // ok
}


