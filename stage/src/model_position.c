///////////////////////////////////////////////////////////////////////////
//
// File: model_laser.c
// Author: Richard Vaughan
// Date: 10 June 2004
//
// Modified Dec. 2005 and later by Reed Hedges/MobileRobots for relative position control, acceleration
// and deceleration, and speed limit configuration, and more
//
///////////////////////////////////////////////////////////////////////////

#define _GNU_SOURCE 1 // to get sincos() in math.h on Linux systems 
                      // (don't worry, the STG_SINCOS macro in stage.h is defined conditionally 
                      // depending on whether sincos() was detected by  configure)

#include <sys/time.h>
#include <math.h>
#include <stdlib.h>

//#define DEBUG

// Print stuff out about acceleration/deceleration calculations: that's every
// update!
//#define ACCEL_DEBUG(x) { x; }
#define ACCEL_DEBUG(x) { }

// Print out current speed configuration every update
//#define DEBUG_SPEED_PROPERTIES 1

// Print property values loaded from world file 
//#define DEBUG_PROPERTY_LOAD 1

#include "stage_internal.h"
#include "gui.h"

// todo move into header?
void stg_model_position_odom_reset( stg_model_t* mod );
void stg_model_position_get_odom( stg_model_t* mod, stg_pose_t* odom );

//extern stg_rtk_fig_t* fig_debug_rays;

/** 
@ingroup model
@defgroup model_position Position model 

The position model simulates a
mobile robot base. It can drive in one of two modes; either
<i>differential</i>, i.e. able to control its speed and turn rate by
driving left and roght wheels like a Pioneer robot, or
<i>omnidirectional</i>, i.e. able to control each of its three axes
independently.

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
position
(
  drive "diff"

  localization "gps"

  # initial position estimate
  localization_origin [ <defaults to model's start pose> ]
 
  # odometry error model parameters, 
  # only used if localization is set to "odom"
  odom_error [0.03 0.03 0.05]

  # upper velocity capability of this robot; it can never go above these limits
  max_speed [3 3 7.85]
  max_accel [2 2 6.28]
  max_decel [2 2 6.28]

  # acceleration and deceleration, changeable by clients
  accel [0.5 0.5 0.8]
  decel [0.5 0.5 1.74]

  # max velocities to use when doing position or relative-position control. 
  # changeable by clients
  default_speed [1.5 1.5 1.74]
)
@endverbatim

@par Note
Since Stage-1.6.5 the odom property has been removed. Stage will generate a warning if odom is defined in your worldfile. See localization_origin instead.

@par Details
- drive "diff" or "omni"
  - select differential-steer mode (like a Pioneer) or omnidirectional mode. Default is "diff"
- localization "gps" or "odom"
  - if "gps" the position model reports its position with perfect accuracy. If "odom", a simple odometry model is used and position data drifts from the ground truth over time. The odometry model is parameterized by the odom_error property. Default is "gps"
- localization_origin [x y theta]
  - set the origin of the localization coordinate system. By default, this is copied from the model's initial pose, so the robot reports its position relative to the place it started out. Tip: If localization_origin is set to [0 0 0] and localization is "gps", the model will return its true global position. This is unrealistic, but useful if you want to abstract away the details of localization. Be prepared to justify the use of this mode in your research! 
- odom_error [x y theta]
  - parameters for the odometry error model used when specifying localization "odom". Each value is the maximum proportion of error in intergrating x, y, and theta velocities to compute odometric position estimate. For each axis, if the the value specified here is E, the actual proportion is chosen at startup at random in the range -E/2 to +E/2. Note that due to rounding errors, setting these values to zero does NOT give you perfect localization - for that you need to choose localization "gps".

- max_speed [x y theta]
  - Upper velocity limit; any greater speed request is capped to these hard limits.
- max_accel [x y theta]
  - Upper acceleration limit; any greater requested acceleration is capped to these hard limits.
- max_decel [x y theta]
  - Upper deceleration limit; any greater requested deceleration is capped to these hard limits.

- accel [x y theta]
  - Acceleration to use when an increase in velocity is requested. Per second.
If any value is 0, instantaneous acceleration will be used.
- decel [x y theta]
  - Deceleration to use when a decrease in velocity is requested. Per second.
If any value is 0, instantaneous deceleration will be used.

- default_speed [x y theta]
  - When a position or relative-position control mode command is performed, limit
speeds to these values (max_speed has priority, though). 
)
@endverbatim
*/

/** 
@ingroup stg_model_position
@ingroup stg_model_props
@defgroup stg_model_position_props Position Properties

- "position_drive" stg_position_drive_mode_t (The current drive mode is)
- "position_data" stg_position_data_t (The current odometry and other output data)
- "position_cmd" stg_position_cmd_t (The current command to act upon)
- "position_speed_config" (Maximum, default speeds; acceleration and deceleration)
- "position_last_pose" stg_pose_t (Internal Use; used only when a relative-movement command is being acted upon to keep track of progress since last update)
- "position_last_update_time" stg_msec_t (Internal use, mostly; Last time we did an update, used to integrate speed and acceleration)

Used still: ?
- "position_rel_trans_x_cmd" int (Internal Use, the current x-axis relative translation command)
- "position_rel_trans_y_cmd" int (Internal Use)
- "position_rel_rot_cmd" int (Internal Use)
*/





  /*** Speed, position and acceleration thresholds. ***/

// closeness thresholds for finishing position control movements
const double close_enough = 0.005; // 5 mm
const double head_close_enough = 0.0003;  // radians 

// Never try to go slower than this when doing rotational position control,
// or we'll *never* get there!
const double head_minimum_speed = 0.007; // r/sec


// Minimum speed differences to bother trying to calculate acceleration for
const double accel_thresh = 0.010;	// m/sec (=10 mm/sec)
const double rot_accel_thresh = 0.05;	// rad/sec (=2.8 degrees)
const double min_speed_thresh = 0.009; // resulting speed (m/sec) below which to just make it be 0


// Battery simulation movement states
enum
{
  BMS_STOPPED,
  BMS_CONSTVEL,
  BMS_ACCEL,
  BMS_DECEL
};

  /*** Defaults for values not given in model definition: ***/

const double STG_POSITION_WATTS_KGMS = 5.0; // cost per kg per meter per second
const double STG_POSITION_WATTS = 10.0; // base cost of position device

const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_X = 0.03;
const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_Y = 0.03;
const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_A = 0.05;

const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_X = -0.03;
const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_Y = -0.03;
const double STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_A = -0.05;

const stg_position_odom_error_mode_t STG_POSITION_DEFAULT_ODOM_ERROR_MODE = STG_POSITION_ODOM_ERROR_CONSTANT;

// Default speed config values.  These are somewhat arbitrary;
// they are based on the internal defaults for a P3DX-sh.
const stg_meters_t STG_POSITION_DEFAULT_MAX_SPEED_X = 3;  // m/s
const stg_meters_t STG_POSITION_DEFAULT_MAX_SPEED_Y = 3;  // m/s
const stg_radians_t STG_POSITION_DEFAULT_MAX_SPEED_A =  7.85;   // radians/s
const stg_meters_t STG_POSITION_DEFAULT_MAX_ACCEL_X = 2;  // m/s/s
const stg_meters_t STG_POSITION_DEFAULT_MAX_ACCEL_Y = 2;  // m/s/s
const stg_radians_t STG_POSITION_DEFAULT_MAX_ACCEL_A = 6.28;   // radians/s/s
const stg_meters_t STG_POSITION_DEFAULT_MAX_DECEL_X = 2;  // -m/s/s
const stg_meters_t STG_POSITION_DEFAULT_MAX_DECEL_Y = 2;  // -m/s/s
const stg_radians_t STG_POSITION_DEFAULT_MAX_DECEL_A = 6.28;   // -radians/s/s
const stg_meters_t STG_POSITION_DEFAULT_SPEED_X = 1.5;  // m/s
const stg_meters_t STG_POSITION_DEFAULT_SPEED_Y = 1.5;  // m/s
const stg_radians_t STG_POSITION_DEFAULT_SPEED_A = 1.75;   // radians/s
const stg_meters_t STG_POSITION_DEFAULT_ACCEL_X = 0.5;  // m/s/s
const stg_meters_t STG_POSITION_DEFAULT_ACCEL_Y = 0.5;  // m/s/s
const stg_radians_t STG_POSITION_DEFAULT_ACCEL_A = 0.8;   // radians/s/s
const stg_meters_t STG_POSITION_DEFAULT_DECEL_X = 0.5;  // -m/s/s
const stg_meters_t STG_POSITION_DEFAULT_DECEL_Y = 0.5;  // -m/s/s
const stg_radians_t STG_POSITION_DEFAULT_DECEL_A = 1.74;   // -radians/s/s


// global odom error mode for all robots:
// todo stage could keep a set of global properties, so applications can access
// them generically, receive updates, display all in a gui, etc.
// if INVALID, then disregard, using model's setting:
stg_position_odom_error_mode_t  force_odom_error_mode_all_models = STG_POSITION_ODOM_ERROR_INVALID;

void  stg_position_force_odom_error_mode_all_models(stg_position_odom_error_mode_t mode)
{
  stg_print_msg("Setting default odometry position error behavior mode to: %s\n", stg_position_odom_error_mode_name(mode));
  force_odom_error_mode_all_models = mode;
}

stg_position_odom_error_mode_t stg_position_get_forced_odom_error_mode_all_models()
{
  return force_odom_error_mode_all_models;
}

void stg_position_choose_new_random_odom_error(stg_position_data_t *data)
{
  assert(data->min_integration_error.x <= data->max_integration_error.x);
  data->integration_error.x = STG_RANDOM_IN(data->min_integration_error.x, data->max_integration_error.x);
  assert(data->min_integration_error.y <= data->max_integration_error.y);
  data->integration_error.y = STG_RANDOM_IN(data->min_integration_error.y, data->max_integration_error.y);
  assert(data->min_integration_error.a <= data->max_integration_error.a);
  data->integration_error.a = STG_RANDOM_IN(data->min_integration_error.a, data->max_integration_error.a);
  PRINT_DEBUG3("chose new random x error in (%f, %f) -->  %f\n", data->min_integration_error.x, data->max_integration_error.x, data->integration_error.x);
  PRINT_DEBUG3("chose new random y error in (%f, %f) -->  %f\n", data->min_integration_error.y, data->may_integration_error.y, data->integration_error.y);
  PRINT_DEBUG3("chose new random angle error in (%f, %f) -->  %f\n", data->min_integration_error.a, data->max_integration_error.a, data->integration_error.a);
}



  /*** Functions follow... ***/
int position_startup( stg_model_t* mod );
int position_shutdown( stg_model_t* mod );
int position_update( stg_model_t* mod );
void position_load( stg_model_t* mod, int wf_id );
int position_render_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp );
int position_unrender_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp );
void position_destroy(stg_model_t *mod);

int position_init( stg_model_t* mod )
{
  static int first_time = 1;

  //printf("created position model");
  PRINT_DEBUG( "created position model" );
  

  if( first_time )
    {
      first_time = 0;
      // seed the RNG on startup
#ifdef HAVE_SRAND48
      srand48( time(NULL) );
#else
      srand(time(NULL));
#endif
    }

  // no power consumed until we're subscribed
  //mod->watts = 0.0; 

  // override the default methods
  mod->f_startup = position_startup;
  mod->f_shutdown = position_shutdown;
  mod->f_update = position_update;
  mod->f_load = position_load;
  mod->f_destroy = position_destroy;

  // sensible position defaults

  {
    stg_velocity_t vel;
    memset( &vel, 0, sizeof(vel));
    stg_model_set_property( mod, "velocity", &vel, sizeof(vel));
  }
    
  {
    stg_blob_return_t blb = 1;
    stg_model_set_property( mod, "blob_return", &blb, sizeof(blb));
  }
    
  {
    stg_position_drive_mode_t drive = STG_POSITION_DRIVE_DEFAULT;  
    stg_model_set_property( mod, "position_drive", &drive, sizeof(drive) );
  }

  {
    stg_position_stall_t stall = 0;
    stg_model_set_property( mod, "position_stall", &stall, sizeof(stall));
  }
  
  {
    stg_position_cmd_t cmd;
    memset( &cmd, 0, sizeof(cmd));
    cmd.transmode = STG_POSITION_CONTROL_IDLE;
    cmd.rotmode = STG_POSITION_CONTROL_IDLE;
    stg_model_set_property( mod, "position_cmd", &cmd, sizeof(cmd));
  }

  {
      // this is a stupid workaround for the fact that position_update
      // function's changes to the position_cmd property are mysteriously
      // reverted back to its original value somewhere.
      char position_rel_ctrl_active_X = 0;
      char position_rel_ctrl_active_Y = 0;
      char position_rel_ctrl_active_A = 0;
      stg_model_set_property(mod, "position_rel_ctrl_active_X", &position_rel_ctrl_active_X, sizeof(position_rel_ctrl_active_X));
      stg_model_set_property(mod, "position_rel_ctrl_active_Y", &position_rel_ctrl_active_Y, sizeof(position_rel_ctrl_active_Y));
      stg_model_set_property(mod, "position_rel_ctrl_active_A", &position_rel_ctrl_active_A, sizeof(position_rel_ctrl_active_A));
  }

    
  {
    stg_pose_t nothing;
    memset(&nothing, 0, sizeof(nothing));
    stg_model_set_property(mod, "position_last_pose", &nothing, sizeof(nothing));
  }

  {
    stg_pose_t nothing;
    memset(&nothing, 0, sizeof(nothing));
    stg_model_set_property(mod, "position_rel_ctrl_progress", &nothing, sizeof(nothing));
  }

  {
    stg_position_data_t data;
    memset( &data, 0, sizeof(data));


    // TODO should integration_error and other non-changing stuff be moved to
    // separate properties?

    data.integration_error_mode = STG_POSITION_DEFAULT_ODOM_ERROR_MODE;
  
    data.integration_error.x = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_X;
    data.integration_error.y = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_Y;
    data.integration_error.a = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_A;

    data.max_integration_error.x = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_X;
    data.max_integration_error.y = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_Y;
    data.max_integration_error.a = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MAX_A;

    data.min_integration_error.x = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_X;
    data.min_integration_error.y = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_Y;
    data.min_integration_error.a = STG_POSITION_DEFAULT_INTEGRATION_ERROR_MIN_A;


      // this random selection may be done in position_load if mode is RANDOM_INIT.
      // x = STG_RAND()
      // * STG_POSITION_INTEGRATION_ERROR_MAX_X - 
      // STG_POSITION_INTEGRATION_ERROR_MAX_X/2.0;
      // y = STG_RAND()
      // * STG_POSITION_INTEGRATION_ERROR_MAX_Y - 
      // STG_POSITION_INTEGRATION_ERROR_MAX_Y/2.0;
      // a = STG_RAND()
      // * STG_POSITION_INTEGRATION_ERROR_MAX_A - 
      // STG_POSITION_INTEGRATION_ERROR_MAX_A/2.0;
  


    // integration_error is now initialized when reading parameter from
    // worldfile based on error mode.  (TODO do the above if mode is
    // legacy random_init, but no error ranges or values are given.)


    data.localization = STG_POSITION_LOCALIZATION_DEFAULT;

    stg_model_set_property( mod, "position_data", &data, sizeof(data));
  }
  
  if(mod->world->gui_enabled)
  {
    stg_model_add_property_toggles( mod, "position_data",  
 				  position_render_data, // called when toggled on
 				  NULL, 
 				  position_unrender_data, // called when toggled off 
 				  NULL,  
 				  "Position Data",
				  FALSE ); 
  }
 
  // speed limits
  {
    stg_position_speed_config_t speed_cfg;

    speed_cfg.max_speed.x = STG_POSITION_DEFAULT_MAX_SPEED_X;
    speed_cfg.max_speed.y = STG_POSITION_DEFAULT_MAX_SPEED_Y;
    speed_cfg.max_speed.a = STG_POSITION_DEFAULT_MAX_SPEED_A;

    speed_cfg.max_accel.x = STG_POSITION_DEFAULT_MAX_ACCEL_X;
    speed_cfg.max_accel.y = STG_POSITION_DEFAULT_MAX_ACCEL_Y;
    speed_cfg.max_accel.a = STG_POSITION_DEFAULT_MAX_ACCEL_A;

    speed_cfg.max_decel.x = STG_POSITION_DEFAULT_MAX_DECEL_X;
    speed_cfg.max_decel.y = STG_POSITION_DEFAULT_MAX_DECEL_Y;
    speed_cfg.max_decel.a = STG_POSITION_DEFAULT_MAX_DECEL_A;

    speed_cfg.default_speed.x = STG_POSITION_DEFAULT_SPEED_X;
    speed_cfg.default_speed.y = STG_POSITION_DEFAULT_SPEED_Y;
    speed_cfg.default_speed.a = STG_POSITION_DEFAULT_SPEED_A;

    speed_cfg.current_accel.x = STG_POSITION_DEFAULT_ACCEL_X;
    speed_cfg.current_accel.y = STG_POSITION_DEFAULT_ACCEL_Y;
    speed_cfg.current_accel.a = STG_POSITION_DEFAULT_ACCEL_A;

    speed_cfg.current_decel.x = STG_POSITION_DEFAULT_DECEL_X;
    speed_cfg.current_decel.y = STG_POSITION_DEFAULT_DECEL_Y;
    speed_cfg.current_decel.a = STG_POSITION_DEFAULT_DECEL_A;
#ifdef DEBUG_PROPERTY_LOAD
    printf("\nDEBUG setting initial default values in position_speed_config property (before loading files): max speed=[%g,%g,%g] max accel=[%g,%g,%g] "
        "max decel=[%g,%g,%g]; current accel=[%g,%g,%g] decel=[%g,%g,%g]; "
        "default speed=[%g,%g,%g]\n",
        speed_cfg.max_speed.x, speed_cfg.max_speed.y, speed_cfg.max_speed.a, 
        speed_cfg.max_accel.x, speed_cfg.max_accel.y, speed_cfg.max_accel.a, 
        speed_cfg.max_decel.x, speed_cfg.max_decel.y, speed_cfg.max_decel.a, 
        speed_cfg.current_accel.x, speed_cfg.current_accel.y, speed_cfg.current_accel.a, 
        speed_cfg.current_decel.x, speed_cfg.current_decel.y, speed_cfg.current_decel.a, 
        speed_cfg.default_speed.x, speed_cfg.default_speed.y, speed_cfg.default_speed.a
      );
#endif
    stg_model_set_property(mod, "position_speed_config", &speed_cfg, sizeof(speed_cfg));
  }

  // Battery simulation (migrate this to model_battery?)
  {
    //printf("adding battery_state_of_charge\n");
    stg_batt_soc_t initial_soc = 100.0;
    stg_model_set_property(mod, "battery_soc", &initial_soc, sizeof(stg_batt_soc_t));
    stg_model_set_property(mod, "battery_capacity", &initial_soc, sizeof(stg_batt_soc_t));
    stg_charge_rate_t initial_soc_rate = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    stg_model_set_property(mod, "battery_soc_rate", &initial_soc_rate, sizeof(stg_charge_rate_t));
  }
     
  {
    stg_msec_t time = stg_timenow();
    stg_model_set_property(mod, "position_last_update_time", &time, sizeof(time));
  }
  
  return 0;
}

void position_destroy(stg_model_t *mod)
{
  if(mod->world->gui_enabled)
    stg_model_remove_property_toggles(mod, "position_data");
}

void position_load( stg_model_t* mod, int wf_id )
{
  char* keyword = NULL;
  stg_position_data_t* data;

  // load steering mode
  if( wf_property_exists( wf_id, "drive" ) )
    {
      stg_position_drive_mode_t* now;
      stg_position_drive_mode_t drive;
      const char* mode_str;

      now = 
	stg_model_get_property_fixed( mod, "position_drive", 
				     sizeof(stg_position_drive_mode_t));
      
      drive =
	now ? *now : STG_POSITION_DRIVE_DIFFERENTIAL;
      
      mode_str =  
	wf_read_string( wf_id, "drive", NULL );
      
      if( mode_str )
	{
	  if( strcmp( mode_str, "diff" ) == 0 )
	    drive = STG_POSITION_DRIVE_DIFFERENTIAL;
	  else if( strcmp( mode_str, "omni" ) == 0 )
	    drive = STG_POSITION_DRIVE_OMNI;
	  else
	    {
	      PRINT_ERR1( "invalid position drive mode specified: \"%s\" - should be one of: \"diff\", \"omni\". Using \"diff\" as default.", mode_str );	      
	    }	 
	  stg_model_set_property( mod, "position_drive", &drive, sizeof(drive)); 
	}
    }      
  
  data =
    stg_model_get_property_fixed( mod, "position_data", sizeof(stg_position_data_t));
  assert( data );
  
  // load odometry if specified
  if( wf_property_exists( wf_id, "odom" ) )
    {
      PRINT_WARN1( "the odom property is specified for model \"%s\","
		   " but this property is no longer available."
		   " Use localization_origin instead. See the position"
		   " entry in the manual or src/model_position.c for details.", 
		   mod->token );
    }

  // set the starting pose as my initial odom position. This could be
  // overwritten below if the localization_origin property is
  // specified
  stg_model_get_global_pose( mod, &data->origin );

  keyword = "localization_origin"; 
  if( wf_property_exists( wf_id, keyword ) )
    {  
      stg_pose_t gpose;
      double cosa, sina, dx, dy;

      data->origin.x = wf_read_tuple_length(wf_id, keyword, 0, data->pose.x );
      data->origin.y = wf_read_tuple_length(wf_id, keyword, 1, data->pose.y );
      data->origin.a = wf_read_tuple_angle(wf_id, keyword, 2, data->pose.a );

      // compute our localization pose based on the origin and true pose
      stg_model_get_global_pose( mod, &gpose );
      
      data->pose.a = NORMALIZE( gpose.a - data->origin.a );
      STG_SINCOS(data->pose.a, sina, cosa);
      dx = gpose.x - data->origin.x;
      dy = gpose.y - data->origin.y; 
      data->pose.x = dx * cosa + dy * sina; 
      data->pose.y = dy * cosa - dx * sina;

      // zero position error: assume we know exactly where we are on startup
      memset( &data->pose_error, 0, sizeof(data->pose_error));      
    }

  // odometry model parameters


  stg_print_msg("%s: Using \"%s\" odometry error mode.", mod->token, stg_position_odom_error_mode_name(force_odom_error_mode_all_models)); // todo check this model's flag.
  if(wf_property_exists(wf_id, "odom_error"))
  {
    data->integration_error.x = wf_read_tuple_length(wf_id, "odom_error", 0, data->integration_error.x );
    data->max_integration_error.x =  data->integration_error.x;
    data->min_integration_error.x = 0 - data->integration_error.x;

    data->integration_error.y = wf_read_tuple_length(wf_id, "odom_error", 1, data->integration_error.y );
    data->max_integration_error.y = data->integration_error.y;
    data->min_integration_error.y = 0 - data->integration_error.y;

    data->integration_error.a = wf_read_tuple_angle(wf_id, "odom_error", 2, data->integration_error.a );
    data->max_integration_error.a = data->integration_error.a;
    data->min_integration_error.a = 0 - data->integration_error.a;
  }

  if(wf_property_exists(wf_id, "odom_error_x"))
  {
    data->min_integration_error.x = wf_read_tuple_length(wf_id, "odom_error_x", 0, data->min_integration_error.x);
    data->max_integration_error.x = wf_read_tuple_length(wf_id, "odom_error_x", 1, data->max_integration_error.x);
    stg_print_msg("%s: Model definition gives a range of (%f,%f) for odom_error_x\n", mod->token, data->min_integration_error.x, data->max_integration_error.x);
  }
  if(wf_property_exists(wf_id, "odom_error_y"))
  {
    data->min_integration_error.y = wf_read_tuple_length(wf_id, "odom_error_y", 0, data->min_integration_error.y);
    data->max_integration_error.y = wf_read_tuple_length(wf_id, "odom_error_y", 1, data->max_integration_error.y);
    printf("Model definition gives a range of (%f,%f) for odom_error_y\n", data->min_integration_error.y, data->max_integration_error.x);
  }
  if(wf_property_exists(wf_id, "odom_error_a"))
  {
    data->min_integration_error.a = wf_read_tuple_length(wf_id, "odom_error_a", 0, data->min_integration_error.a);
    data->max_integration_error.a = wf_read_tuple_length(wf_id, "odom_error_a", 1, data->max_integration_error.a);
    printf("Model definition gives a range of (%f,%f) for odom_error_a\n", data->min_integration_error.a, data->max_integration_error.a);
  }


  // todo do this whenever min/max integration error or mode properties change
  switch(force_odom_error_mode_all_models) // todo also check this model's flag
  {
    case STG_POSITION_ODOM_ERROR_RANDOM_INIT:
      stg_print_msg("%s: This model has random_init odometry error mode selected, with ranges: x = (%f,%f) m, y = (%f,%f) m, a = (%f,%f) rad",
        mod->token,
        data->min_integration_error.x, data->max_integration_error.x,
        data->min_integration_error.y, data->max_integration_error.y,
        data->min_integration_error.a, data->max_integration_error.a
      );
      stg_position_choose_new_random_odom_error(data);
      stg_print_msg("%s: This robot will have randomly chosen constant odometry error added: (%f m, %f m, %f rad)",
        mod->token,
        data->integration_error.x,
        data->integration_error.y,
        data->integration_error.a);
      break;
    case STG_POSITION_ODOM_ERROR_RANDOM_EACH_UPDATE:
      stg_print_msg("%s: This model has random_each_update odometry error mode selected, with ranges: x = (%f,%f) m, y = (%f,%f) m, a = (%f,%f) rad",
        mod->token,
        data->min_integration_error.x, data->max_integration_error.x,
        data->min_integration_error.y, data->max_integration_error.y,
        data->min_integration_error.a, data->max_integration_error.a
      );
      break;
    case STG_POSITION_ODOM_ERROR_CONSTANT:
      stg_print_msg("%s: This model has constant odometry error mode selected, values (%f m, %f m, %f rad)",
        mod->token,
        data->integration_error.x, data->integration_error.y, data->integration_error.a
      );
      break;
    case STG_POSITION_ODOM_ERROR_NONE:
      stg_print_msg("%s: This model will have no odometry error.", mod->token);
      data->integration_error.x = 0;
      data->integration_error.y = 0;
      data->integration_error.a = 0;
      break;
  }

  // choose a localization model
  if( wf_property_exists( wf_id, "localization" ) )
    {
      const char* loc_str =  
	wf_read_string( wf_id, "localization", NULL );
   
      if( loc_str )
	{
	  if( strcmp( loc_str, "gps" ) == 0 )
	    data->localization = STG_POSITION_LOCALIZATION_GPS;
	  else if( strcmp( loc_str, "odom" ) == 0 )
	    data->localization = STG_POSITION_LOCALIZATION_ODOM;
	  else
	    PRINT_ERR2( "unrecognized localization mode \"%s\" for model \"%s\"."
			" Valid choices are \"gps\" and \"odom\".", 
			loc_str, mod->token );
	}
      else
	PRINT_ERR1( "no localization mode string specified for model \"%s\"", 
		    mod->token );
    }

  // speed limits
  {
    stg_position_speed_config_t* speed_cfg = stg_model_get_property_fixed(mod, "position_speed_config", sizeof(stg_position_speed_config_t));
    assert(speed_cfg);

    speed_cfg->max_speed.x = wf_read_tuple_length(wf_id, "max_speed", 0, STG_POSITION_DEFAULT_MAX_SPEED_X);
    speed_cfg->max_speed.y = wf_read_tuple_length(wf_id, "max_speed", 1, STG_POSITION_DEFAULT_MAX_SPEED_Y);
    speed_cfg->max_speed.a = wf_read_tuple_length(wf_id, "max_speed", 2, STG_POSITION_DEFAULT_MAX_SPEED_A);

    speed_cfg->max_accel.x = wf_read_tuple_length(wf_id, "max_accel", 0, STG_POSITION_DEFAULT_MAX_ACCEL_X);
    speed_cfg->max_accel.y = wf_read_tuple_length(wf_id, "max_accel", 1, STG_POSITION_DEFAULT_MAX_ACCEL_Y);
    speed_cfg->max_accel.a = wf_read_tuple_length(wf_id, "max_accel", 2, STG_POSITION_DEFAULT_MAX_ACCEL_A);

    speed_cfg->max_decel.x = wf_read_tuple_length(wf_id, "max_decel", 0, STG_POSITION_DEFAULT_MAX_DECEL_X);
    speed_cfg->max_decel.y = wf_read_tuple_length(wf_id, "max_decel", 1, STG_POSITION_DEFAULT_MAX_DECEL_Y);
    speed_cfg->max_decel.a = wf_read_tuple_length(wf_id, "max_decel", 2, STG_POSITION_DEFAULT_MAX_DECEL_A);

    speed_cfg->default_speed.x = wf_read_tuple_length(wf_id, "default_speed", 0, STG_POSITION_DEFAULT_SPEED_X);
    speed_cfg->default_speed.y = wf_read_tuple_length(wf_id, "default_speed", 1, STG_POSITION_DEFAULT_SPEED_Y);
    speed_cfg->default_speed.a = wf_read_tuple_length(wf_id, "default_speed", 2, STG_POSITION_DEFAULT_SPEED_A);

    speed_cfg->current_accel.x = wf_read_tuple_length(wf_id, "accel", 0, STG_POSITION_DEFAULT_ACCEL_X);
    speed_cfg->current_accel.y = wf_read_tuple_length(wf_id, "accel", 1, STG_POSITION_DEFAULT_ACCEL_Y);
    speed_cfg->current_accel.a = wf_read_tuple_length(wf_id, "accel", 2, STG_POSITION_DEFAULT_ACCEL_A);

    speed_cfg->current_decel.x = wf_read_tuple_length(wf_id, "decel", 0, STG_POSITION_DEFAULT_DECEL_X);
    speed_cfg->current_decel.y = wf_read_tuple_length(wf_id, "decel", 1, STG_POSITION_DEFAULT_DECEL_Y);
    speed_cfg->current_decel.a = wf_read_tuple_length(wf_id, "decel", 2, STG_POSITION_DEFAULT_DECEL_A);
#ifdef DEBUG_PROPERTY_LOAD
    printf("DEBUG loaded position_speed_config property from world file: max speed=[%g,%g,%g] max accel=[%g,%g,%g] "
        "max decel=[%g,%g,%g]; current accel=[%g,%g,%g] decel=[%g,%g,%g]; "
        "default speed=[%g,%g,%g]\n",
        speed_cfg->max_speed.x, speed_cfg->max_speed.y, speed_cfg->max_speed.a, 
        speed_cfg->max_accel.x, speed_cfg->max_accel.y, speed_cfg->max_accel.a, 
        speed_cfg->max_decel.x, speed_cfg->max_decel.y, speed_cfg->max_decel.a, 
        speed_cfg->current_accel.x, speed_cfg->current_accel.y, speed_cfg->current_accel.a, 
        speed_cfg->current_decel.x, speed_cfg->current_decel.y, speed_cfg->current_decel.a, 
        speed_cfg->default_speed.x, speed_cfg->default_speed.y, speed_cfg->default_speed.a
      );
#endif
    stg_model_property_changed(mod, "poosition_speed_config");
  }

  // we've probably poked the localization data, so we must refresh it
  stg_model_property_changed( mod, "position_data" );


}


// simple linear acceleration or deceleration.
// return 'current' increased or decreased according to 'accel' or 'decel' and 'dt', until
// within 'thresh' of 'desired', then just return 'desired'.
stg_meters_t accelerate(double desired, double current, double accel, double decel, double dt, double thresh, int *accel_state, int print)
{
/*
  ACCEL_DEBUG(
    printf("stg_model_position: requested change from current speed %.4f to desired speed %.4f (delta=%.4f) with accel=%.4f, decel=%.4f, dt=%.4f, thresh=%.4f\n", current, desired, fabs(fabs(current)-fabs(desired)), accel, decel, dt, thresh);
  )
*/

  if( desired == current || fabs( fabs(current) - fabs(desired)) <= thresh )
  { 
    if(fabs(current) < thresh)
      (*accel_state) = BMS_STOPPED; // robot is stationary
    else
      (*accel_state) = BMS_CONSTVEL; // robot is moving at constant vel

    ACCEL_DEBUG(if(desired != current) printf("acceleration from %.4f to %.4f is below threshold %.4f. (accel was %.4f, decel was %.4f, dt was %.4f)\n", current, desired, thresh, accel, decel, dt);)
    return desired;
  }
  else if(current >= 0 && desired >= 0 && desired > current)   // accelerate forward
  {
    (*accel_state) = BMS_ACCEL;
    ACCEL_DEBUG(printf("stg_model_position: acceleration: accelerating forwards from curretn %.4fm/s towards desired %.4fm/s by %0.3fm/s/s over dt %0.3fs; new velocity will be %.4fm/s.\n", current, desired, decel, dt, MIN(desired, (current + (accel * dt))));)
    return (accel == 0) ? desired : MIN(desired, (current + (accel * dt)));
    // MIN prevents returned velocity from being greater than desired (too much
    // acceleration)
  }
  else if(current <= 0 && desired < 0 && desired < current)   // accelerate backward
  {
    (*accel_state) = BMS_ACCEL;
    ACCEL_DEBUG(puts("accelerating backwards...");)
    return (accel == 0) ? desired : MAX(desired, current - (accel * dt)); 
    // MAX prevents returned velocity from being less (faster backwards) than desired (too much acceleration)
  }
  else if(current >= 0 && desired >= 0 && desired < current)   // decelerate forward
  {
    (*accel_state) = BMS_DECEL;
    double result;
    const double decel_amt = decel * dt;
    const double delta = current - desired;
    if(decel_amt > delta)
    {
      ACCEL_DEBUG(printf("stg_model_position: accelerateion: would have to decelerate by %.4f which is more than delta difference %.4f! just returning desired speed %.4f (current is %.4f).", decel_amt, delta, desired))
      return desired;
    }
    result = (decel == 0 || fabs(current - desired) <= min_speed_thresh) ? desired : MAX(desired, fabs(current - (decel * dt)));
    ACCEL_DEBUG(
      printf("stage_model_position: aceleration: decelerating forward from currrent %.4fm/s towards desired %.4fm/s (delta %.4fm/s) by %0.3fm/s/s over dt %0.3fs; calculated deceleration amount to be %.4f and decelerated speed to be %.4f; min. speed is %.4f; new velocity will be %.4fm/s.\n", current, desired, current-desired, decel, dt, (decel * dt), fabs(current - (decel * dt)), min_speed_thresh, result);
    )
    if(result > current)
    {
      stg_print_warning("position model calculated decelerated speed %f greater than current speed %f! ignoring.  (decel=%f, desired speed=%f, thresh=%f, dt=%; decel %f * dt %f = %f; current %f - %f = %f)", result, current, decel, desired, min_speed_thresh, dt, decel, dt, (decel * dt), current, (decel * dt), current - (decel * dt));
      return current;
    }
    return result;
    // MAX and fabs prevents returned velocity from being less than desired (too
    // much deceleration)
  }
  else                                         // decelerate backward
  {
    (*accel_state) = BMS_DECEL;
    ACCEL_DEBUG(
      puts("decelerating backwards...");
      printf("stage_model_position: aceleration: decelerating backward from currrent %.4fm/s towards desired %.4fm/s (delta %.4fm/s) by %0.3fm/s/s over dt %0.3fs; new velocity will be %.4fm/s.\n", current, desired, current-desired, decel, dt, current + (decel * dt));
    )
    return (decel == 0 || fabs(current - desired) <= min_speed_thresh) ? desired : MIN(desired, current + (decel * dt)); 
    // MIN prevents returned velocity from being greater than desired (too much
    // deceleration)
  }

  // If we reach here, my logic above is badly flawed...
  printf("\t?????????\n");
  printf("stage model_position: acceleration(): BUG: cannot decide what to do. current=%f desired=%f accel=%f decel=%f dt=%f thr=%f\n", 
      current, desired, accel, decel, dt, thresh);
  printf("\t?????????\n");

  return desired;
}

void battery_update( stg_model_t * mod, double dt, int x_accel_state, int y_accel_state, int a_accel_state)
{

  int* batt_op_state = NULL;
  stg_batt_soc_t* batt_capacity = NULL;
  stg_batt_soc_t* batt_charge = NULL;
  stg_batt_soc_t* cur_soc = NULL;
  stg_charge_rate_t* soc_rates;

  // Check the battery's operation state
  batt_op_state = stg_model_get_property_fixed( mod, "battery_operation_state", sizeof(int));
  if(batt_op_state == NULL)
  {
    //printf("batt_op_state not yet set\n");
    return;
  }
  //printf( "model %s batt_op_state %d\n", mod->token, (*batt_op_state));

  // If it's DISABLED, set the SoC to 100%
  if((*batt_op_state) == BOS_DISABLED)
  {
    //printf("Battery simulation disable. Setting battery to 100% SoC\n");
    cur_soc = stg_model_get_property_fixed( mod, "battery_soc", sizeof(stg_batt_soc_t));
    assert(cur_soc);
    (*cur_soc) = 100.0;
    return;
  }

  batt_capacity = stg_model_get_property_fixed( mod, "battery_capacity", sizeof(stg_batt_soc_t));
  if(batt_capacity == NULL)
  {
    //printf("batt_capacity not yet set\n");
    return;
  }
  //printf( "model %s batt_capacity %.10f\n", mod->token, (*batt_capacity));

  batt_charge = stg_model_get_property_fixed( mod, "battery_charge", sizeof(stg_batt_soc_t));
  if(batt_charge == NULL)
  {
    //printf("batt_charge not yet set\n");
    return;
  }
  //printf( "model %s batt_charge %.10f\n", mod->token, (*batt_charge));

  cur_soc = stg_model_get_property_fixed( mod, "battery_soc", sizeof(stg_batt_soc_t));
  assert(cur_soc);
  //printf( "model %s cur_soc %.10f\n", mod->token, (*cur_soc));

  soc_rates = stg_model_get_property_fixed( mod, "battery_soc_rate", sizeof(stg_charge_rate_t));
  assert(soc_rates);
  //printf( "model %s soc_rates {%.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f,}\n", mod->token,
  //      soc_rates->docked, soc_rates->robothotel, soc_rates->constvel, soc_rates->accel, soc_rates->decel, soc_rates->payloadhotel, soc_rates->payloadatgoal );


  // Start with no charge change
  stg_batt_soc_t charge_change = 0.0;

  // Deduct the constant discharge values (robotHotel/payloadHotel)
  charge_change -= (soc_rates->robothotel * dt);
  charge_change -= (soc_rates->payloadhotel * dt);

  // Charging at the dock
  if((*batt_op_state) == BOS_CHARGING)
    charge_change += (soc_rates->docked * dt);

  // Working at a Goal
  if((*batt_op_state) == BOS_WORKING)
  {
    //printf("model %s batt_op_state: BOS_WORKING");
    charge_change -= (soc_rates->payloadatgoal *dt);
  }


  // Battery drain due to robot velocity/accel/decel
  //if((*batt_op_state) == BOS_DRIVING)
  {
    // NOTE: I used to think this would require a special state BOS_DRIVING,
    //        but it seems we should be doing this drain due to movement
    //        no matter what state we're in.
    stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*) stg_model_get_property_fixed(mod, "position_speed_config", sizeof(stg_position_speed_config_t));
    assert(speed_cfg);

    // Battery simulation for driving with accel/decel
    /*if(x_accel_state == BMS_STOPPED)                    // no longer relevant to discharge
      charge_change = -(cur_soc_rate->constant * dt);   // no longer relevant to discharge
    else*/ if (x_accel_state == BMS_CONSTVEL)
      charge_change -= (soc_rates->constvel * dt);
    else if (x_accel_state == BMS_ACCEL)
    {
      double pct_of_max = speed_cfg->current_accel.x / speed_cfg->max_accel.x;
      charge_change -= (soc_rates->accel * dt * pct_of_max);
    }
    else if (x_accel_state == BMS_DECEL)
    {
      double pct_of_max = speed_cfg->current_decel.x / speed_cfg->max_decel.x;
      charge_change -= (soc_rates->decel * dt);
    }
  }

  //printf("batt_charge = %.10f, ", (*batt_charge));
  //printf("charge_change = %.10f, ", charge_change);
  (*batt_charge) += charge_change;
  if((*batt_charge) > (*batt_capacity))
    (*batt_charge) = (*batt_capacity);
  //printf("batt_charge = %.10f\n", (*batt_charge));

  //printf("cur_soc = %.10f, ", (*cur_soc));
  //printf("batt_charge/batt_capacity = %.10f/%.10f = %.10f, ", (*batt_charge), (*batt_capacity), ((*batt_charge)/(*batt_capacity)));
  (*cur_soc) = ((*batt_charge)/(*batt_capacity)) * 100.0;
  if(*cur_soc > 100.0)
    (*cur_soc) = 100.0;
  //printf("cur_soc = %.10f\n", (*cur_soc));
}


// use current command ("position_cmd" property) and other factors to
// choose velocity values for moving the model.
int position_update( stg_model_t* mod )
{
  // Model properties:
  stg_velocity_t* vel;
  stg_position_data_t* data;
  stg_position_speed_config_t* speed_cfg;
  stg_position_cmd_t* cmd;
  stg_position_drive_mode_t* drive;
  stg_pose_t* last_pose;


  // this flag is set while translational position control is active
  // on differential drive robots (since it has to turn in order to 
  // drive to the goal point), temporarily preventing rotation command
  // (vel or pos) from being used.
  char override_rotation_control = 0;

  // result of applying commands stored here, then model velocity set based
  // on accelerations and speed limits.
  stg_velocity_t desired_vel;

  // Used when comparing floating point numbers for practical equality
  const double epsilon = 0.00002;

  PRINT_DEBUG1( "[%lu] position update", mod->world->sim_time );
  
  // Get pointers to properties
  vel = 
    stg_model_get_property_fixed( mod, "velocity", 
				  sizeof(stg_velocity_t));
  assert(vel);

  if(mod->subs == 0) 
  {
    // no subscribers, no driving.
    memset(vel, 0, sizeof(stg_velocity_t));
    _model_update(mod);
    stg_model_property_changed( mod, "velocity" );
    return 0;
  }
    
  data = 
    stg_model_get_property_fixed( mod, "position_data", 
				  sizeof(stg_position_data_t));
  assert(data);

  speed_cfg = 
    stg_model_get_property_fixed(mod, "position_speed_config", 
        sizeof(stg_position_speed_config_t));
  assert(speed_cfg);

#ifdef DEBUG_SPEED_PROPERTIES
  printf("DEBUG position_update. speed config: max speed=[%g,%g,%g] max accel=[%g,%g,%g] "
      "max decel=[%g,%g,%g]; current accel=[%g,%g,%g] current decel=[%g,%g,%g]; "
      "default speed=[%g,%g,%g]\n" ,
      speed_cfg->max_speed.x, speed_cfg->max_speed.y, speed_cfg->max_speed.a, 
      speed_cfg->max_accel.x, speed_cfg->max_accel.y, speed_cfg->max_accel.a, 
      speed_cfg->max_decel.x, speed_cfg->max_decel.y, speed_cfg->max_decel.a, 
      speed_cfg->current_accel.x, speed_cfg->current_accel.y, speed_cfg->current_accel.a, 
      speed_cfg->current_decel.x, speed_cfg->current_decel.y, speed_cfg->current_decel.a, 
      speed_cfg->default_speed.x, speed_cfg->default_speed.y, speed_cfg->default_speed.a
    );
#endif
  

  memset(&desired_vel, 0, sizeof(stg_velocity_t));

  cmd = (stg_position_cmd_t*) stg_model_get_property_fixed(mod, "position_cmd", sizeof(stg_position_cmd_t));
  assert(cmd);

  drive = (stg_position_drive_mode_t*) stg_model_get_property_fixed(mod, "position_drive", sizeof(stg_position_drive_mode_t));
  assert(drive);

  last_pose = (stg_pose_t*) stg_model_get_property_fixed(mod, "position_last_pose", sizeof(stg_pose_t));

  /* Translation (X and Y): */
  /* TODO: separate X and Y (if omni) */
  switch( cmd->transmode)
  {
    /* No translation */
    case STG_POSITION_CONTROL_IDLE:
      desired_vel.x = desired_vel.y = 0;
      break;

    /* Velocity Control Mode: */
    case STG_POSITION_CONTROL_VELOCITY :
    {
      PRINT_DEBUG( "trans: velocity control mode" );
      switch( *drive )
      {
        case STG_POSITION_DRIVE_DIFFERENTIAL:
          // differential-steering model, like a Pioneer
          PRINT_DEBUG2( "model %s diff drive X-vel command %.4f", mod->token, cmd->x);
          desired_vel.x = cmd->x;
          desired_vel.y = 0;
          break;
          
        case STG_POSITION_DRIVE_OMNI:
          // direct steering model, like an omnidirectional robot
          PRINT_DEBUG3( "model %s omni drive X-vel command %.4f Y-vel %.4f", mod->token, cmd->x, cmd->y);
          desired_vel.x = cmd->x;
          desired_vel.y = cmd->y;
          break;
          
        default:
          PRINT_ERR1( "unknown steering mode %d", *drive );
      }
    } break;

    /* Relative Position Control (Delta) Mode: */
    // TODO: drive at linear speed, not proportional?
    // TODO need to seperate x and y position control modes!!
    case STG_POSITION_CONTROL_RELATIVE:
    {

      stg_pose_t* progress = stg_model_get_property_fixed(mod, "position_rel_ctrl_progress", sizeof(stg_pose_t));

      {
        char* active = (char*) stg_model_get_property_fixed(mod, "position_rel_ctrl_active_X", sizeof(char));

        if(*active == 0 ) {
            // Nothing to do.
            desired_vel.x = 0;
        }
        else
        {

          stg_print_msg("Doing relative move command %.4f m. Progress is %.4f m.", cmd->x, progress->x);
          if( fabs(progress->x) + epsilon >= fabs(cmd->x) )
          {
            stg_print_msg("Finished relative move command, went %.4f m. (commanded %.4f m)", progress->x, cmd->x);
            progress->x = 0;
            cmd->x = 0;   
            cmd->transmode = STG_POSITION_CONTROL_IDLE; // WHY DOES THIS GET CHANGED BACK LATER?
            stg_model_property_changed(mod, "position_cmd");
            stg_model_property_changed(mod, "position_rel_ctrl_progress");
            desired_vel.x = 0;
            *active = 0;
            stg_model_property_changed(mod, "position_rel_ctrl_active_X");
          }
          else
          {

            // Update progress
            progress->x += fabs( fabs(last_pose->x - data->pose.x) / cos(last_pose->a) );

            // Continue current movement
            desired_vel.x = ((cmd->x - progress->x) * speed_cfg->default_speed.x); 
          }
        }

      }

      // Do lateral (Y) movement if still active:
      {
        char* active;

        // Ignore the command in differential drive mode
        if(*drive == STG_POSITION_DRIVE_DIFFERENTIAL) {
          desired_vel.y = 0;
          break;
        } 

        active = (char*) stg_model_get_property_fixed(mod, "position_rel_ctrl_active_Y", sizeof(char));
        if(*active == 0) {
            // Nothing to do.
            desired_vel.y = 0;
            break;
        }
        

        if(fabs(progress->y) + epsilon >= fabs(cmd->y))
        {
          stg_print_msg("Finished relative lateral move command, went %.4f m. (commanded %.4f m)", progress->y, cmd->y);
          progress->y = cmd->y = 0;
          cmd->transmode = STG_POSITION_CONTROL_IDLE;
          stg_model_property_changed(mod, "position_cmd");
          stg_model_property_changed(mod, "position_rel_ctrl_progress");
          desired_vel.y = 0;
          *active = 0;
          stg_model_property_changed(mod, "position_rel_ctrl_active_Y");
          break;
        }

        // Update progress from last time
        progress->y += fabs(last_pose->y - data->pose.y) / cos(last_pose->a);

        // Continue current movement
        desired_vel.y = ((cmd->y - progress->y) * speed_cfg->default_speed.y); 
      }

      stg_model_property_changed(mod, "position_rel_ctrl_progress");

    }
    break;
      
    /* Absolute Position Control Mode: */
    case STG_POSITION_CONTROL_POSITION:
    {
      //double x_error, y_error;
      const double x_error = cmd->x - data->pose.x;
      const double y_error = cmd->y - data->pose.y;
      PRINT_DEBUG( "trans: position control mode" );
      PRINT_DEBUG2( "errors: %.4f %.4f\n", x_error, y_error );
      switch( *drive )
      {
        case STG_POSITION_DRIVE_OMNI:
        {
          // this is easy - we just reduce the errors in each axis
          // independently with a proportional controller, speed limited
          desired_vel.x = (MIN( x_error, speed_cfg->default_speed.x ) );
          desired_vel.y = (MIN( y_error, speed_cfg->default_speed.y ) );
        }
        break;
        
        case STG_POSITION_DRIVE_DIFFERENTIAL:
        {
          // axes can not be controlled independently. We have to
          // turn towards the desired x,y position, drive there,
          // then turn to face the desired angle.  this is a
          // simple controller that works ok. Could easily be
          // improved if anyone needs it better. Who really needs
          // X,Y position control anyway?
          
          // start out with no velocity
          stg_velocity_t calc;
          memset( &calc, 0, sizeof(calc));
          
          
          // if we're at the right spot do nothing, and let rotation command 
          // take effect later in this function.
          if( fabs(x_error) <= close_enough && fabs(y_error) <= close_enough )
          {
            override_rotation_control = 0;
          }
          else
          {
            // otherwise turn to face the goal point, preventing rotation command
            // from taking effect later.
            double goal_angle, goal_distance, a_error;
            override_rotation_control = 1;
            PRINT_DEBUG( "TURNING TO FACE THE GOAL POINT" );
            goal_angle = atan2( y_error, x_error );
            goal_distance = hypot( y_error, x_error );
            a_error = NORMALIZE( goal_angle - data->pose.a );
            calc.a = (MIN(a_error, speed_cfg->default_speed.a));
            PRINT_DEBUG3( "rot error: %.4f, trans dist: %.4f epsilon: %.4f\n", a_error, goal_distance, head_close_enough );
            
            // if we're pointing about the right direction, drive forward
            if( fabs(a_error) <= head_close_enough )
            {
              PRINT_DEBUG( "DRIVING TOWARDS THE GOAL" );
              calc.x = (MIN(goal_distance, speed_cfg->default_speed.x)); 
            } 
          }
            
          // now set the desired velocities. in diff mode, you can't have
          // any y velocity.
          desired_vel.x = calc.x;
          desired_vel.y = 0;
          desired_vel.a = calc.a;
        }
        break;
        
        default:
          PRINT_ERR1( "unknown steering mode %d", (int)drive );
      }
    }
    break;

    default:
      PRINT_ERR1( "unknown translation control mode %d", cmd->transmode );
  }
      


  /* Rotation (A): */

  if(!override_rotation_control)
  {
    switch( cmd->rotmode )
    {
      /* No movement */
      case STG_POSITION_CONTROL_IDLE:
	//printf("rotation control: idle position control. desired=0.\n");
        desired_vel.a = 0.0;
        break;

      /* Velocity */
      case STG_POSITION_CONTROL_VELOCITY :
      {
        PRINT_DEBUG( "rot: velocity control mode" );
        PRINT_DEBUG2( "model %s X command %.4f", mod->token, cmd->a);
        desired_vel.a = cmd->a;
	//printf("rotation control: velocity control. desired=%f.\n", desired_vel.a);
      }
      break;

      /* Relative (Delta) control */
      // TODO: drive at linear speed, not proportional?
      case STG_POSITION_CONTROL_RELATIVE:
      {

        stg_pose_t* progress;
        char* active = (char*) stg_model_get_property_fixed(mod, "position_rel_ctrl_active_A", sizeof(char));
        if(*active == 0) {
            // Nothing to do.
            desired_vel.a = 0.0;
	    //printf("rotation control: inactive relative control. desired=%f.\n", desired_vel.a);
            break;
        }
        
        progress = stg_model_get_property_fixed(mod, "position_rel_ctrl_progress", sizeof(stg_pose_t));

        // Check if command is done
        // TODO: use current simulation speed to determine whether we'll
        // overshoot in the next cycle.
        //stg_print_msg("Doing relative rotation command, went %.4f deg, commanded %.4f deg, threshold %f deg", progress->a, cmd->a, head_close_enough);
        if( fabs( fabs(progress->a) - fabs(cmd->a ) ) <= head_close_enough)
        {
          stg_print_msg("Finished relative rotation command, went %.4f deg. (commanded %.4f deg, threshold %f deg)", progress->a, cmd->a, head_close_enough);
          progress->a = cmd->a = 0;
          cmd->rotmode = STG_POSITION_CONTROL_IDLE;
          stg_model_property_changed(mod, "position_rel_ctrl_progress");
          stg_model_property_changed(mod, "position_cmd");
          desired_vel.a = 0.0;
          *active = 0;
          stg_model_property_changed(mod, "position_rel_ctrl_active_A");
          //printf("rotation control: relative control reached goal. desired=%f.\n", desired_vel.a);
          break;
        }

        // Update progress from last time
        progress->a += fabs( ( fabs(last_pose->a) + TWOPI ) - ( fabs(data->pose.a) + TWOPI ) );

        {
          // Continue current movement
          double togo = (fabs(cmd->a) + TWOPI) - (fabs(progress->a) + TWOPI);
          if(cmd->a < 0) togo *= -1;
          desired_vel.a = (MIN(togo, speed_cfg->default_speed.a));
          //printf("rotation control: relative control active. desired=%f.\n", desired_vel.a);
        }
      }
      stg_model_property_changed(mod, "position_rel_ctrl_progress");
          
      break;

      /* Absolute position (heading) */
      case STG_POSITION_CONTROL_POSITION:
      {
        const double a_error = NORMALIZE( (cmd->a + TWOPI) - (data->pose.a + TWOPI) );
        //printf("rotation control: error=%fr, pose=%f r (%fdeg), cmd=%fr/sec, closeEnough=%fr\n", a_error, data->pose.a, RTOD(data->pose.a), cmd->a, head_close_enough);
        
        /*
        printf("  pose + thresh = %f.\n", data->pose.a + head_close_enough);
        printf("  |pose| +  thresh = %f; |cmd| = %f\n", fabs(data->pose.a) + head_close_enough, fabs(cmd->a));
        if(  ( fabs(data->pose.a) + head_close_enough >= fabs(cmd->a) )
          ) 
        */
        // TODO we need to actually allow it to overshoot the goal heading
        // slightly, so that we stop at (goal + head_close_enough) ??

        if( fabs(a_error) <= head_close_enough )
        {
          //printf("ABS ROT DONE.\n");
          desired_vel.a = 0.0;
          cmd->rotmode = STG_POSITION_CONTROL_IDLE;
          stg_model_property_changed(mod, "position_cmd");
          //printf("rotation control: absolute control reached goal. desired=%f.\n", desired_vel.a);
          break;
        }

        // This uses the error for proportional control, with the default as a
        // maximum speed, but never go below head_minimum_vel (that would be
        // way too slow)
        if(a_error < 0) {
          desired_vel.a = MIN(-head_minimum_speed, 
            MAX(a_error, -(speed_cfg->default_speed.a) ) );
        }  else {
          desired_vel.a = MAX(head_minimum_speed, 
            MIN(a_error, speed_cfg->default_speed.a) );
        }


        // This just uses the default speed, which causes overshoot, so don't use it:
        //desired_vel.a = speed_cfg->default_speed.a;

        //printf("rotation control: absolute control desired=%f. \n", desired_vel.a);
      }
      break;
          
    default:
      PRINT_ERR1( "unrecognized rotational control mode %d", cmd->rotmode );
    }
  }
        
  // Limit desired velocities to be within (+/-)max_speed:
  desired_vel.x = MIN(desired_vel.x, speed_cfg->max_speed.x);
  desired_vel.x = MAX(desired_vel.x, -1 * speed_cfg->max_speed.x);
  desired_vel.y = MIN(desired_vel.y, speed_cfg->max_speed.y);
  desired_vel.y = MAX(desired_vel.y, -1 * speed_cfg->max_speed.y);
  desired_vel.a = MIN(desired_vel.a, speed_cfg->max_speed.a);
  desired_vel.a = MAX(desired_vel.a, -1 * speed_cfg->max_speed.a);
    
  //printf("position: after speed limit, desired=%f.\n\taccelerating. current=%f, accel=%f, |desired|-|current|=%f, thresh=%f\n", desired_vel.a, vel->a, speed_cfg->current_accel.a, fabs(desired_vel.a)-fabs(vel->a), rot_accel_thresh);
    
  // use acceleration and deceleration values to ramp up or down towards
  // desired velocity.
  {
    stg_msec_t* last_update = stg_model_get_property_fixed(mod, "position_last_update_time", sizeof(stg_msec_t));
    const stg_msec_t now = stg_timenow();
    const double dt = (now - *last_update)/1000.0; 

    stg_velocity_t decel;
    if(cmd->override_decel.x > epsilon)
      decel.x = cmd->override_decel.x;
    else
      decel.x = speed_cfg->current_decel.x;
    if(cmd->override_decel.y > epsilon)
      decel.y = cmd->override_decel.y;
    else
      decel.y = speed_cfg->current_decel.y;
    if(cmd->override_decel.a > epsilon)
      decel.a = cmd->override_decel.a;
    else
      decel.a = speed_cfg->current_decel.a;

    // dt is seconds since last update. assume that the next update will be
    // in a similar interval, so use that to calculate acceleration or
    // deceleration:
    if(dt > epsilon) {
      int x_accel_state = -1;
      int y_accel_state = -1;
      int a_accel_state = -1;

      //ACCEL_DEBUG(printf("X:");)
      vel->x = accelerate(desired_vel.x, vel->x, speed_cfg->current_accel.x, decel.x, dt, accel_thresh, &x_accel_state, 1);
      //ACCEL_DEBUG(printf("Y:");)
      vel->y = accelerate(desired_vel.y, vel->y, speed_cfg->current_accel.y, decel.y, dt, accel_thresh, &y_accel_state, 0);
      //ACCEL_DEBUG(printf("A:");)
      vel->a = accelerate(desired_vel.a, vel->a, speed_cfg->current_accel.a, decel.a, dt, rot_accel_thresh, &a_accel_state, 0);
      stg_model_set_property(mod, "position_last_update_time", &now, sizeof(now));   // remember current time for next time
      //ACCEL_DEBUG(puts("---");)

      battery_update(mod, dt, x_accel_state, y_accel_state, a_accel_state);
    } else {
      PRINT_WARN("position model: supposedly it's been 0.00002 msec or less since last update! skipping acceleration!");
      vel->x = desired_vel.x;
      vel->y = desired_vel.y;
      vel->a = desired_vel.a;
    }
  }

//printf("stage model_position: update(): after acceleration, vel.x=%f, vel.a=%f.\n", vel->x, vel->a);

  // copy current pose to remember for next time:
  stg_model_set_property(mod, "position_last_pose", &(data->pose), sizeof(stg_pose_t));

  // simple model of power consumption
  // mod->watts = STG_POSITION_WATTS + 
  //fabs(vel->x) * STG_POSITION_WATTS_KGMS * mod->mass + 
  //fabs(vel->y) * STG_POSITION_WATTS_KGMS * mod->mass + 
  //fabs(vel->a) * STG_POSITION_WATTS_KGMS * mod->mass; 

  // we've poked the velocity - must refresh it so others notice
  // the change
  stg_model_property_changed( mod, "velocity" );

  // now  inherit the normal update - this does the actual moving
  _model_update( mod );

  // Get new pointers to data, old pointers may have become invalid during
  // _model_update() or stg_model_property_changed(mod, "velocity").
  vel = stg_model_get_property_fixed(mod, "velocity", sizeof(stg_velocity_t));

  // Set stall based on whether _model_update detected a collision
  {
    stg_position_stall_t *stall = stg_model_get_property_fixed(mod, "position_stall", sizeof(stg_position_stall_t));
    (*stall) = mod->collision;
    stg_model_property_changed(mod, "position_stall");
  }

  // Change our reported pose in the position_data property according to true pose moved
  // by _model_update, and our localization type:
  switch( data->localization )
  {
    case STG_POSITION_LOCALIZATION_GPS:
    {
      double dx, dy, cosa, sina;

      // compute our localization pose based on the origin and true pose
      stg_pose_t gpose;
      stg_model_get_global_pose( mod, &gpose );

      data->pose.a = NORMALIZE( gpose.a - data->origin.a );
      //data->pose.a =0;// NORMALIZE( gpose.a - data->origin.a );
    
      STG_SINCOS(data->origin.a, sina, cosa);
      dx = gpose.x - data->origin.x;
      dy = gpose.y - data->origin.y; 
      data->pose.x = dx * cosa + dy * sina; 
      data->pose.y = dy * cosa - dx * sina;
    }
    break;
    
    case STG_POSITION_LOCALIZATION_ODOM:
    {
      // integrate our velocities to get an 'odometry' position estimate.
      double cosa, sina, vx, vy;

      const double dt = mod->world->sim_interval/1e3;   // sec

      if(force_odom_error_mode_all_models  == STG_POSITION_ODOM_ERROR_RANDOM_EACH_UPDATE) // todo also check a flag in this model's property
        stg_position_choose_new_random_odom_error(data);

      STG_SINCOS((DTOR(90.0) - data->pose.a), sina, cosa);

      vx = (vel->x * dt) * (1.0 + data->integration_error.x );
      vy = (vel->y * dt) * (1.0 + data->integration_error.y );

/*
      vmagx = sqrt(vx * vx + vy * vy);
      vmagy = vmagx;
      if(vx < 0) vmagx = -vmagx;
      if(vy < 0) vmagy = -vmagy;

      if(vmagx != 0 || vmagy != 0 )
        printf("vx=%f, vy=%f, vmagx=%f, vmagy=%f cosa=%f, sina=%f, moving by dx=%f, dy=%f.\n", vx, vy, vmagx, vmagy, cosa, sina, vmagx*cosa, vmagy*sina);

      data->pose.x += vmagx * cosa;
      data->pose.y += vmagy * sina;
      */

      data->pose.a = NORMALIZE( data->pose.a + (vel->a * dt) * (1.0 +data->integration_error.a) );

      data->pose.x -= vy * cosa - vx * sina;
      data->pose.y += vx * cosa + vy * sina; 
    }
    break;
    
    default:
      PRINT_ERR2( "unknown localization mode %d for model %s\n",
      data->localization, mod->token );
      break;
  }

  // we've probably poked the position data - must refresh 
  stg_model_property_changed( mod, "position_data" );


  //stg_print_msg( "%s: position origin %.2f %.2f %.2f", mod->token, data->pose.x, data->pose.y, data->pose.a );
//printf("\n");

  return 0; //ok
}

int position_startup( stg_model_t* mod )
{
  PRINT_DEBUG( "position startup" );

  //mod->watts = STG_POSITION_WATTS;

  
  //stg_model_position_odom_reset( mod );

  return 0; // ok
}

int position_shutdown( stg_model_t* mod )
{
  stg_position_cmd_t cmd;
  stg_velocity_t vel;

  PRINT_DEBUG( "position shutdown" );
  
  // safety features!
  memset( &cmd, 0, sizeof(cmd) ); 
  stg_model_set_property( mod, "position_cmd", &cmd, sizeof(cmd));
   
  memset( &vel, 0, sizeof(vel));
  stg_model_set_property( mod, "velocity", &vel, sizeof(vel) );
  
  return 0; // ok
}

int position_unrender_data( stg_model_t* mod, char* name, 
			    void* data, size_t len, void* userp )
{
  stg_model_fig_clear( mod, "position_data_fig" );
  return 1;
}

int position_render_data( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp )
{
  stg_rtk_fig_t* fig = stg_model_get_fig( mod, "position_data_fig" );
  
  if( !fig )
    {
      stg_color_t* col;

      fig = stg_model_fig_create( mod, "position_data_fig", NULL, STG_LAYER_POSITIONDATA );
      //stg_rtk_fig_color_rgb32( fig, 0x9999FF ); // pale blue
      
      col = stg_model_get_property_fixed( mod, "color", sizeof(stg_color_t));
      assert(col);

      stg_rtk_fig_color_rgb32( fig, *col ); 
    }

  stg_rtk_fig_clear(fig);
	  
  if( mod->subs )
    {  
      char buf[512];
      gboolean useDeg;
      const char* rotUnits;
      const char* transmode;
      const char* rotmode;

      stg_position_data_t* odom = (stg_position_data_t*)data;
      
      stg_velocity_t* vel = stg_model_get_property_fixed( mod, "velocity", sizeof(stg_velocity_t));

      stg_pose_t* truepose = 
          (stg_pose_t*) stg_model_get_property_fixed(mod, "pose", sizeof(stg_pose_t));
      
      stg_position_speed_config_t *speed_cfg = 
        (stg_position_speed_config_t*) stg_model_get_property_fixed(mod, "position_speed_config", sizeof(stg_position_speed_config_t));

      stg_position_stall_t *stall = (stg_position_stall_t*) stg_model_get_property_fixed(mod, "position_stall", sizeof(stg_bool_t));

      stg_position_cmd_t *cmd = (stg_position_cmd_t*) stg_model_get_property_fixed(mod, "position_cmd", sizeof(stg_position_cmd_t));

      //stg_rtk_fig_origin( fig,  odom->pose.x, odom->pose.y, odom->.a );
      stg_rtk_fig_origin( fig,  odom->origin.x, odom->origin.y, odom->origin.a );
      
      stg_rtk_fig_rectangle(  fig, 0,0,0, 0.06, 0.06, 0 );
      stg_rtk_fig_line( fig, 0,0, odom->pose.x, 0);
      stg_rtk_fig_line( fig, odom->pose.x, 0, odom->pose.x, odom->pose.y );

      // draw text next to the robot's actual pose in stage (next to its icon) 
      useDeg = mod->world->display_degrees;
      if(useDeg)
        rotUnits = "deg";
      else
        rotUnits = "rad";
      switch(cmd->transmode)
      {
        case STG_POSITION_CONTROL_POSITION:
          transmode = "ABSMOVE ";
          break;
        case STG_POSITION_CONTROL_RELATIVE:
          transmode = "MOVE ";
          break;
        default:
          transmode = "";
      }
      switch(cmd->rotmode)
      {
        case STG_POSITION_CONTROL_POSITION:
          rotmode = "HEAD ";
          break;
        case STG_POSITION_CONTROL_RELATIVE:
          rotmode = "DHEAD ";
          break;
        default:
          rotmode = "";
      }

      snprintf( buf, 512, 
        "%s: %s\n"
        "true pose: (% 6.3f m, % 6.3f m, % 3.2f deg)\n" 
        "velocity:  (% 6.3f m/s%s, % 6.3f m/s%s, % 3.2f deg/s%s)\n"
        "vel cmd:   (%s% 6.3f %s, % 6.3f %s,%s% 6.3f %s)\n" 
        "accel:     (% 6.3f m/s/s%s, % 6.3f m/s/s%s, % 3.2f deg/s/s%s)\n"
        "decel:     (% 6.3f m/s/s%s, % 6.3f m/s/s%s, % 3.2f deg/s/s%s)",
        mod->token,
        stall && *stall ? "[STALL]" : "",
        truepose->x, truepose->y, RTOD(NORMALIZE(truepose->a)),
		    vel->x, (speed_cfg->max_speed.x > 0 && (vel->x >= speed_cfg->max_speed.x || vel->x <= -speed_cfg->max_speed.x) ? " (MAX)":""),
          vel->y, (speed_cfg->max_speed.y > 0 && (vel->y >= speed_cfg->max_speed.y || vel->y <= -speed_cfg->max_speed.y) ? " (MAX)":""),
          RTOD(NORMALIZE(vel->a)), (speed_cfg->max_speed.a > 0 && (vel->a >= speed_cfg->max_speed.a || vel->a <= -speed_cfg->max_speed.a) ? " (MAX)":""),
        transmode, cmd->x, (cmd->transmode==STG_POSITION_CONTROL_VELOCITY)?"m/s":"m",
          cmd->y, (cmd->transmode==STG_POSITION_CONTROL_VELOCITY)?"m/s":"m", 
          rotmode, RTOD(NORMALIZE(cmd->a)), (cmd->rotmode==STG_POSITION_CONTROL_VELOCITY)?"deg/s":"deg",
        speed_cfg->current_accel.x, ((speed_cfg->max_accel.x > 0 && speed_cfg->current_accel.x >= speed_cfg->max_accel.x) ? " (MAX)":""),
          speed_cfg->current_accel.y, ((speed_cfg->max_accel.y > 0 && speed_cfg->current_accel.y >= speed_cfg->max_accel.y) ? " (MAX)":""),
          RTOD(NORMALIZE(speed_cfg->current_accel.a)), ((speed_cfg->max_accel.a > 0 && speed_cfg->current_accel.a >= speed_cfg->max_accel.a) ? " (MAX)":""),
        speed_cfg->current_decel.x, ((speed_cfg->max_decel.x > 0 && speed_cfg->current_decel.x >= speed_cfg->max_decel.x) ? " (MAX)":""),
          speed_cfg->current_decel.y, ((speed_cfg->max_decel.y > 0 && speed_cfg->current_decel.y >= speed_cfg->max_decel.y) ? " (MAX)":""),
          RTOD(NORMALIZE(speed_cfg->current_decel.a)), ((speed_cfg->max_decel.a > 0 && speed_cfg->current_decel.a >= speed_cfg->max_decel.a) ? " (MAX)":"")
      );

      stg_rtk_fig_text(fig, truepose->x + 0.8, truepose->y + 0.4, 0, buf );

      // draw text next to odometric pose. TODO combine with real data above if
      // they overlap.
      snprintf(buf, 255,
        "%s:\nodom pose: (% 6.3f m, % 6.3f m, % 3.2f deg)\napplying error:   (% 6.5f m, % 6.5f m, % 3.5f deg)" ,
        mod->token,
		    odom->pose.x, odom->pose.y, RTOD(NORMALIZE(odom->pose.a)),
        odom->integration_error.x, odom->integration_error.y, RTOD(odom->integration_error.a)
      );
      stg_rtk_fig_text(fig, odom->pose.x + 0.8, odom->pose.y + 0.4, 0, buf);


      // draw an outline of the position model
      {
        stg_geom_t *geom = stg_model_get_property_fixed( mod, "geom", sizeof(stg_geom_t));
        assert(geom);

        stg_rtk_fig_rectangle(  fig, 
              odom->pose.x, odom->pose.y, odom->pose.a,
              0.1, 0.1, 0 );

        stg_rtk_fig_arrow( fig, 
         odom->pose.x, odom->pose.y, odom->pose.a, 
         geom->size.x/2.0, geom->size.y/2.0 );

        //stg_pose_t gpose;
        //stg_model_get_global_pose( mod, &gpose );
        //stg_rtk_fig_line( fig, gpose.x, gpose.y, odom->pose.x, odom->pose.y );
        
      }
    }

  return 0;
}

const char *stg_position_control_mode_name(stg_position_control_mode_t m)
{
 switch(m) {
    case STG_POSITION_CONTROL_VELOCITY:
      return "velocity control";
    case STG_POSITION_CONTROL_POSITION:
      return "position control";
    case STG_POSITION_CONTROL_RELATIVE:
      return "relative position control";
    case STG_POSITION_CONTROL_IDLE:
      return "idle";
  }
  return "UNKNOWN POSITION CONTROL MODE!";
}


const char *stg_position_odom_error_mode_name(stg_position_odom_error_mode_t mode)
{
  switch(mode) {
    case STG_POSITION_ODOM_ERROR_RANDOM_INIT:
      return ("random_init");
    case STG_POSITION_ODOM_ERROR_RANDOM_EACH_UPDATE:
      return "random_each_update";
    case STG_POSITION_ODOM_ERROR_CONSTANT:
      return "constant";
    case STG_POSITION_ODOM_ERROR_NONE:
      return "none";
    case STG_POSITION_ODOM_ERROR_INVALID:
      return "INVALID";
  }
  return "UNKNOWN ODOM ERROR MODE!";
}

