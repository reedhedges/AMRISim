#ifndef STG_H
#define STG_H
/*
 *  Stage : a multi-robot simulator.  
 * 
 *  Copyright (C) 2001-2004 Richard Vaughan, Andrew Howard and Brian
 *  Gerkey for the Player/Stage Project
 *  http://playerstage.sourceforge.net
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* File: stage.h
 * Desc: External header file for the Stage library
 * Authors: Richard Vaughan vaughan@sfu.ca 
 *          Andrew Howard ahowards@usc.edu
 *          Brian Gerkey gerkey@stanford.edu
 *          MobileRobots, Inc (Reed Hedges, reed@mobilerobots.com)
 */


/*! \file stage.h 
  Stage library header file

  This header file contains the external interface for the Stage
  library
*/


// to allow definitions of extra functions we might use (but checked for in configure):
#ifndef __USE_GNU
#define __USE_GNU 1 
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <glib.h> // we use GLib's data structures extensively

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h> // for portable int types eg. uint32_t
#include <sys/types.h>
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#ifdef HAVE_PTHREAD_H // have pthread.h and thread locking enabled
#include <pthread.h>
#endif
#ifdef HAVE_SEMAPHORE_H // have pthread.h and thread locking enabled
#include <semaphore.h>
#endif

#include <rtk.h> // and graphics stuff pulled from Andrew Howard's RTK2 library

#include "config.h"
#include "replace.h"

/** @defgroup libstage libstage - Stage library API
 *
 *  @brief The Stage library platform for creating simulator applications

    libstage (The Stage Library) provides a C code library for
    simulating a population of mobile robots and sensors. It is
    usually used by the plugin driver for <a
    href="http://playerstage.sf.net/player/player.html">Player</a>,
    but it can also be used directly to build custom simulators.

    libstage is modular and fairly simple to use. The following code is
    enough to get a complete robot simulation running:

@verbatim
#include "stage.h"

int main( int argc, char* argv[] )
{ 
  stg_init( argc, argv );

  stg_world_t* world = stg_world_create_from_file( argv[1] );
  if(!world)
  {
    // Error loading world...
    return 1;
  }
  
  While( (stg_world_update( world,TRUE )==0) )
    {}
  
  stg_world_destroy( world );

  stg_uninit();
  
  return 0;
}
@endverbatim

The two key objects in libstage are the @ref stg_world and @ref stg_model.
The world represents the running simulation environment as well as control over the simulation. 
The world contains models, which represent movable robots, sensors, obstacles, and other objects
that operate and interract in the simulation. Models may be nested or attached to each other.  
Several model types are built in, and they may
also be specialized by setting properties and nested child models in type definitions (a/k/a macros)
in a world file.


@par Contact and support

For help with libstage, please use the mailing list playerstage_users@lists.sourceforge.net. 

    @{
*/


  

  /** About Stage text. If you want to modify what the About box says to indicate more about 
      your application that is using libstage, you can augment or 
      replace these strings. Please don't completely replace the copyright string
      completely, as it is required that your application display 
      its information in some way to the user.
  */
  // @{
  extern const char *stg_about_info_appname;
  extern const char *stg_about_info_stageversion;
  extern const char *stg_about_info_appversion;  ///< If NULL, stageversion is used instead.
  extern const char *stg_about_info_description;
  extern const char *stg_about_info_url;
  extern const char *stg_about_info_copyright;
  extern const char *stg_help_link;
   // @}



  // Basic self-describing measurement types. All packets with real
  // measurements are specified in these terms so changing types here
  // should work throughout the code If you change these, be sure to
  // change the byte-ordering macros below accordingly.

  /** uniquely identify a model */
  typedef int stg_id_t;

  #define STG_ID_T_MAX INT_MAX

  /** unit of distance */
  typedef double stg_meters_t;

  /** unit of angle */
  typedef double stg_radians_t;
  
  /** unit of (short) time */
  typedef unsigned long stg_msec_t;

  /** unit of mass */
  typedef double stg_kg_t; // Kilograms (mass)

  /** unit of energy */
  typedef double stg_joules_t;

  /** unit of power */
  typedef double stg_watts_t; 

  /** unit of battery charge */
  typedef double stg_batt_soc_t;  // a value from 0.0 to 100.0 (represents % charge)
  typedef struct
  {
    stg_batt_soc_t docked, robothotel, constvel, accel, decel, payloadhotel, payloadatgoal;
  } stg_charge_rate_t;
  typedef enum { BOS_DISABLED = 0, BOS_CHARGING, BOS_IDLE, BOS_DRIVING, BOS_WORKING } stg_batt_op_state_t; // Battery simulation operation states


  /** boolean */
  typedef int stg_bool_t;

  //typedef double stg_friction_t;
  
  /** 24-bit RGB color packed 0x00RRGGBB */
  typedef uint32_t stg_color_t;

  /** obstacle value. 0 means the model does not behave, or is sensed,
      as an obstacle */
  typedef int stg_obstacle_return_t;

  /** blobfinder return value. 0 means not detected by the
      blobfinder */
  typedef int stg_blob_return_t;

  /** fiducial return value. 0 means not detected as a fiducial */
  typedef int stg_fiducial_return_t;

  typedef int stg_ranger_return_t;
  
  typedef enum { STG_GRIP_NO = 0, STG_GRIP_YES } stg_gripper_return_t;
  
  /** specify a rectangular size 
   */
  typedef struct 
  {
    stg_meters_t x, y;
  } stg_size_t;
  
  /** \struct stg_pose_t
      Specify a 3 axis position, in x, y and heading.
   */
  typedef struct
  {
    stg_meters_t x, y, a;
  } stg_pose_t;
  
  /** specify a 3 axis velocity in x, y and heading.
   */
  typedef stg_pose_t stg_velocity_t;  

  /** specify an object's basic (invisible) geometry: position and rectangular
      size
   */
  typedef struct
  {
    stg_pose_t pose;
    stg_size_t size;
  } stg_geom_t;
  
  /** bound a range of values, from min to max */
  typedef struct
  {
    double min; //< smallest value in range
    double max; //< largest value in range
  } stg_bounds_t;
  
  /** define a field-of-view: an angle and range bounds */
  typedef struct
  {
    stg_bounds_t range; //< min and max range of sensor
    stg_radians_t angle; //< width of viewing angle of sensor
  } stg_fov_t;
  

  /** Whether the GUI window should show the messages pane at bottom or not. */
  extern int stg_show_messages_view;

  /** Whether the GUI window should contain a menu bar or not */
  extern int stg_show_menubar;

  // PRETTY PRINTING -------------------------------------------------
  
  /** @ingroup libstage
   *  @defgroup print Pretty-printing: console output functions.
   *  @brief Functions to format and print messages to stdout or stderr.
      @{ 
  */


  /// Print error with timestamp and color. Output goes to stderr. Arguments are just like printf. @a m is copied to the stg_last_error_message buffer.
  void stg_print_error(const char* m, ...);

  /// Print error with timestamp and color. Output goes to stderr. @a m is copied to the stg_last_error_message buffer.
  void stg_print_error_s(const char *m);

  /// Stores the last message printed with stg_print_error or stg_print_error_s.
  extern char stg_last_error_message[];

  /// Print warning with timestamp and color. Output goes to stderr. Arguments are just like printf.
  void stg_print_warning(const char* m, ...);

  /// Print an informative message with timestamp. Output goes to stdout. Arguments are like printf.
  void stg_print_msg(const char* m, ...);

  void stg_print_message(const char *m, ...);

  void stg_print_msg_s(const char *s);

  /// @see stg_print_warning
  void stg_print_warning_s(const char* m);

  /// @see stg_print_error
  void stg_print_error_v(const char* m, va_list args);

  /// @see stg_print_warning
  void stg_print_warning_v(const char* m, va_list args);

  /// @see stg_print_msg
  void stg_print_msg_v(const char* m, va_list args);



  #define stg_print_message_v(m, args) stg_print_msg_v(m, args)

  /// Output format type identifiers
  typedef enum {STG_PRINT_COLOR_TEXT, STG_PRINT_PLAIN_TEXT, STG_PRINT_HTML} stg_print_format_t;

  /// Set the output format used by the stg_print_* functions.
  void stg_set_print_format(stg_print_format_t fmt);

  /** Old function to report an error and request that stage exit.  */
  void stg_err( const char* err );
  
  /** Print human-readable description of a geometry struct on stdout */
  void stg_print_geom( stg_geom_t* geom );

  /** Maximum length (characters) of a log message */
  #define STG_LOG_MSG_MAX 512

// end group printmsg
/**@}*/

  /** @} */

  // ENERGY --------------------------------------------------------------

  /** energy data packet */
  typedef struct
  {
    /** estimate of current energy stored */
    stg_joules_t stored;

    /** maximum storage capacity */
    stg_joules_t capacity;

    /** total joules received */
    stg_joules_t input_joules;

    /** total joules supplied */
    stg_joules_t output_joules;

    /** estimate of current energy output */
    stg_watts_t input_watts;

    /** estimate of current energy input */
    stg_watts_t output_watts;

    /** TRUE iff the device is receiving energy from a charger */
    stg_bool_t charging;

    /** the range to the charger, if attached, in meters */
    stg_meters_t range;
  } stg_energy_data_t;

  /** energy config packet (use this to set or get energy configuration)*/
  typedef struct
  {
    /** maximum storage capacity */
    stg_joules_t capacity;

    /** when charging another device, supply this many joules/sec at most*/
    stg_watts_t give;

    /** when charging from another device, receive this many
	joules/sec at most*/
    stg_watts_t take;

    /** length of the charging probe */
    stg_meters_t probe_range;
  } stg_energy_config_t;

  // there is currently no energy command packet

  // BLINKENLIGHT -------------------------------------------------------

  //typedef struct
  //{
  //int enable;
  //stg_msec_t period;
  //} stg_blinkenlight_t;

  // GRIPPER ------------------------------------------------------------

  // Possible Gripper return values
  //typedef enum 
  //{
  //  GripperDisabled = 0,
  //  GripperEnabled
  //} stg_gripper_return_t;

  // GUIFEATURES -------------------------------------------------------
  
  // Movement masks for figures
#define STG_MOVE_TRANS (1 << 0)
#define STG_MOVE_ROT   (1 << 1)
#define STG_MOVE_SCALE (1 << 2)
  
  typedef int stg_movemask_t;
  
/*   typedef struct */
/*   { */
/*     uint8_t show_data; */
/*     uint8_t show_cfg; */
/*     uint8_t show_cmd; */
    
/*     uint8_t nose; */
/*     uint8_t grid; */
/*     //uint8_t boundary; */
/*     uint8_t outline; */
/*     stg_movemask_t movemask; */
/*   } stg_guifeatures_t; */


  // LASER ------------------------------------------------------------

  /// laser return value
  /*
  typedef enum 
    {
      LaserTransparent, ///<not detected by laser model 
      LaserVisible, ///< detected by laser with a reflected intensity of 0 
      LaserBright  ////< detected by laser with a reflected intensity of 1 
    } stg_laser_return_t;
  */
  typedef int stg_laser_return_t;


  /** returns the real (wall-clock) time in milliseconds since the
      simulation started. */
  stg_msec_t stg_timenow( void );
  
  /** initialize the stage library. Optionally pass in the arguments
      from main(), so Stage can read cmdline options. Stage then
      passes the arguments to GTK+ and Xlib so they can read their own
      options.
  */
  int stg_init( int argc, char** argv );

  /** Free any memory allocated to global variables by stage. */
  void stg_uninit();
  
  
  /// if stage wants to quit, this will return non-zero
  int stg_quit_test( void );

  /** set stage's quit flag. Stage will quit cleanly very soon after
      this function is called. */
  void stg_quit_request( void );

  /**  set stage's quit flag to the given integer. If code is not 0, then
   * Stage will shutdown cleanly very soon after this function is called.
   * stg_world_update will return this code.
   */
  void stg_quit_request_code( int code );


  // UTILITY STUFF ----------------------------------------------------

  /** @ingroup libstage
   *  @defgroup util Utilities
      @brief Miscellaneous functions that don't fit anywhere else.
      @{
  */
  
  /** Get a string identifying the version of stage. The string is
      generated by autoconf 
  */
  const char* stg_version_string( void );

  /** Look up the color in the X11 database.  (i.e. transform color
      name to color value).  If the color is not found in the
      database, a set of preset defaults will be searched, or a bright 
      red color (0xFF0000) will be returned instead.
      Note, this function opens the file and searches it, so is potentially
      time consuming.
  */
  stg_color_t stg_lookup_color(const char *name);

  /** calculate the sum of @a p1 and @a p2, in @a p1's coordinate system, and
      copy the result into result. */
  void stg_pose_sum( stg_pose_t* result, stg_pose_t* p1, stg_pose_t* p2 );

  /** Calculate the angle at which a line at angle b intersects a line with
   * angle a. (e.g. a sensor ray at angle b hits an obstacle at angle a).
   * Value is normalized to (-PI/2, PI/2) (so 0 is a perfect 90 degree hit,
   * < 0 is on the "left side", > 0 is on the "right side" of line a)
   */
  stg_radians_t stg_intersection_angle(stg_radians_t a, stg_radians_t b);
    

  // POINTS ---------------------------------------------------------

  /** @ingroup libstage
   *  @defgroup stg_point Points
      @brief Creating and manipulating points
      @{
  */

  /** define a point on the plane */
  typedef struct _stg_point
  {
    stg_meters_t x, y;
  } stg_point_t;

  /** Create an array of @a count points. Caller must free the returned
      pointer, preferably with stg_points_destroy(). */
  stg_point_t* stg_points_create( size_t count );

  /// frees a point array
  void stg_points_destroy( stg_point_t* pts );

  /**@}*/


  // POLYGONS ---------------------------------------------------------

  /** @ingroup libstage
   *  @defgroup stg_polygon Polygons
      @brief Creating and manipulating polygons
      @{
  */
  
  /** define a polygon: a set of connected vertices drawn with a
      color. Can be drawn filled or unfilled. */
  typedef struct
  {
    /// pointer to an array of stg_point_t objects
    GArray* points;
    
    /// if TRUE, this polygon is drawn filled
    stg_bool_t filled; 
    
    /// render color of this polygon - TODO  - implement color rendering
    stg_color_t color;
  } stg_polygon_t; 

  
  /// return an array of @a count polygons. Allocates. Caller must free() the space.
  stg_polygon_t* stg_polygons_create( int count );
  
  /// destroy an array of @a count polygons
  void stg_polygons_destroy( stg_polygon_t* p, size_t count );
  
  /** creates a unit square polygon. Allocates memory.  */
  stg_polygon_t* stg_unit_polygon_create( void );
    
  /// Copies @a count points from @a pts into polygon @a poly, allocating
  /// memory if mecessary. Any previous points in @a poly are
  /// overwritten.
  void stg_polygon_set_points( stg_polygon_t* poly, stg_point_t* pts, size_t count );			       

  /// Appends @a count points from @a pts into polygon @a poly,
  /// allocating memory if mecessary.
  void stg_polygon_append_points( stg_polygon_t* poly, stg_point_t* pts, size_t count );			       
  
  /// scale the array of @a num polygons so that all its points fit
  /// exactly in a rectagle of @a width by @a height units
  void stg_polygons_normalize( stg_polygon_t* polys, int num, 
			       double width, double height );
  void stg_points_normalize( stg_point_t* polys, int num, 
			       double width, double height );
  
  /// print a human-readable description of a polygon on stdout
  void stg_polygon_print( stg_polygon_t* poly );
  
  /// print a human-readable description of an array of polygons on stdout
  void stg_polygons_print( stg_polygon_t* polys, unsigned int count );
  
  /** interpret a bitmap file as a set of polygons. Returns an array
      of polygons. @a poly_count is set to the number of polygons
      found.
   */
  stg_polygon_t* stg_polygons_from_image_file(  const char* filename, 
						size_t* poly_count );
       
  /**@}*/

  // end util documentation group
  /**@}*/


  // end property typedefs -------------------------------------------------


  // forward declare struct types from stage.h
  struct _stg_model;
  struct _stg_matrix;
  struct _gui_window;
  struct _stg_world;

  /** \struct opaque data structure implementing a world
   */
  typedef struct _stg_world stg_world_t;


  /** \struct opaque data structure implementing a model. You get and set all
      the interesting properties of a model using the
      stg_model_set_property(), stg_model_get_property() and
      stg_model_get_property_fixed() functions.
   */
  typedef struct _stg_model stg_model_t; // defined in stage_internal.h




  //  MODEL --------------------------------------------------------
    
  // group the docs of all the model types
  /** @ingroup libstage
   *  @defgroup stg_model Models
   *  @brief Models represent objects participating in the simulation.
      @{ */
  
  /** the maximum length of a model's name, in characters (bytes). */
#define STG_PROPNAME_MAX 128




  
  /** Define a callback function type that can be attached to a
      model's property and called whenever the property is set with
      stg_model_set_property() or stg_model_property_refresh().
  */
  typedef int (*stg_property_callback_t)(stg_model_t* mod, char* propname, void* data, size_t len, void* userdata );
  
  
  /** function type for an initialization function that configures a
      specialized model. Each special model type (laser, position,
      etc) has a single initializer function that is called when the
      model type is specified in the worldfile. The mapping is done in
      a table in typetable.cc.
  */
  typedef int(*stg_model_initializer_t)(stg_model_t*);
  

   /** container for a callback function and a single argument, so
       they can be stored together in a list with a single pointer. */
  typedef struct
  {
    stg_property_callback_t callback;
    void* arg;
  } stg_cbarg_t;

  /** Create a new model, but do not add it to the world. 
   *  This is mostly used internally when loading models from a world file, 
   *  since to create the model properly a lot of additional context information is
   *  needed.
   *  stg_world_new_model() is more useful to programs dynamically adding models
   *  at run time.
   *  The token must be unique to the world. Use stg_world_lookup() to verify.
   *
   *  @internal
   *  @sa stg_world_new_model()
   */
  stg_model_t* stg_model_create(  stg_world_t* world,
				  stg_model_t* parent, 
				  stg_id_t entity_id,
				  const char* token,
          const char* base_type_name,
          const char* instance_type_name,
          size_t index,
				  stg_model_initializer_t initializer,
          stg_bool_t bg_fig
       );

  /** destroy a model, freeing all memory. 
   * The model's destroy callback function is called, then all
   * properties are destroyed. Then the model data structure itself is
   * destroyed. */
  void stg_model_destroy( stg_model_t* mod );
  void stg_model_destroy_tree(stg_model_t *mod);
  void stg_model_destroy_ex(stg_model_t *mod, stg_bool_t destroy_children);

  /** get the pose of a model in the global CS */
  void stg_model_get_global_pose( stg_model_t* mod, stg_pose_t* pose );

  /** get the velocity of a model in the global CS */
  //void stg_model_get_global_velocity( stg_model_t* mod, stg_velocity_t* gvel );

  /* set the velocity of a model in the global coordinate system */
  //void stg_model_set_global_velocity( stg_model_t* mod, stg_velocity_t* gvel );

  /** subscribe to a model's data */
  void stg_model_subscribe( stg_model_t* mod );

  /** unsubscribe from a model's data */
  void stg_model_unsubscribe( stg_model_t* mod );

  /** configure a model by reading from the current world file */
  void stg_model_load( stg_model_t* mod );

  /** save the state of the model to the current world file */
  void stg_model_save( stg_model_t* mod );

  /** Set model's pose (in parent coordinate system) */
  void stg_model_set_pose(stg_model_t* mod, stg_pose_t pose);
  void stg_model_set_pose_position(stg_model_t* mod, stg_meters_t x, stg_meters_t y);
  void stg_model_set_pose_angle(stg_model_t* mod, stg_radians_t a);

  /** Get model's pose (in parent coordinate system). Model must have the "pose"
   * property. */
  stg_pose_t stg_model_get_pose(stg_model_t* mod);

  /** Get model's initial pose. */
  stg_pose_t stg_model_get_initial_pose(stg_model_t *mod);
 
  /** Reset model's pose to it's initial (original) pose. */
  void stg_model_reset_pose(stg_model_t *mod);
   
  /** Set model's origin (local coordinate system offset) */
  void stg_model_set_origin(stg_model_t* mod, stg_pose_t origin);

  /** Get model's origin (local coordinate system offset). Model must have the
   * "geom" property. */
  stg_pose_t stg_model_get_origin(stg_model_t* mod);

  /** Set model's size */
  void stg_model_set_size(stg_model_t* mod, stg_size_t size);

  /** Get model's size. Model must have the "geom" property. */
  stg_size_t stg_model_get_size(stg_model_t* mod);

  /** Set whether or not model geometry should be scaled according to its size,
   *  or whether geometry is at the correct scale already.
   */
  void stg_model_set_scaling(stg_model_t *mod, stg_bool_t s);

  /** Initialize the model's shape polygons. (a model may have either polys or points or both) */
  void stg_model_init_polygons(stg_model_t* mod, stg_polygon_t* polys, size_t npolys);

  /** Initialize the model's shape points (a model may have polys or points or both) */
  void stg_model_init_points(stg_model_t* mod, stg_point_t* points, size_t npoints);

  /** get a type identifier (base name 
   * e.g. after "define foo position() foo()" foo
   * has base type name "position", not "foo")
   */
  char* stg_model_get_base_type_name(stg_model_t* mod);

  /** get identifying index (if more than one model under a parent has the same base type
   * name). this is the number that is combined with the type name to 
   * create this model's unique token string.
   */
  size_t stg_model_get_instance_index(stg_model_t* mod);
   

  /** get a type identifier (actial name as used in world file, 
   * e.g. after "define foo position() foo()" foo
   * has instance type name "foo", not "position")
   */
  char* stg_model_get_instance_type_name(stg_model_t* mod);

  /** Get model's token (unique name) */
  char* stg_model_get_token(stg_model_t* mod);


  stg_world_t* stg_model_get_world(stg_model_t* mod);

  /** Add a new custom model type to the global type registry.
   *  @param type_name Type name (use this string in the world file)
   *  @param init_func  Initialization function for new models of this type.
   */
  void stg_model_register_custom_type(char* type_name, stg_model_initializer_t init_func, GData *init_properties);

  // SET properties - use these to set props, don't set them directly

  /** set the pose of model in global coordinates */
  int stg_model_set_global_pose( stg_model_t* mod, stg_pose_t* gpose );
  
  /** set a model's velocity in it's parent's coordinate system */
  int stg_model_set_velocity( stg_model_t* mod, stg_velocity_t* vel );
 
  /** @ingroup libstage
   *  @defgroup stg_model_locking Multi-thread access to a model 
   *  @brief Tools for accessing stage models from multiple threads.
   *
   *  If your program wishes to access model data from multiple threads, you
   *  must do four things. First, you must enable locking of models during 
   *  stg_world_update() by calling stg_world_set_model_updatelocking() Second, 
   *  you must enable the world mutex by calling stg_world_enable_world_lock() 
   *  with a TRUE argument. Thereafter, you must use stg_model_lock()
   *  and stg_model_unlock() before and after accessing any model, and
   *  stg_world_lock() and stg_world_unlock() before and after accessing the
   *  world data structures.
   *
   *  @{
   */
  /** Get exclusive access to a model, for threaded
      applications. Release with stg_model_unlock(). 
      Define STG_DEBUG_MODEL_LOCK to print messages when models are locked and unlocked.
   *  (Does nothing if model locking is not enabled at compile time.)
      @return false if the model mutex is invalid; this can happen if a model is destroyed while waiting to acquire the lock.
   */
  gboolean _stg_model_lock( stg_model_t* mod );
#ifdef STG_ENABLE_MODEL_LOCK
#ifdef STG_DEBUG_MODEL_LOCK
#define stg_model_lock(mod) (stg_print_message("stage: locking model %s in %s at %s:%d", stg_model_get_token(mod), __PRETTY_FUNCTION__, __FILE__, __LINE__),  _stg_model_lock(mod))
#else
#define stg_model_lock(mod) _stg_model_lock(mod)
#endif
#else
#define stg_model_lock(mod) {}
#endif

  /** Release exclusive access to a model, obtained with stg_model_lock() 
        Define STG_DEBUG_MODEL_LOCK to print messages when models are locked and unlocked.
   *  (Does nothing if model locking is not enabled at compile time.)
   */
  void _stg_model_unlock( stg_model_t* mod );
#ifdef STG_ENABLE_MODEL_LOCK
#ifdef STG_DEBUG_MODEL_LOCK
#define stg_model_unlock(mod) { stg_print_message("stage: unlocking model %s in %s at %s:%d", stg_model_get_token(mod), __PRETTY_FUNCTION__, __FILE__, __LINE__);  _stg_model_unlock(mod); }
#else
#define stg_model_unlock(mod) _stg_model_unlock(mod)
#endif
#else
#define stg_model_unlock(mod) {}
#endif

  /** Return type of stg_model_try_lock */
  enum stg_model_lock_status { STG_LOCK_SUCCESS, STG_LOCK_ALREADY_LOCKED, STG_LOCK_ERROR };

  /** If model is already locked, return STG_LOCK_ALREADY_LOCKED. Otherwise lock it and return STG_LOCK_SUCCESS.  */
  enum stg_model_lock_status stg_model_try_lock(stg_model_t* mod);

  /** @} */

  /** Change a model's parent - experimental*/
  int stg_model_set_parent( stg_model_t* mod, stg_model_t* newparent);
  
  void stg_model_get_geom( stg_model_t* mod, stg_geom_t* dest );

  void stg_model_get_velocity( stg_model_t* mod, stg_velocity_t* dest );
  
  /** Copy new data into a model's property. 
   *  @param mod The model
   *  @param prop The name of the property
   *  @param data Pointer to source data to copy from
   *  @param len Number of bytes to copy from source data into model's property
   
    @warning any extant pointers to old data may become invalid
      after this function if reallocated (depending on platform implementation of 
      realloc)
  */
  void stg_model_set_property( stg_model_t* mod, 
			       const char* prop, 
			       void* data, 
			       size_t len );  

  /** Lock the model, call stg_model_set_property, then unlock. 
   
    @warning any extant pointers to old data may become invalid
      after this function if reallocated (depending on platform implementation of 
  */
  #define stg_model_set_property_locked(mod, prop, data, len) { \
    stg_model_lock(mod);                                        \
    stg_model_set_property(mod, prop, data, len);               \
    stg_model_unlock(mod);                                      \
  }

  /** For this model and all of its descendents, set the named property to the
 * given value. New property will be set if the model or any descendent does not
 * yet have it */
  void stg_model_set_all_child_properties(stg_model_t *mod, const char* prop, void *data, size_t len);
  
  /** gets the named property data. if len is non-NULL, it is set with
      the size of the data in bytes. Use this for variable-length properties
      (strings, arrays). 
   
    @warning This pointer may become invalid after next update of model data 
      (during stg_world_update for example).
  */
  void* stg_model_get_property( stg_model_t* mod, 
				const char* prop,
				size_t* len );

  /** Evaluates to true if property with name @a propname exists on model @a mod. */
  #define stg_model_property_exists(mod, propname) (stg_model_get_property(mod, propname, NULL) != NULL)
  
  /** gets a property of a known size. Fails if the size isn't right.
   
    @warning This pointer may become invalid after next update of model data 
      (during stg_world_update for example).
   */
  void* stg_model_get_property_fixed( stg_model_t* mod, 
				      const char* name,
				      size_t size );

  /** Re set property with a new buffer copied from its existing data, and call
   * callbacks. Any pointers to the old data obtained from
   * stg_model_get_property() will be invalid.
   * @sa stg_model_property_changed()
   * Same as:
   * @code
   *   size_t len=0;
   *   void* data = stg_model_get_property( mod, propname, &len );
   *   void* tmp = malloc(len);
   *   memcpy(tmp,data,len);
   *   stg_model_set_property( mod, propname, tmp, len );
   *   free(tmp);
   * @endcode
   *
   * This causes callbacks to be called. Use if you have changed the property
   * data using a pointer recieved from stg_model_get_property() or 
   * stg_model_get_property_fixed().
   */
  void stg_model_property_refresh( stg_model_t* mod, const char* propname );

  /** Call this after directly changing property data buffer. It calls property's special
    * storage function if it has one, and calls its callbacks. If property
    * doesn't exist, do nothing.
    */
  void stg_model_property_changed(stg_model_t *mod, const char *propname);

  /// gets a model's "polygons" property and fills poly_count with the
  /// number of polygons to be found
  ///  @warning This pointer may become invalid after next update of model data 
  ///    (during stg_world_update for example).
  stg_polygon_t* stg_model_get_polygons( stg_model_t* mod, size_t* poly_count );
  void stg_model_set_polygons( stg_model_t* mod,
			       stg_polygon_t* polys, 
			       size_t poly_count );

  /// gets model's "points" property
  ///  @warning This pointer may become invalid after next update of model data 
  ///    (during stg_world_update for example).
  stg_point_t * stg_model_get_points(stg_model_t *mod, size_t *point_count);

  void stg_model_set_bg_figure(stg_model_t *mod);

  // get a copy of the property data - caller must free it
  //int stg_model_copy_property_data( stg_model_t* mod, const char* prop,
  //			   void** data );
  
  int stg_model_add_property_callback( stg_model_t* mod, const char* prop, 
				       stg_property_callback_t, void* user );
  
  int stg_model_remove_property_callback( stg_model_t* mod, const char* prop, 
					  stg_property_callback_t );
  
  int stg_model_remove_property_callbacks( stg_model_t* mod, const char* prop );

  /** print human-readable information about the model on stdout
   */
  void stg_model_print( stg_model_t* mod );
  void stg_model_print_tree(stg_model_t* mod);
  void stg_model_print_ex(stg_model_t* mod, stg_bool_t print_children, int indent);
  void model_print_pre_update( stg_model_t* mod );

  
  /** returns TRUE iff @a testmod exists above @a mod in a model tree 
   */
  int stg_model_is_antecedent( stg_model_t* mod, stg_model_t* testmod );
  
  /** returns TRUE iff @a testmod exists below @a mod in a model tree 
   */
  int stg_model_is_descendent( stg_model_t* mod, stg_model_t* testmod );
  
  /** returns TRUE iff @a mod1 and @a mod2 both exist in the same model
      tree 
  */
  int stg_model_is_related( stg_model_t* mod1, stg_model_t* mod2 );

  /** return the top-level model above mod */
  stg_model_t* stg_model_root( stg_model_t* mod );
  
  /** Get a model's children (models) list pointer. */
  GPtrArray* stg_model_get_children_ptr(stg_model_t* model);


  /** Create an array of all of a model's descendent models. You must later
   *  free the arry wit g_ptr_array_free(). */
  GPtrArray* stg_model_get_all_descendents(stg_model_t* root);
  
  /** initialize a model - called when a model goes from zero to one subscriptions */
  int stg_model_startup( stg_model_t* mod );

  /** finalize a model - called when a model goes from one to zero subscriptions */
  int stg_model_shutdown( stg_model_t* mod );

  /** Update a model by one simulation timestep. This is called by
      stg_world_update(), so users don't usually need to call this. 
      @return 0 if updated with no errors
  */
  int stg_model_update( stg_model_t* model );
  
  /** Update a model tree by one simulation timestep. Instead of
      updating one submodel, this will update a model and all of
      its children.

      Generally, this should only be called on the root model.
      If this is the case, pass NULL as the parent pointer.
      If starting with submodel, pass the model's parent pointer.  

      @return result of calling update function attached to @a mod,
              or 0 if no errors, 
              or -1 if validity checks on @a mod, @a mod's parent,
              and @a parent, fail.
  */
  int stg_model_update_tree( stg_model_t* mod, stg_model_t* parent );

  /** Convert a pose in the world coordinate system into a model's
      local coordinate system. Overwrites @a pose with the new
      coordinate. */
  void stg_model_global_to_local( stg_model_t* mod, stg_pose_t* pose );
  
  /** Convert a pose in the model's local coordinate system into the
      world coordinate system. Overwrites @a pose with the new
      coordinate. */
  void stg_model_local_to_global( stg_model_t* mod, stg_pose_t* pose );


  /** add an item to the View menu that will automatically install and
      remove a callback when the item is toggled. The specialized
      model types use this call to set up their data visualization. */
  void stg_model_add_property_toggles( stg_model_t* mod, 
				       const char* propname, 
				       stg_property_callback_t callback_on,
				       void* arg_on,
				       stg_property_callback_t callback_off,
				       void* arg_off,
				       const char* menu_label,
				       int enabled );
  void stg_model_remove_property_toggles(stg_model_t *mod, const char *propname);

  int stg_model_fig_clear_cb( stg_model_t* mod, void* data, size_t len, 
			      void* userp );

  char* stg_model_get_token(stg_model_t* mod);

  /// Debugging tool
  void stg_model_print_children(stg_model_t* model);

  stg_model_t* stg_model_find_first_child_with_type(stg_model_t* parent, char* type);
  stg_model_t* stg_model_find_last_child_with_type(stg_model_t* parent, char* type);

  typedef enum {
      STG_INT,    ///< Value is an int.
      STG_FLOAT,  ///< Value is a float.
      STG_DOUBLE, ///< Value is a double.
      STG_STRING  ///< Value is a NULL-terminated char*.
  } stg_datatype_t;

  /** Causes Stage to automatically create a new property when creating a new
   model, with the property value read from the world file. This is a way
   for applications using libstage to attach any information to a model
   supplied by the user in a model definition in the world file.
   @param modeltype Name of the model the property applies to (e.g. "position")
   @param propertyname Name of the property to create; also the name of the property in the world file
   @param datatype Data type of the property value.
  */
  void stg_model_user_property(const char* model_type, const char* property_name, stg_datatype_t datatype);

  /** @} */   // end model doc group

 //  WORLD --------------------------------------------------------

  /** @ingroup libstage
   *  @defgroup stg_world World
     @brief Implements the world - a collection of models and a matrix
       containing current model geometry positions for efficient interaction detection.
     @{
  */

  extern const double STG_DEFAULT_RESOLUTION;  ///< 2cm pixels
  extern const double STG_DEFAULT_INTERVAL_REAL; ///< default msec between updates
  extern const double STG_DEFAULT_INTERVAL_SIM;  ///< default simulated duration of an update in msec
  extern const double STG_DEFAULT_WORLD_WIDTH; ///< meters
  extern const double STG_DEFAULT_WORLD_HEIGHT; ///< meters
  
  /** Create a new world, to be configured and populated
      manually. Or, use the
      function stg_world_create_from_file() to create a world based on
      a worldfile instead.
      Does not set up the GUI's configuration, that must be done seperately.
   */
  stg_world_t* stg_world_create( stg_id_t id, 
				 const char* token, 
				 int sim_interval, 
				 int real_interval,
				 double ppm,
				 double width,
				 double height );

  /** Create a new world as described in the world file @a worldfile_path.
   *  Stage can only have one world file active at a time.
   *  Any old world information will be discarded and the new world file
   *  loaded. 
   *  If any model instances are given in the world file they will be added
   *  to the new world.
   */
  stg_world_t* stg_world_create_from_file( const char* worldfile_path, int echo, void  (*loop_callback)() );

  /** Destroy a world and everything it contains
   */
  void stg_world_destroy( stg_world_t* world );

  /** Clear old world information and load a new world file. If any
   *  models are instantiated in the new world file, they will be
   *  added to the given world.
   *  @return FALSE on error, TRUE otherwise.
   */
  gboolean stg_world_load_world_file(stg_world_t *world, const char *file, int echo);
  
  /** Stop the world clock
   */
  void stg_world_stop( stg_world_t* world );
  
  /** Start the world clock
   */
  void stg_world_start( stg_world_t* world );

  /** Get cumulative world run time. */
  stg_msec_t stg_world_get_time(stg_world_t* world);

  /** Run one simulation step in world @a world. You must call
   *  this function repeatedly for the stage simulation and GUI to
   *  run properly.
      @param sleepflag if TRUE, and the simulation update takes less than one
      real-time step, this function will sleep until it's time for the update.
      @param skiptooson if TRUE, and @a sleepflag is FALSE, return if it's not
      yet time for an update. If FALSE, and @a sleepflag is FALSE,
      always do the update.  If you are doing your own timing/scheduling, set
      both @a sleepflag and @a skiptoosoon to FALSE.
      @return 0 if all is well, and the simulation
      should continue, negative numbers on errors, or positive numbers
      if a quit request was registered internally or by the stg_quit()
      function.
   */
  int stg_world_update(stg_world_t* world, stg_bool_t sleepflag, stg_bool_t skiptooson);

  /** Run in a loop, calling stg_world_update with sleepflag set to TRUE until it returns a nonzero value.
      Return that value.
   */
  int stg_world_run(stg_world_t *world);

  /** Create a new model in the world. The model is created and set up according
   * to its type, including loading properties from worldfile macro definitions
   * if this type was defined with "define" macros. However, after creating the 
   * new model, you will also have to set properties unique to this model 
   * instance (according to its type).
   *
   *  @param world The world
   *  @param type  The model type string.  Use "model" for a base model with no
   *   specific type.
   *  @param parent A parent model. Use NULL for none.
   *  @return the newly created model, or NULL if there was an error creating
   *  it. (Most common, no model definition for the given type.)
   */
  stg_model_t* stg_world_new_model(stg_world_t* world, const char* type, stg_model_t* parent, const char *requestedName);

  /** @copydoc stg_world_new_model
   *  @param token  If non-null, Use this instead of autogenerating a token (unique model name)
   *  @param world_id If not -1, then use this identifier to find this model's
   *    properties in the world file, and load model properties from
   *    this worldfile section. If -1, then only load properties defined for macros
   *    this model type uses, if any, and generate a new unique ID. you will then have to set any properties
   *    unique to this new model instance using stg_model_set_property() etc.
   *    (this ID number is also sometimes called a model's world file "section"
   *    id)
   *  @param parsing_worldfile When used internally parsing the worldfile (a
   *    special case), then
   *    this flag is set. If called by anything other than
   *    stg_world_load_from_file(), this really must be FALSE.
   */
  stg_model_t* stg_world_new_model_ex(stg_world_t* world, const char* type, const char* token, stg_model_t* parent, stg_id_t world_id, gboolean parsing_worldfile);

  /** Adds a new type definition to the world "file". Returns the section/type
   * id for the new type. You can then use stg_world_set_type_property to set a
   * property value. 
   * If parent is -1, then a top-level entry is made.
   */
  int stg_world_add_type_def(stg_world_t *world, const char *type, int parent);

  gboolean stg_world_type_defined(stg_world_t *world, const char *type);

  /** Adds a property values to a type definition in the world "file". */
  //@{
  void stg_world_set_type_property_int(stg_world_t *world, int wf_section_id, char *name, int value);
  void stg_world_set_type_property_float(stg_world_t *world, int wf_section_id, char *name, double value);
  void stg_world_set_type_property_string(stg_world_t *world, int wf_section_id, char *name, char *value);
  void stg_world_set_type_property_tuple_int(stg_world_t *world, int wf_section_id, char *name, int which, int value);
  void stg_world_set_type_property_tuple_float(stg_world_t *world, int wf_section_id, char *name, int which, double value);
  void stg_world_set_type_property_tuple_string(stg_world_t *world, int wf_section_id, char *name, int which, char* value);
  //@}


  /** Remove the given model from the world. Does not destroy the model. */
  void stg_world_remove_model( stg_world_t* world, stg_model_t* mod );

  /** add a model to the world's tables. Mostly internal, usually you would load
   * all models from the world file, or use stg_world_new_model() 
   * @internal
   */
  void stg_world_add_model( stg_world_t* world, stg_model_t* mod  );

  /** Reconfigure the world by reading from the current world file */
  void stg_world_load( stg_world_t* world );

  /** save the state of the world to the current world file */
  void stg_world_save( stg_world_t* world );

  /** print human-readable information about the world on stdout 
   *  (for debuging etc.)
   */
  void stg_world_print( stg_world_t* world );

  /** Set the duration in milliseconds of each simulation update step 
   */
  void stg_world_set_interval_real( stg_world_t* world, stg_msec_t val );
  
  /** Set the real time in intervals that Stage should attempt to take
      for each simulation update step. If Stage has too much
      computation to do, it might take longer than this. */
  void stg_world_set_interval_sim( stg_world_t* world, stg_msec_t val );

  stg_msec_t stg_world_get_sim_interval(stg_world_t* world);
  stg_msec_t stg_world_get_real_interval(stg_world_t* world);
  stg_msec_t stg_world_get_last_interval(stg_world_t* world);
  double stg_world_get_avg_interval_ratio(stg_world_t* world);
  size_t stg_world_num_zero_interval_warnings(stg_world_t* world);
  size_t stg_world_num_interval_too_long_warnings(stg_world_t* world);

  /** look up a pointer to a model in @a world from the model's unique
      ID @a mid. */ 
  stg_model_t* stg_world_get_model( stg_world_t* world, stg_id_t mid );
  
  /** look up a pointer to a model from from the model's name. */
  stg_model_t* stg_world_model_name_lookup( stg_world_t* world, const char* name );

  /** Reset all models to their original poses */
  void stg_world_reset_all_model_poses(stg_world_t *world);

  /** Reset models with the given type to their original poses */
  void stg_world_reset_model_type_poses(stg_world_t *world, const char *type);

  /** Reset position models to their original poses */
  void stg_world_reset_position_model_poses(stg_world_t *world);
  
  /** Represent ways for stage to do locking during stg_world_update(). 
   *  @see stg_world_set_model_locking_policy()
   */
  typedef enum { 
    STG_WORLD_NO_LOCKING,     ///< Never lock any models. Only use in single-threaded programs. 
    STG_WORLD_LOCK,           ///< Always lock each model, and the worla, in stg_world_update()d
    STG_WORLD_SKIP_IF_LOCKED  ///< If a model is already locked in stg_world_update(), then do not update it this iteration. @warning this breaks synchronization between models each update! Use this policy with care! Also lock the world during update.
  } stg_world_locking_policy_t;

  /** Set locking policy used by stg_world_update(). By default, no locking is done. However, if
   * your program will be accessing stage models through multiple threads, you
   * must enable a locking policy. See the description of multithreaded access
   * to Stage in the reference below:
   * @see @ref stg_model_locking
   */
#ifdef STG_ENABLE_WORLD_LOCK
  void stg_world_set_model_update_locking_policy(stg_world_t* world, stg_world_locking_policy_t policy);
#else
#define stg_world_set_model_update_locking_policy(w, p) {}
#endif


  /** Enable locking (stg_world_lock() will actually lock) and initialize the
   * mutex. (Has no effect if world locking is not enabled at compile time.)
   */
#ifdef STG_ENABLE_WORLD_LOCK
  void stg_world_enable_world_lock(stg_world_t *world, gboolean enable);
#else
#define stg_world_set_model_update_locking_policy(w, p) {}
#endif

  /** @define stg_world_lock(world)
   * Lock the world mutex, but only if world locking was enabled by
   * stg_world_enable_world_lock(). Use this if accessing world datastructures from more
   *  than one thread.   (Does nothing if world locking is not enabled at
   *  compile time.)
   */
#ifdef STG_ENABLE_WORLD_LOCK
  void _stg_world_lock(stg_world_t *world);
#if defined(STG_DEBUG_WORLD_LOCK_VERBOSE)
#define stg_world_lock(world) { stg_print_message("stage: locking world in %s at %s:%d", __PRETTY_FUNCTION__, __FILE__, __LINE__);  _stg_world_lock(world); }
#else
#define stg_world_lock(world) _stg_world_lock(world)
  stg_msec_t stg_world_get_lock_time(stg_world_t *world);
#endif
#else
#define stg_world_lock(world) {}
#endif


/** @define STG_WORLD_MUTEX_LOCK_WARNING_TIME
 *  If the world mutex is locked for more than this many ms, print a warning. 
 */
#define STG_WORLD_MUTEX_LOCK_WARNING_TIME 200

  /** @define stg_world_unlock(world)
   *  Unlock the world mutex, but only if world locking was enabled by
   *  stg_world_enable_world_lock().
      Define STG_DEBUG_WORLD_LOCK_VERBOSE to print messages whenever the world is locked and unlocked, and STG_DEBUG_WORLD_LOCK_TIME to receive warnings when the world was locked for more than STG_WORLD_MUTEX_LOCK_WARNING_TIME
   *  (Does nothing if world lockingis not enabled at compile time.)
   *  @sa stg_world_lock()
   */
#ifdef STG_ENABLE_WORLD_LOCK
  void _stg_world_unlock(stg_world_t *world);
#ifdef STG_DEBUG_WORLD_LOCK_VERBOSE
#define stg_world_unlock(world) { stg_print_message("stage: unlocking world in %s at %s:%d", __PRETTY_FUNCTION__, __FILE__, __LINE__);  _stg_world_unlock(world); }
#elif defined(STG_DEBUG_WORLD_LOCK_TIME)
#define stg_world_unlock(world) {\
  _stg_world_unlock(world); \
  stg_msec_t _stg_world_unlock_time = stg_timenow(); \
  stg_msec_t _stg_world_unlock_dur; \
  if(_stg_world_unlock_time > stg_world_get_lock_time(world) && (_stg_world_unlock_dur = (_stg_world_unlock_time - stg_world_get_lock_time(world))) > STG_WORLD_MUTEX_LOCK_WARNING_TIME) \
    stg_print_warning("(debug) world mutex was locked for %lu ms! (at %s in %s:%d)", _stg_world_unlock_dur, __PRETTY_FUNCTION__, __FILE__, __LINE__); \
}
#else
#define stg_world_unlock(world) _stg_world_unlock(world)
#endif
#else
#define stg_world_unlock(world) {}
#endif


  /** If true, supress some verbose log messages. */
  void stg_world_set_quiet(stg_world_t* world, gboolean b);

  gboolean stg_world_get_quiet(stg_world_t* world);


  /** install a property callback on every model in the world that
      CURRENTLY has this property set. Calls
      stg_model_add_property_callback() on each model in the world.*/
  void stg_world_add_property_callback( stg_world_t* world, 
					char* propname, 
					stg_property_callback_t callback, void*
					userdata );
  
  /** remove a property callback from every model in the world that
      has this property set. Calls
      stg_model_remove_property_callback() on each model in the
      world. */
  void stg_world_remove_property_callback( stg_world_t* world,
					   char* propname,
					   stg_property_callback_t callback );

   /// Use to iterate over each top-level model in the world
  void stg_world_foreach_model_by_name(stg_world_t* world, void(*func)(stg_model_t* model, char* name, void* user_data), void* user_data);
  
  /// Set the title of the (system) window showing this world
  void stg_world_set_window_title(stg_world_t* world, const char* str);

  /// Pause or unpause the simulation
  void stg_world_pause(stg_world_t* world, int paused);

  /// Set the world window's cursor
  void stg_world_set_cursor_busy(stg_world_t* world);

  /// Set the world window's cursor
  void stg_world_set_cursor_normal(stg_world_t* world);

  void stg_world_window_request_maximize(stg_world_t* world);

  void stg_world_window_request_minimize(stg_world_t* world);

  void stg_world_window_request_fullscreen(stg_world_t* world);

  /// Note see GTK documentation for GtkWindow functions gtk_window_get_size() and gtk_window_get_position().
  void stg_world_window_get_geometry(stg_world_t* world, int* x, int *y, int *width, int *height);

  void stg_world_set_fill_polygons(stg_world_t* world, gboolean fill);

  void stg_world_hide_all_graphics(stg_world_t* world);

  void stg_world_disable_gui(stg_world_t *world);

  gboolean stg_world_gui_enabled(stg_world_t *world);

  void stg_world_set_mouse_buttons(stg_world_t* world, int panbutton, int zoombutton);

  /** Change the size of the world. Events between models outside the world
   * extents may be ignored, so the world size defines the usable space in the
   * world.  Internal data structures may need to be re-created, so this is
   * potentially a very time consuming process.
   * @sa stg_world_resize_to_contents()
   */
  void stg_world_resize(stg_world_t* world, stg_meters_t new_width, stg_meters_t new_height);

  /** Resize of the world to include all current contents, plus extra padding
   * around the edge given by @a border_padding.
   * @sa stg_world_resize() 
   */
  void stg_world_resize_to_contents(stg_world_t* world, stg_meters_t border_padding);

  /** Calculate the bounding edges of all the models in the world. */
  void stg_world_dimensions(stg_world_t* world, double* min_x, double* min_y, double* max_x, double* max_y);

  /** Change the world's resolution. All models must be re-mapped into a new
   * matrix, so this may take some time.
   * @param res Matrix cell smallest size limit in meters.
   */
  void stg_world_change_resolution(stg_world_t* world, double res);

  /** A callback function to load data from a file into a world. */
  typedef int (*stg_world_file_loader_callback_t)(stg_world_t* world, char* filename, void* userdata);

  /** Add a callback function for loading a type of file. When at least one
   *  loader is available, then a menu item is included in the world's GUI.
   *  @param world The world
   *  @param callback Pointer to the callback function
   *  @param pattern A pattern to match on filename using typical shell glob matching. E.g. *.map matches foo.map and bar.map.
   *  @param userdata Whatever you want, will be passed to the callback function.
   *
   */
  void stg_world_add_file_loader(stg_world_t* world, stg_world_file_loader_callback_t callback, const char* pattern, const char* name, void* userdata);


  void stg_world_log_stats(stg_world_t *world);
  /**@}*/


  // BLOBFINDER MODEL --------------------------------------------------------
#ifdef ENABLE_BLOBFINDER_MODEL
  
  /** @ingroup libstage
   *  @defgroup stg_model_blobfinder Blobfinder Model
      @brief Implements the blobfinder  model.
      @{ */
  
#define STG_BLOB_CHANNELS_MAX 16
  
  /** blobfinder config packet
   */
  typedef struct
  {
    int channel_count; // 0 to STG_BLOBFINDER_CHANNELS_MAX
    stg_color_t channels[STG_BLOB_CHANNELS_MAX];
    int scan_width;
    int scan_height;
    stg_meters_t range_max;
    stg_radians_t pan, tilt, zoom;
  } stg_blobfinder_config_t;
  
  /** blobfinder data packet 
   */
  typedef struct
  {
    int channel;
    stg_color_t color;
    int xpos, ypos;   // all values are in pixels
    //int width, height;
    int left, top, right, bottom;
    int area;
    stg_meters_t range;
  } stg_blobfinder_blob_t;

  /** Create a new blobfinder model 
   */
  stg_model_t* stg_blobfinder_create( stg_world_t* world,	
				      stg_model_t* parent, 
				      stg_id_t id,  
				      char* token );   
  /**@}*/
#endif

  // LASER MODEL --------------------------------------------------------
  
  /** @ingroup libstage
   *  @defgroup stg_model_laser Laser range scanner model
      @brief Implements the laser model: emulates a scanning laser rangefinder
      @{ */
  
  /** laser sample packet
   */
  typedef struct
  {
    uint32_t range; ///< range to laser hit in mm
    char reflectance; ///< intensity of the reflection 0-4
  } stg_laser_sample_t;

  /** Rule for determining detected return value of an object. */
  typedef struct _stg_laser_rule_struct
  {
    /* Model's laser_return value preconditions. At least one ofthese must match for the rule to be
     * considered. If any of these are -1 then it is not used. */
    stg_laser_return_t model_value_eq;
    stg_laser_return_t model_value_gt;
    stg_laser_return_t model_value_lt;

    /* Laser-model spatial relationship precondition. (Only one can be used per rule)*/
    enum { 
      STG_LASER_RULE_COND_NONE,
      STG_LASER_RULE_COND_OUTSIDE_RANGE, 
      STG_LASER_RULE_COND_OUTSIDE_ANGLE 
    } condition;
    union {
      stg_radians_t angle;
      stg_meters_t range;
    } condition_value;

    /* Result: */
    enum {
      STG_LASER_RULE_RETURN_VALUE
    } result;
    union {
      stg_laser_return_t detect;
    } result_value;

    /* Whether to stop evaluating rules if this rule is used */
    gboolean stop;

    /* Next rule: */
    struct _stg_laser_rule_struct *next;
  } stg_laser_rule_t;
  
  /** laser configuration packet
   */
  typedef struct
  {
    stg_radians_t fov; ///< field of view 
    stg_meters_t range_max; ///< the maximum range
    stg_meters_t range_min; ///< the miniimum range

    /** the number of range measurements (and thus the size
    of the array of stg_laser_sample_t's returned) */ 
    int samples; 

    /** If true, a sweep of laser readings starts on the "right", or at the
     * maximum angle. If false, it's the other way (e.g. a SICK mounted
     * upside-down would have this true).
     */
    stg_bool_t reverse_scan;

    /** Static random noise on range readings (meters). To each reading range, 
     * a random number between 0-noise and 0+noise will be added, before converting to mm. */
    double noise;

    /** Range of random error (noise) added to the angle (radians) at which an individual 
     * reading is taken. 
     */
    double reading_angle_error;


    /** List of return-value rules */
    stg_laser_rule_t* rules;

  } stg_laser_config_t;


  
  /** print human-readable version of the laser config struct
   */
  void stg_print_laser_config( stg_laser_config_t* slc );
  
  /** Create a new laser model 
   */
  stg_model_t* stg_laser_create( stg_world_t* world, 
				 stg_model_t* parent, 
				 stg_id_t id,
				 char* token );

  /**@}*/

  // GRIPPER MODEL --------------------------------------------------------
  
#ifdef ENABLE_GRIPPER_MODEL

  /** @ingroup libstage
   *  @defgroup stg_model_gripper Gripper model
      @brief Implements the gripper model: a fixed two-degree-of-freedom manipulator that can close
        to grasp an carry other models.
      @{ */
  
  typedef enum {
    STG_GRIPPER_PADDLE_OPEN = 0, // default state
    STG_GRIPPER_PADDLE_CLOSED, 
    STG_GRIPPER_PADDLE_OPENING,
    STG_GRIPPER_PADDLE_CLOSING,
  } stg_gripper_paddle_state_t;

  typedef enum {
    STG_GRIPPER_LIFT_DOWN = 0, // default state
    STG_GRIPPER_LIFT_UP, 
    STG_GRIPPER_LIFT_UPPING, // verbed these to match the paddle state
    STG_GRIPPER_LIFT_DOWNING, 
  } stg_gripper_lift_state_t;
  
  typedef enum {
    STG_GRIPPER_CMD_NOP = 0, // default state
    STG_GRIPPER_CMD_OPEN, 
    STG_GRIPPER_CMD_CLOSE,
    STG_GRIPPER_CMD_UP, 
    STG_GRIPPER_CMD_DOWN    
  } stg_gripper_cmd_type_t;
  
  /** gripper configuration packet
   */
  typedef struct
  {
    stg_size_t paddle_size; ///< paddle dimensions 

    stg_gripper_paddle_state_t paddles; 
    stg_gripper_lift_state_t lift;

    double paddle_position; ///< 0.0 = full open, 1.0 full closed
    double lift_position; ///< 0.0 = full down, 1.0 full up

    stg_meters_t inner_break_beam_inset; ///< distance from the end of the paddle
    stg_meters_t outer_break_beam_inset; ///< distance from the end of the paddle  
    stg_bool_t paddles_stalled; // true iff some solid object stopped
				// the paddles closing or opening
    
    GSList *grip_stack;  ///< stack of items gripped
    int grip_stack_size; ///< maximum number of objects in stack, or -1 for unlimited

  } stg_gripper_config_t;

  /** gripper command packet
   */
  typedef struct
  {
    stg_gripper_cmd_type_t cmd;
    int arg;
  } stg_gripper_cmd_t;


  /** gripper data packet
   */
  typedef struct
  {
    stg_gripper_paddle_state_t paddles; 
    stg_gripper_lift_state_t lift;
    
    double paddle_position; ///< 0.0 = full open, 1.0 full closed
    double lift_position; ///< 0.0 = full down, 1.0 full up

    stg_bool_t inner_break_beam; ///< non-zero iff beam is broken
    stg_bool_t outer_break_beam; ///< non-zero iff beam is broken
    
    stg_bool_t left_paddle_contact[3]; ///< non-zero iff left paddle touches something [1] inner, [2] front [3] outer contacts
    stg_bool_t right_paddle_contact[3]; ///< non-zero iff right paddle touches something[1] inner, [2] front [3] outer contacts
    
    stg_bool_t paddles_stalled; // true iff some solid object stopped
				// the paddles closing or opening

    int stack_count; ///< number of objects in stack

  } stg_gripper_data_t;


  /** print human-readable version of the gripper config struct
   */
  void stg_print_gripper_config( stg_gripper_config_t* slc );
  
  /** Create a new gripper model 
   */
  stg_model_t* stg_gripper_create( stg_world_t* world, 
				 stg_model_t* parent, 
				 stg_id_t id,
				 char* token );
  /**@}*/

#endif

  // FIDUCIAL MODEL --------------------------------------------------------

#ifdef ENABLE_FIDUCIAL_MODEL
  
  /** @ingroup libstage
   *  @defgroup stg_model_fiducial Fidicial detector model
      @brief Implements the fiducial detector model. This model scans
        other models which contain ID codes (numbers) wthin its field of view, and provides a set of 
        ID codes for detected models.
      @{ */

  /** any integer value other than this is a valid fiducial ID 
   */
  // TODO - fix this up
#define FiducialNone 0

  /** fiducial config packet 
   */
  typedef struct
  {
    stg_meters_t max_range_anon;
    stg_meters_t max_range_id;
    stg_meters_t min_range;
    stg_radians_t fov; // field of view 
    stg_radians_t heading; // center of field of view

  } stg_fiducial_config_t;
  
  /** fiducial data packet 
   */
  typedef struct
  {
    stg_meters_t range; // range to the target
    stg_radians_t bearing; // bearing to the target 
    stg_pose_t geom; // size and relative angle of the target
    int id; // the identifier of the target, or -1 if none can be detected.
    
  } stg_fiducial_t;

  /** Create a new fiducial model 
   */
  stg_model_t* stg_fiducial_create( stg_world_t* world,  
				    stg_model_t* parent,  
				    stg_id_t id, 
				    char* token );  
  /**@}*/

#endif
  
  // RANGER MODEL --------------------------------------------------------
  
  /** @ingroup libstage
   *  @defgroup stg_model_ranger Range finder model
      @brief Implements the ranger model: emulates
      sonar, IR, and other non-scanning range sensors 
      @{ */

  typedef enum {
    STG_RANGER_SINGLE_RAY,
    STG_RANGER_CLOSEST_RAY
  } stg_ranger_projection_t;

  typedef struct
  {
    stg_pose_t pose;
    stg_size_t size;
    stg_bounds_t bounds_range;
    stg_radians_t fov;
    double noise;
    stg_ranger_projection_t projection_type;
    double resolution;
    char enable_throwaway;
    double throwaway_thresh;
    double throwaway_prob;
  } stg_ranger_config_t;
  
  typedef struct
  {
    stg_meters_t range;
    //double error; // TODO
    gboolean intersect_flag;
    stg_radians_t intersect_angle;
  } stg_ranger_sample_t;
  
  /** Create a new ranger model 
   */
  stg_model_t* stg_ranger_create( stg_world_t* world,  
				  stg_model_t* parent, 
				  stg_id_t id, 
				  char* token );
  /**@}*/
  
  // POSITION MODEL --------------------------------------------------------
  
  /** @ingroup libstage
   *  @defgroup stg_model_position Position model
      @brief Implements the position model: a model that can move itself either in any direction
       (omni) or like a differential-drive robot (diff), and estimate its position with optional
       error; This is used for the mobile robot base.
      @{ */
  
  //#define STG_MM_POSITION_RESETODOM 77
  
  typedef enum
    { STG_POSITION_CONTROL_VELOCITY, STG_POSITION_CONTROL_POSITION, STG_POSITION_CONTROL_RELATIVE, STG_POSITION_CONTROL_IDLE }
  stg_position_control_mode_t;
  
#define STG_POSITION_CONTROL_DEFAULT STG_POSITION_CONTROL_VELOCITY

  const char* stg_position_control_mode_name(stg_position_control_mode_t m);
  
  typedef enum
    { STG_POSITION_LOCALIZATION_GPS, STG_POSITION_LOCALIZATION_ODOM }
  stg_position_localization_mode_t;
  
#define STG_POSITION_LOCALIZATION_DEFAULT STG_POSITION_LOCALIZATION_GPS
  
  /** "position_drive" property */
  typedef enum
    { STG_POSITION_DRIVE_DIFFERENTIAL, STG_POSITION_DRIVE_OMNI }
  stg_position_drive_mode_t;
  
#define STG_POSITION_DRIVE_DEFAULT STG_POSITION_DRIVE_DIFFERENTIAL
  
  /** "position_cmd" property. If @a override_decel is
   * > 0, use them instead of configured values. */
  typedef struct
  {
    stg_meters_t x,y,a; 
    stg_position_control_mode_t transmode, rotmode;
    stg_velocity_t override_decel;
  } stg_position_cmd_t;

  typedef enum { 
    STG_POSITION_ODOM_ERROR_RANDOM_INIT, 
    STG_POSITION_ODOM_ERROR_RANDOM_EACH_UPDATE, 
    STG_POSITION_ODOM_ERROR_CONSTANT, 
    STG_POSITION_ODOM_ERROR_NONE,
    STG_POSITION_ODOM_ERROR_INVALID
  } stg_position_odom_error_mode_t;
  
  /** "position_data" property */
  typedef struct
  {
    stg_pose_t pose; ///< position estimate in local coordinates
    stg_pose_t pose_error; ///< estimated error in position estimate
    stg_pose_t origin; ///< global origin of the local coordinate system
    stg_velocity_t velocity; ///< current translation and rotaation speeds
    stg_position_odom_error_mode_t integration_error_mode; ///< How to accumulate odometry integration error
    stg_velocity_t integration_error; ///< constant error or last randomly chosen error in simple odometry model
    stg_velocity_t max_integration_error; ///< range max for randomly generated integration error, if using random
    stg_velocity_t min_integration_error; ///< range min for randomly generated integration error, if using random
    //stg_bool_t stall; ///< TRUE iff the robot can't move due to a collision
    stg_position_localization_mode_t localization; ///< global or local mode
  } stg_position_data_t;
  
  /** "position_stall" property */
  typedef int stg_position_stall_t;

  /** "position_speed_config" property */
  typedef struct
  {
    stg_pose_t max_speed;     ///< Maximum velocities
    stg_pose_t default_speed; ///< Used for position control modes
    stg_pose_t max_accel;     ///< Maximum accelerations
    stg_pose_t max_decel;     ///< Maximum decelerations
    stg_pose_t current_accel; ///< Currently set accelerations
    stg_pose_t current_decel; ///< Currently set decelerations
  } stg_position_speed_config_t;

  /// create a new position model
  stg_model_t* stg_position_create( stg_world_t* world,  stg_model_t* parent,  stg_id_t id, char* token );
  
  /// set the current odometry estimate 
  void stg_model_position_set_odom( stg_model_t* mod, stg_pose_t* odom ); 


  /// global setting specifies how to apply odometry error.
  /// RANDOM_INIT was the only behavior before AMRISim 0.9.8.
  /// RANDOM_EACH_UPDATE is the new default if not set by user code.

  const char *stg_position_odom_error_mode_name(stg_position_odom_error_mode_t mode);
  void stg_position_force_odom_error_mode_all_models(stg_position_odom_error_mode_t mode);
  stg_position_odom_error_mode_t stg_position_get_forced_odom_error_mode_all_models();


  /// Turn all of a model's property gui toggles on or off
  void stg_model_all_property_toggles(stg_model_t* mod, gboolean on);


  /** Return TRUE if a model has a height property. If a model has no height
   * property, then all height rules should apply to it as well; it should be
   * considered "infinitely" tall.
   */
  //gboolean stg_model_has_height(stg_model_t *model);
  #define stg_model_has_height(model) (stg_model_get_property(model, "height", NULL) != NULL)

  /// Set the height of this model, and recalculate its sum height in the world
  void stg_model_set_height(stg_model_t *model, stg_meters_t h);

  /** Get the height of a model, or 0.0 if it has no height property (check with
   * stg_model_has_height().) This height is just the height of the model in isolation,
   * not as attached to other models-- see stg_model_get_height_in_world() for
   * that.
   */
  stg_meters_t stg_model_get_height(stg_model_t *model);

  /// Recalculate this model's height in the world, i.e. the sum of its height, its vertical offset, and all of its parent model's heights-in-the-world.
  void stg_model_recalc_height_in_world(stg_model_t *model) ;


  /** Get the height of this model in the world (i.e. the sum of this model's and
    * all of its parent models's height and vertical offset properties.
    * Calculate and store this value in a property if it's missing.
    * You should use see if a model has a height property with
    * stg_model_has_height() before calling this function, or the hight
    * will be set to 0.0.  If a model has no height property, then any height
    * checks should apply to it as well; i.e. it should be "infinitely" tall.
    */
  stg_meters_t stg_model_get_height_in_world(stg_model_t *model);

  /**@}*/

  // BUMPERS MODEL --------------------------------------------------------
  
  /** @ingroup libstage
   *  @defgroup stg_model_bumpers Bumpers model
      @brief Implements the bumpers model: outward-facing switches that are activated by 
      collision. 
      
      It's basically a simplified ranger device.
      The "bumpers_cfg" property contains overall bumper array settings
      The "bumpers_switch_poses" property contains an array of switch specific configs 
      The "bumpers_data" property contains a byte for each bumper switch (1 or 0)
      @{ */

  /** "bumpers_config" property */
  typedef struct
  {
    unsigned int num_bumpers; ///< how many bumpers there are
    stg_bool_t autostop; ///< whether to automatically stop the parent position model if a bumper is triggered.
    stg_bool_t autostall; ///< Whether to automatically set "collision" flag on parent model
  } stg_bumpers_config_t;

  /** for each "bumpers_switch_cfg" property */
  typedef struct
  {
    stg_pose_t pose;
    stg_size_t size;
  } stg_bumpers_switch_config_t;
  

  /** Return whether a particular bumper is triggered. */
  stg_bool_t stg_bumper_triggered(stg_model_t* model, unsigned int which);

  /* @} */


  // MESSAGING  --------------------------------------------------------

  /** @ingroup libstage
   *  @defgroup stg_model_messages Messages model
   *  @brief Displays messages for the user in the world GUI.
    @{ */

  typedef enum { STG_MSG_CRITICAL, STG_MSG_WARNING, STG_MSG_INFORMATION } stg_message_level_t;

  typedef struct {
    volatile int displayed;
    time_t timestamp; /// Timestamp (from time()) to distinguish new messages
    char  category[32];  /// Type or origin of message
    stg_message_level_t level; /// Level
    char message[1024];  
  } stg_message_t;

  /** Send a new message to a messages model, if possible, and also print to console. If model is
   * NULL, then the message is only printed to the console. When printing to
   * the console, stderr is used for errors and warnings, stdout for all other levels).
   * Color and the text "Warning" differentiates warnings. If the messages model
   * exists, then the message will be sen there (and it will display the message
   * both in the GUI and on the console). In the GUI a critical
   * error creates a dialog box with the option to exit (default) or continue
   * anyway.
   * @return the message property, if the message was sent to the messages
   * model, else NUL
   */
  stg_message_t* stg_messages_send(stg_model_t* model, const char* category, stg_message_level_t level, const char* fmt, ...);
  /// same as stg_messages_send() but takes just a string
  stg_message_t* stg_messages_send_s(stg_model_t* model, const char* category, stg_message_level_t level, const char* message);

  // end messages
  /**@}*/
  
  // end the group of all models
  /**@}*/
  

// MACROS ------------------------------------------------------
// Some useful macros



/** 
@ingroup util
@defgroup floatcomparison Floating point comparisons

 @brief Macros for comparing floating point numbers. 
 
 It's a troublesome
 limitation of C and C++ that floating point comparisons are not very
 accurate. These macros multiply their arguments by a large number
 before comparing them, to improve resolution.

  @{
*/

/** Precision of comparison. The number of zeros to the left of the
   decimal point determines the accuracy of the comparison in decimal
   places to the right of the point. E.g. precision of 100000.0 gives
   a comparison precision of within 0.000001 */
#define PRECISION 100000.0

/** TRUE iff A and B are equal to within PRECISION */
#define EQ(A,B) ((lrint(A*PRECISION))==(lrint(B*PRECISION)))

/** TRUE iff A is less than B, subject to PRECISION */
#define LT(A,B) ((lrint(A*PRECISION))<(lrint(B*PRECISION)))

/** TRUE iff A is greater than B, subject to PRECISION */
#define GT(A,B) ((lrint(A*PRECISION))>(lrint(B*PRECISION)))

/** TRUE iff A is greater than or equal B, subject to PRECISION */
#define GTE(A,B) ((lrint(A*PRECISION))>=(lrint(B*PRECISION)))

/** TRUE iff A is less than or equal to B, subject to PRECISION */
#define LTE(A,B) ((lrint(A*PRECISION))<=(lrint(B*PRECISION)))

/** generate a random double precision floating point number between 0.0 and
 * 1.0. uses drand48() if available, otherwise rand()/RAND_MAX. */
#ifdef HAVE_DRAND48
#include <stdlib.h>
#define STG_RAND() (drand48())
#else
#define STG_RAND() ((double)rand()/(double)RAND_MAX)
#endif


/** generate a random double in the range (min,max) */
#define STG_RANDOM_IN(min, max) ( \
  assert(min <= max), \
  (min == max) ? min : \
  ( min + ( STG_RAND() * (max-min) ) ) \
)

/* generate a random double in the range (-err/2,+err/2) */
// not used #define STG_RANDOM_ERR(err) (((double)(rand() - RAND_MAX/2) / (double)RAND_MAX) * err)

/** Random integer in range.  */
#define STG_RANDOM_INT_IN(min,max) ( (min == max) ? min : ( ((rand())%(int)rint(max-min)) + min ) )

/** @} */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MILLION 1e6
#define BILLION 1e9

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef TWOPI
#define TWOPI (2.0*M_PI)
#endif
  
#ifndef RTOD
  /// Convert radians to degrees
#define RTOD(r) ((r) * 180.0 / M_PI)
#endif
  
#ifndef DTOR
  /// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180.0)
#endif
  
#ifndef NORMALIZE
  /// Normalize angle to domain -pi, pi
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif


/** Display the given message in the GUI window, and also log it to the console.
 *  The maximum size for a message when using variable arguments is 2048.
 * @param timestamp When this message was generated, or 0 to generate a 
 *   timestamp in this function.
 * @param category Short string identifying the type or origin of the message
 * @param level Indicate the severity: 0=fatal (dialog box with offer to
 *   continue or exit may also be shown), 1=warning
 *   (might be important), 2=informative, 3=debugging (You should only use this
 *   in debug modes).
 * @param format Format string (just like printf)
 * @param ... Arguments (just like printf)
 */
void stg_world_display_message(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* format, ...);
void stg_world_display_message_s(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* s);
void stg_world_display_message_v(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* format, va_list args);

/** Use degrees instead of radians when displaying rotation values in the GUI */
void stg_world_display_degrees(stg_world_t *w);
/** Use radians instead of degrees when displaying rotation values in the GUI
 * (this is the default setting) */
void stg_world_display_radians(stg_world_t *w);

/** Display an error dialog box with options to continue or exit the program. */
void stg_gui_fatal_error_dialog(const char* primary_text, const char* secondary_text, int exit_code, gboolean continue_button);

/** Same as stg_gui_fatal_error_dialog with TRUE for continue button */
void stg_gui_error_dialog(const char* primary_text, const char* secondary_text, int exit_code);


/// If TRUE (the default value), then display an interactive dialog box for
/// critical error messages. If FALSE, then log the error and exit the
/// program.
extern gboolean stg_use_error_dialog;

/// If greater than 0, then periodically print out timing statistics etc.
/// (Same values as displayed in the status bar in the GUI).
/// Default is 0.
/// @sa stg_world_log_stats() to manually log stats
extern stg_msec_t stg_log_stats_freq;

extern FILE* stg_output_file;
FILE* stg_open_log_file(const char *filename, const char *mode);
void stg_set_log_file(FILE* file);
void stg_flush_log_file();
void stg_close_log_file();


typedef void(*stg_log_file_full_callback_t)(FILE* fp, size_t sz, size_t max);

/** If set, keep track of amount of data written using the print functions. When
 *  the limit is exceeded, call the callback if not NULL.
 */
void stg_set_log_file_max_size(size_t max, stg_log_file_full_callback_t callback);


/** @define STG_SINCOS(a, sina, cosa) 
    Set sina and cosa to the sin and cos of a. sina and cosa must be
    dereferencable variables. If the platform supports the sincos function,
    then both @a sina and @a cosa are set by one call to that function, passed
    in as pointers. Otherwise, the macro expands to two assignments of
    @a sina and @a cosa from calls to the sin() and cos() functions, with 
    @a a passed as the parameter to each call (so @a will be evaluated twice
    if it's an expression).
    @note With some versions of GCC, if you include math.h before stage.h, then
          the sincos() function will not be defined: to ensure that sincos()
	  is defined, either include math.h after stage.h, or define the
	  _GNU_SOURCE and __USE_GNU preprocessor symbols before including <math.h>.
*/
#ifdef HAVE_SINCOS
#define STG_SINCOS(a, sina, cosa) sincos((a), &(sina), &(cosa));
#else
#define STG_SINCOS(a, sina, cosa) {\
  sina = sin(a); \
  cosa = cos(a); \
}
#endif


// end documentation group libstage
/**@}*/



#ifdef __cplusplus
}
#endif 

#endif
