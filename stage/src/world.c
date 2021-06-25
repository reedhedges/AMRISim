
#define _GNU_SOURCE
#include <stdlib.h>
#include <assert.h>
#include <string.h> // for strdup(3)
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>

//#define DEBUG 1
//#define DEBUG_WORLD_UPDATE_TIME_STATS

#include "gui.h"
#include "stage_internal.h"


/** @define STG_TOKEN_MAX Maximum size for a model token (its unique name) */
#define STG_TOKEN_MAX (64)

#if 0
/** @define STG_PLATFORM_CLOCK_OVER
 *  Define to the approximate system clock error (ms), used as a kind of fudge
 *  factor in stg_world_update() when sleeping and measuring simulation loop timing
 *  to deal with system clock innacuracy, and misc. overhead that might be interfering
 *  with perfect loop timing.
 *  You can redefine this when compiling, otherwise a value is chosen
 *  based on the build platform (10ms for Linux, 100ms for Windows, 1ms for all others.)
 *  and the time to sleep is rounded to that value.  (On Linux, the clock is 
 *  updated every 10ms or better unless compiled with an alternate value; Windows' 
 *  behavior is currently unknown but empirically seems to be about 10 using
 *  Sleep().). 
 *  @todo use clock_getres() if available.
 */
#ifndef STG_PLATFORM_CLOCK_OVER
#  ifdef __linux__
#   warning Platform is Linux; Using clock overshoot/fudge factor of 2 ms.
#   define STG_PLATFORM_CLOCK_OVER (2) 
#  else
#   if defined(__win32__) || defined(__MINGW__)
#     warning Platform is Windows; Using clock overshoot/fudge factor of 2 ms.
#     define STG_PLATFORM_CLOCK_OVER (2)
#   else
#     warning Platform is unknown; Using clock overshoot/fudge factor of 0 ms.
#     define STG_PLATFORM_CLOCK_OVER (0)
#   endif // ifdef __win32__
#  endif // ifdef __linux__
#endif
#else
#define STG_PLATFORM_CLOCK_OVER 0
#endif

/** @define STG_WORLD_SLEEP_MAX
 *  Maximum sleep time within stg_world_update() if sleepflag argument is TRUE.
 *  Don't make it too large or
 *  stg_world_update() can end up sleeping too much.
 */
#ifndef STG_WORLD_SLEEP_MAX
#define STG_WORLD_SLEEP_MAX (20)
#endif

#ifdef STG_ENABLE_WORLD_LOCK
// Uncomment this to enable some lock debugging logging
//#define STG_DEBUG_WORLD_PENDING_LOCKS 1
#endif

#ifdef STG_DEBUG_WORLD_PENDING_LOCKS
pthread_mutex_t _stg_world_pending_lock_count_mutex;
unsigned int _stg_world_pending_lock_count;
#endif

// Define values for default constants declared in stage.h:
const double STG_DEFAULT_RESOLUTION = 0.02;  // 2cm pixels
const double STG_DEFAULT_WORLD_WIDTH = 20.0; // meters
const double STG_DEFAULT_WORLD_HEIGHT = 20.0; // meters
const double STG_DEFAULT_INTERVAL_REAL = 100.0; //ms
const double STG_DEFAULT_INTERVAL_SIM = 100.0;  // ms


// Private global data for stg_world:
static int init_occurred = 0;
static unsigned long _trylock_skip_update_count = 0; // for debugging

#include "stage_internal.h"

// Stage-private data kept in stage.c:
extern int _stg_quit; // quit flag is returned by stg_world_update()
extern int _stg_disable_gui;

// Private callback functions
void _world_model_update_cb( gpointer key, gpointer value, gpointer user );
void _world_model_update_locking_cb( gpointer key, gpointer value, gpointer user );
void _world_model_update_trylock_cb( gpointer key, gpointer value, gpointer user );

/** @addtogroup stage
    @{ */

/** @defgroup world Worlds

Stage simulates a 'world' composed of models, defined in a 'world
file'. 

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
world
(
   name            "[filename of worldfile]"
   interval_real   100
   interval_sim    100
   resolution      0.01
)
@endverbatim

@par Details
- name [string]
  - the name of the world, as displayed in the window title bar. Defaults to the worldfile file name.
- interval_sim [milliseconds]
  - the length of each simulation update cycle in milliseconds.
- interval_real [milliseconds]
  - the amount of real-world (wall-clock) time the siulator will attempt to spend on each simulation cycle.
- resolution [meters]
  - specifies the resolution of the underlying bitmap model. Larger values speed up raytracing at the expense of fidelity in collision detection and sensing. 

@par More examples

The Stage source distribution contains several example world files in
<tt>(stage src)/worlds</tt> along with the worldfile properties
described on the manual page for each model type.

*/

/**@}*/

extern stg_type_record_t typetable[]; // "built in" types (defined in typetable.c)

void stg_world_set_interval_real( stg_world_t* world, stg_msec_t val )
{
  world->real_interval = val;
}

void stg_world_set_interval_sim( stg_world_t* world, stg_msec_t val )
{
  world->sim_interval = val;
}

// create a world containing a passel of Stage models based on the
// worldfile
stg_world_t* stg_world_create_from_file( const char* worldfile_path, int echo, void (*loop_callback)() )
{
  STG_F()
  int section;
  const char* world_name;
  stg_msec_t interval_real, interval_sim;
  double ppm, width, height;
  stg_world_t* world;
  int section_count;

  if(! wf_load( (char*)worldfile_path, echo ) )
    return NULL;
  
  section = 0;
 
  world_name =
    wf_read_string( section, "name", (char*)worldfile_path );
  
  interval_real = 
    wf_read_int( section, "interval_real", STG_DEFAULT_INTERVAL_REAL );

  interval_sim = 
    wf_read_int( section, "interval_sim", STG_DEFAULT_INTERVAL_SIM );
      
  ppm = 
    1.0 / wf_read_float( section, "resolution", STG_DEFAULT_RESOLUTION ); 
  
  width = 
    wf_read_tuple_float( section, "size", 0, STG_DEFAULT_WORLD_WIDTH ); 

  height = 
    wf_read_tuple_float( section, "size", 1, STG_DEFAULT_WORLD_HEIGHT ); 

  _stg_disable_gui = wf_read_int( section, "gui_disable", _stg_disable_gui );

  // create a single world
  world = 
    stg_world_create( 0, 
		      world_name, 
		      interval_sim, 
		      interval_real,
		      ppm,
		      width,
		      height );

  if( world == NULL )
    return NULL; // failure
  
  stg_world_set_cursor_busy(world);
  

  section_count = wf_section_count();
  
  // Iterate through sections and create client-side models
  for( section = 1; section < section_count; section++ )
  {
    if(loop_callback) (*loop_callback)();
    if( strcmp( wf_get_section_type(section), "window") == 0 )
	{
	  // configure the GUI
	if( world->win )
    {
      gui_lock();
	    gui_load( world->win, section ); 
      gui_unlock();
    }
  }
  else
  {
    char *namestr;
	  int parent_section = wf_get_parent_section( section );
	  stg_model_t* mod = NULL;
	  stg_model_t* parent_mod = NULL ;

    // select model type based on the worldfile token
	  char *typestr = (char*)wf_get_section_type(section);      
	  
	  namestr = (char*)wf_read_string(section, "name", NULL );

	  PRINT_DEBUG2( "section %d parent section %d\n", 
			section, parent_section );
	  
	  parent_mod = stg_world_get_model( world, parent_section );
	  PRINT_DEBUG4( "creating new model %s, type %s, from section %d (parent section %d)",
			namestr, typestr, section, parent_section );
      mod = stg_world_new_model_ex(world, typestr, namestr, parent_mod, section, TRUE);
      if(!mod) return NULL;
      assert(mod);
	}
  }
  stg_world_set_cursor_normal(world);
  return world;
}

gboolean stg_world_load_world_file(stg_world_t *world, const char *file, int echo)
{
  STG_F()
  return wf_load(file, echo);
}


stg_model_t* stg_world_new_model_ex(stg_world_t* world, const char* typestr, const char* override_token, stg_model_t* parent_mod, stg_id_t world_id, gboolean parsing_worldfile)
{
  STG_F()
  stg_model_t* mod;
  int index;
  char tokenbuf[STG_TOKEN_MAX];  
  stg_type_record_t *rec = NULL;
  size_t tokindex = 0;
  char * base_typestr = NULL;
  char *token;

  // Maybe typestr is actually a model definition macro in the worldfile,
  // if so remember index in macrodef, and replace typestr with the base
  // type (which ought to be in the type table, looked up below)
  int macrodef = wf_lookup_macro_def(typestr);
  if(macrodef == -1)
  {
    //printf("\t\t\t\t\tUsing macrodef from typestr\n");
    base_typestr = typestr;
  }
  else
  {
    //printf("\t\t\t\t\tUsing macrodef #%d from wf_lookup_macro_def\n", macrodef);
    base_typestr = (char*) wf_get_macro_base_type(macrodef);
  }
  //printf("\t\t\t\t\tstg_world_new_model_ex: base_typestr of %s determined to be %s.\n", typestr, base_typestr);

 // lookup the key in the global type table
  rec = typetable; 
  index=0;
  //printf("\t\t\t\t\tstg_world_new_model_ex: attempting to use typetable.\n");
  while( rec->keyword )
  {
    if( strcmp( base_typestr, rec->keyword ) == 0 )
      break;
    rec++;
    index++;
  }

  // Try custom user types
  if(rec->keyword == NULL && stg_user_typetable)
  {
    printf("\t\t\t\t\tstg_world_new_model_ex: attempting to use stg_user_typetable.\n");

    rec = stg_user_typetable;
    while(rec && rec->keyword)
    {
      if(strcmp(base_typestr, rec->keyword) == 0)
        break;
      rec++;
      index++;
    }
  }

  // if we didn't find the keyword, throw an error
  if( rec->keyword == NULL ) // if we failed to find matching record
  {
    if( strcmp( typestr, "model" ) != 0) 		  
    {
      stg_print_error( "No robot or device model \"%s\" defined.", typestr );
      return NULL;
    }
  }

  // generate a token and count this type in its parent (or world,
  // if it's a top-level object)
  if( parent_mod == NULL )
  {
    tokindex = world->child_type_count[index]++;
    snprintf( tokenbuf, STG_TOKEN_MAX, "%s:%zd",
        typestr, tokindex);
  }
  else
  {
    tokindex = parent_mod->child_type_count[index]++;
    assert(parent_mod->token != NULL);
    snprintf( tokenbuf, STG_TOKEN_MAX, "%s.%s:%zd",
        parent_mod->token, typestr, tokindex);
  }

  //PRINT_WARN1( "generated name %s", namebuf );
  
  // having done all that, allow the user to specify a name instead  (but we
  // still need tokindex, that's useful information)
  if(override_token == NULL) token = tokenbuf;
  else token = override_token;
  

  //PRINT_WARN2( "loading model name %s for type %s", namebuf, typestr );
  
  // determine a worldfile identifier if not given
  // XXX TODO start at wf_section_count() to avoid having identifiers that
  // match existing worldfile sections. how neccesary is this? 
  if(world_id < 0) {
    // Check first if there is a base entity that has been initialized from this typestr.
    //   If there has, we should not be creating a new entity (template) for this model type.
    //   Previously, Stage was creating a new entity for each model request, and never removing them. This is patently stupid and should not be done.
    //printf("\t\t\t\t\twf_find_entity_id_for_model_type(%s): %d\n", typestr, wf_find_entity_id_for_model_type(typestr));
    world_id = wf_find_entity_id_for_model_type(typestr);

    if (world_id < 0)
    {
      //world_id = wf_create_entity(typestr, parent_mod?parent_mod->id:-1);   // mod->id replaced by mod->entity_id (to decouple robot models from their 1-to-1 relationship with CEntity entries)
      world_id = wf_create_entity(typestr, parent_mod?parent_mod->entity_id:-1);
      if(world_id == -1)
      {
        stg_print_error("Unable to add a new entity entry for a \"%s\" type model. to world data store (worldfile).  Error in model type definitions?", typestr);
        return NULL;
      }
      else
        wf_set_entity_id_for_model_type(typestr, world_id);
    }
  }

  //printf("\t\t\t\t\tworld_file section count: %d  \n", wf_section_count());

  //  for(world_id = wf_section_count(); stg_world_get_model(world, world_id) != NULL; world_id++);

  // create the model
  mod = stg_model_create( world, parent_mod, world_id, token, base_typestr, typestr, tokindex, rec->initializer, FALSE);
  assert( mod );

  // add the new model to the world. do this before children, since they need to
  // look at already added models to determine an ID.
  stg_world_add_model( world, mod );

  // if it uses macros, typestr is the macro name, and we need to manually create an "entity"
  // in the world file data store, at which time it will be populated with properties from the macros.
  // If the type is a macro, then we also need to load properties from the macro
  // and also go through and create child  models defined in the macros.
  // However if this function was called while parsing the world file, this is
  // all unneccesary, since it will have already created entities and handled
  // children.
  if(!parsing_worldfile && macrodef != -1)
  {
    int child_world_id = 0;
    //printf("Creating a macro-based model (%s)... adding new worldfile entity for it...\n", typestr);

    // Did this above:
    //int ent = wf_create_entity(typestr, parent_mod?parent_mod->id:-1);
    //printf("Created a new worldfile entity for \"%s\", has id %d.\n", typestr, ent);
    /*
    if(ent == -1)
    { 
      stg_print_error("Unable to add a new entity entry to world data store (worldfile).  Error in model type definition?");
      return NULL;
    }
    */
    
    //printf("loading macro properties for model \"%s\", which now has worldfile id %d.\n", mod->token, world_id);
    stg_model_load_from_worldfile_section_id(mod, world_id);

    // Now recursively create child models defined by the macro:
    while( (child_world_id = wf_get_next_child_entity(world_id, child_world_id)) != -1 )
    {
      //printf("recurse down");
      const char *type = wf_get_section_immediate_type(child_world_id);
      PRINT_DEBUG4("stg_world_new_model: creating new model for child of this model \"%s\" with type=\"%s\", section_type=\"%s\" (section id=%d)", token, type, wf_get_section_type(child_world_id), child_world_id);
      stg_model_t *child_mod = stg_world_new_model_ex(world, type, NULL, mod, child_world_id, FALSE);
      assert(child_mod);
      child_world_id++; // start at next potential child
      //printf("recurse up");
    }
  }
  else
  {
    stg_model_load( mod );
  }

  return mod;
}

stg_model_t* stg_world_new_model(stg_world_t* world, const char* type, stg_model_t* parent, const char *requestedName)
{
  STG_F()
  // if requestedName already exists and if so, append a number
  if(requestedName)
  {
    char buf[256];
    unsigned long i = 1;
    char *name = requestedName;
    while(stg_world_model_name_lookup(world, name) != NULL)
    {
      snprintf(buf, 256, "%s-%lu", requestedName, ++i);
      assert(i < ULONG_MAX);
      name = buf;
    }
    requestedName = name;
//    printf("XXXXXXX decided on %s for new model name based on requestedName\n", requestedName);
  }

  stg_model_t* return_model = stg_world_new_model_ex(world, type, requestedName, parent, -1, FALSE);  // TODO: DON'T PASS A -1 FOR THE world_id IF WE'VE ALREADY CREATED A CEntity FOR THIS type!!!!!

  return return_model;
}
    

stg_world_t* stg_world_create( stg_id_t id, 
			       const char* token, 
			       int sim_interval, 
			       int real_interval,
			       double ppm,
			       double width,
			       double height )
{
  STG_F()
  stg_world_t* world;

  if( ! init_occurred )
    {
      //puts( "STG_INIT" );
      stg_init( 0, NULL );
      init_occurred = 1;
    }

  world = calloc( sizeof(stg_world_t),1 );
  
  world->id = id;
  world->token = strdup( token );
  world->models = g_hash_table_new( g_int_hash, g_int_equal);
  world->models_by_name = g_hash_table_new( g_str_hash, g_str_equal);
  world->sim_time = 0.0;
  world->sim_interval = sim_interval;
  world->real_interval = real_interval;
  world->real_last_update = 0;
  world->avg_interval_ratio = 1.0;
  world->real_interval_measured = real_interval;
  world->real_interval_too_long_warnings = 0;
  world->zero_interval_warnings = 0;
  
  world->width = width;
  world->height = height;
  world->ppm = ppm; // this is the finest resolution of the matrix
  world->matrix = stg_matrix_create( ppm, width, height ); 
  
  
  world->paused = TRUE; // start paused.
  
  world->destroy = FALSE;
  
  if( _stg_disable_gui )
    world->win = NULL;
  else    
    world->win = gui_world_create( world );
  
  world->subs = 0;
  world->model_property_toggles = NULL;

  world->file_loaders = g_hash_table_new( g_str_hash, g_str_equal);

  world->model_update_cb = _world_model_update_cb;
  world->model_locking_policy = STG_WORLD_NO_LOCKING;
  world->world_locking_enabled = FALSE;

  world->use_model_height = TRUE;
  world->quiet = FALSE;
  world->gui_enabled = TRUE;
  world->display_degrees = FALSE;
  //world->display_mm = FALSE;

  world->last_stats_log_time = stg_timenow();
  world->update_miss_count = 0;

  return world;
}


void _stg_world_map_model_hashtable_cb(gpointer key, gpointer value, gpointer userdata)
{
  stg_model_t* model = (stg_model_t*)value;
  gboolean flag = (userdata != 0);
  stg_model_map(model, flag);
}

void world_unmap_all(stg_world_t *world)
{
  STG_F()
  g_hash_table_foreach(world->models, &_stg_world_map_model_hashtable_cb, (gpointer)0);
}

void world_map_all(stg_world_t *world)
{
  STG_F()
  g_hash_table_foreach(world->models, &_stg_world_map_model_hashtable_cb, (gpointer)1);
}

void stg_world_resize(stg_world_t* world, stg_meters_t new_width, stg_meters_t new_height)
{
  STG_F()
  double oldppm;

  PRINT_DEBUG2("resizing to %0.2f X %0.2f\n", new_width, new_height);

  stg_world_set_cursor_busy(world);

  if(world->win)
  {
    gui_lock();
    gui_window_draw_world_extent(world->win, new_width, new_height);
    gui_unlock();
  }
  world->width = new_width;
  world->height = new_height;

  // Remove (unmap)  all models from the old matrix
  world_unmap_all(world);

  // Destroy the old matrix, create a new one at the right size, and repopulate
  stg_matrix_lock(world->matrix);
  oldppm = world->matrix->ppm;
  stg_matrix_destroy(world->matrix);
  // destroyed matrix,  don't need to unlock the mutex (except that any others
  // waiting will now have an invalid mutex and crash!)
  // XXX: TODO need to find a way to resize without destroying the matrix.
  world->matrix = stg_matrix_create(oldppm, new_width, new_height);
  world_map_all(world);

  stg_world_set_cursor_normal(world);
}

void stg_world_change_resolution(stg_world_t* world, double res)
{
  STG_F()
  stg_world_set_cursor_busy(world);


  // Remove (unmap)  all models from the old matrix
  world_unmap_all(world);

  // Destroy the old matrix, create a new one at the right size, and repopulate
  stg_matrix_lock(world->matrix);
  stg_matrix_destroy(world->matrix);
  // destroyed matrix,  don't need to unlock the mutex (except that any others
  // waiting may now have an invalid mutex and crash!)
  // XXX: TODO need to find a way to resize without destroying the matrix.
  // Maybe just lock the world instead of the matrix?
  world->ppm = 1.0/res;
  world->matrix = stg_matrix_create(world->ppm, world->width, world->height);
  world_map_all(world);

  stg_world_set_cursor_normal(world);
} 


void _stg_world_reset_model_pose_cb(gpointer key, gpointer value, gpointer userdata)
{
  STG_F()
    stg_model_t *mod = (stg_model_t*)value;
    if(userdata == NULL || strcmp((const char*)userdata, mod->base_type_name) == 0)
      stg_model_reset_pose(mod);
}

void stg_world_reset_model_type_poses(stg_world_t *world, const char *type)
{
    g_hash_table_foreach(world->models, _stg_world_reset_model_pose_cb, (gpointer)type);
}


void stg_world_reset_all_model_poses(stg_world_t *world)
{
  stg_world_reset_model_type_poses(world, NULL);
}

void stg_world_reset_position_model_poses(stg_world_t *world)
{
  stg_world_reset_model_type_poses(world, "position");
}

void stg_world_stop( stg_world_t* world )
{
  world->paused = TRUE;
}

void stg_world_start( stg_world_t* world )
{
  world->paused = FALSE;
}

stg_msec_t stg_world_get_time(stg_world_t* world)
{
    return world->sim_time;
}

struct _stg_world_dimensions_t {
  stg_bounds_t x;
  stg_bounds_t y;
};

// hash table callback to check if a model is outside the world dimensions
// currently being calculated, and expand the bounds if neccesary.
void _stg_world_expand_dimensions_cb(gpointer key, gpointer value, gpointer user_data)
{
  STG_F()
  stg_model_t *model = (stg_model_t*)value;
  //printf(" * adding model id=%d token=%s to world dimensions\n", *((stg_id_t*)key), model->token);
  struct _stg_world_dimensions_t *dim = (struct _stg_world_dimensions_t *)user_data;
  stg_pose_t *pose = stg_model_get_property_fixed(model, "pose", sizeof(stg_pose_t));
  stg_geom_t *geom = stg_model_get_property_fixed(model, "geom", sizeof(stg_geom_t));
  stg_bounds_t mod_x, mod_y;
  assert(pose);
  assert(model);
  assert(dim);
  assert(geom);
  //printf("  -> current dims are min=%0.2f,%0.2f max=%0.2f,%0.2f\n", dim->x.min, dim->y.min, dim->x.max, dim->y.max);

  //printf("  -> this model's position is %0.2f, %0.2f, size is %0.2f, %0.2f\n", geom->pose.x, geom->pose.y, geom->size.x, geom->size.y);
  mod_x.max = pose->x + geom->pose.x + ((geom->size.x) /*/ 2.0*/);
  mod_x.min = pose->x + geom->pose.x - ((geom->size.x) /*/ 2.0*/);
  mod_y.max = pose->y + geom->pose.y + ((geom->size.y) /*/ 2.0*/);
  mod_y.min = pose->y + geom->pose.y - ((geom->size.y) /*/ 2.0*/);
  //printf("  -> this model's values for comparison are min=%0.2f,%0.2f max=%0.2f,%0.2f\n", mod_x.min, mod_y.min, mod_x.max, mod_y.max);

  if( (mod_x.max) > dim->x.max) dim->x.max = mod_x.max;
  if( (mod_x.min) < dim->x.min) dim->x.min = mod_x.min;
  if( (mod_y.max) > dim->y.max) dim->y.max = mod_y.max;
  if( (mod_y.min) < dim->y.min) dim->y.min = mod_y.min;
  //printf("  -> new dims are min=%0.2f,%0.2f max=%0.2f,%0.2f\n", dim->x.min, dim->y.min, dim->x.max, dim->y.max);
}

// calculate the bounding rectangle of everything in the world
void stg_world_dimensions( stg_world_t* world, 
			   double* min_x, double * min_y,
			   double* max_x, double * max_y )
{
  STG_F()
  struct _stg_world_dimensions_t dim;
  dim.x.max = dim.y.max = -MILLION;
  dim.x.min = dim.y.min = MILLION;
  assert(world);
  assert(world->models);
  g_hash_table_foreach(world->models, &_stg_world_expand_dimensions_cb, (gpointer) &dim);
  if(min_x) *min_x = dim.x.min;
  if(min_y) *min_y = dim.y.min;
  if(max_x) *max_x = dim.x.max;
  if(max_y) *max_y = dim.y.max;
}

// resize the world to fit all models in a square big enough to fit all models.
void stg_world_resize_to_contents(stg_world_t* world, stg_meters_t border_padding)
{
  STG_F()
  double min_x, min_y, max_x, max_y, size_x, size_y, offset_x, offset_y, extent_x, extent_y, extent_max;
  assert(world);
  stg_world_dimensions(world, &min_x, &min_y, &max_x, &max_y);
  size_x = max_x - min_x;
  size_y = max_y - min_y;
  offset_x = (size_x / 2.0) + min_x;
  offset_y = (size_y / 2.0) + min_y;
  extent_x = (fabs(offset_x) + (size_x / 2.0)) * 2.0 + border_padding;
  extent_y = (fabs(offset_y) + (size_y / 2.0)) * 2.0 + border_padding;
  extent_max = fmax(extent_x, extent_y); 
  stg_world_resize(world, extent_max, extent_max);
}


void _stg_world_destroy_model_hashtable_cb(gpointer key, gpointer value, gpointer userdata)
{
  STG_F()
  stg_model_t* mod = (stg_model_t*)value;
  if(mod)
    stg_model_destroy(mod);
}

void stg_world_destroy( stg_world_t* world )
{
  STG_F()
  assert( world );
		   
  PRINT_DEBUG1( "destroying world %d", world->id );
  
  g_hash_table_foreach(world->models, &_stg_world_destroy_model_hashtable_cb, NULL);
  g_hash_table_destroy( world->models );
  
  stg_matrix_destroy( world->matrix );


  gui_world_destroy( world );

  free( world->token );
  free( world );
}

void world_destroy_cb( gpointer world )
{
  STG_F()
  stg_world_destroy( (stg_world_t*)world );
}


/** @warning Do not store any pointers to model property internal data beyond
 * one update, they may become invalid if the data has to be restored and
 * reallocated due to changes during the update. Use stg_model_get_property 
 * to get a new pointer to the data each time you need to access it.
 */
int stg_world_update( stg_world_t* world, int sleepflag, int skiptoosoon )
{
  STG_F()
  stg_msec_t now, elapsed, left;

  //PRINT_WARN( "World update" );
  //puts("update");

  stg_world_lock(world);


  if( world->win && world->gui_enabled) 
  { 
    gui_poll();       
  } 

  now = stg_timenow();

   
  elapsed =  now - world->real_last_update;

#ifdef DEBUG_WORLD_UPDATE_TIME_STATS
  PRINT_DEBUG4(
  //fprintf(stderr,
	"now=%lu,\tlast update=%lu,\telapsed=%lu\tworld simtime=%lu\n", 
  	now, world->real_last_update, elapsed, world->sim_time  
  );
  //fflush(stderr);
#endif

  if(elapsed > (world->real_interval + (stg_msec_t)(world->real_interval/10.0)))
  {
    ++ (world->real_interval_too_long_warnings);
    if(elapsed > (world->real_interval * 3))
      stg_print_warning("(debug) last interval was more than 10%% too long! (%lu vs. %lu)", elapsed, world->real_interval);
  }
  else if(elapsed < 5)
    ++ (world->zero_interval_warnings);


  // If we only have a few ms to wait before the next update, sleep for that time. Otherwise, sleep
  // for a fixed time and return (to allow the gui and the external program to do stuff).
  if(sleepflag)
  {
    left = world->real_interval - elapsed + STG_PLATFORM_CLOCK_OVER;

    /* Another approach to using CLOCK_RES:
	    // Truncate down to the nearest 10ms, since by default Linux's clock only has a resolution of
	    // 10ms, err on the side of slightly too fast.

	    //left = (stg_msec_t)trunc((double)left/STG_PLATFORM_CLOCK_OVER) * STG_PLATFORM_CLOCK_OVER;

	    // or, try rounding instead of truncating: 
	    //left = (stg_msec_t)rint((double)left/STG_PLATFORM_CLOCK_OVER) * STG_PLATFORM_CLOCK_OVER;
    */

    //printf("left=%lu\n", left); fflush(stdout);

    // Sleep for the time remaining then do the update, or for MAX and return
    // if the time left is big.
    // (If elapsed is greater than real_interval, don't sleep (left will have
    // rolled over below zero anyway and it would have slept for days.))
    if( (elapsed <= world->real_interval + STG_PLATFORM_CLOCK_OVER) && (left <= STG_WORLD_SLEEP_MAX))
    {
	//printf("sleep(left=%lu)\n", left); fflush(stdout);
      //one method, doesn't always work so well: usleep( (left > STG_PLATFORM_CLOCK_OVER)?(left - STG_PLATFORM_CLOCK_OVER):0 * 1000 );
      if(left > 0) 
      { 
        stg_world_unlock(world);
        usleep( (left) * 1000);
        stg_world_lock(world);
      }
    }
    else if(elapsed < world->real_interval + STG_PLATFORM_CLOCK_OVER)
    {
      // Sleep for MAX and then don't do the update.
	//printf("sleep(max=%lu)\n", STG_WORLD_SLEEP_MAX); fflush(stdout);
      stg_world_unlock(world);
      usleep( (STG_WORLD_SLEEP_MAX + STG_PLATFORM_CLOCK_OVER ) * 1000 );
      return _stg_quit;
    }
  }
  else if(skiptoosoon && elapsed < world->real_interval)
  {
    ++(world->update_miss_count);
    stg_world_unlock(world);
    return _stg_quit;
  }



  // time for an update?
  //already checked above before sleepingif(elapsed >= world->real_interval)
  {

    if( ! world->paused ) // only actually do the update if we're not paused
    {
      //stg_print_msg("\nBeginning world update...");

      world->real_interval_measured = elapsed; 
      g_hash_table_foreach( world->models, world->model_update_cb, world );	  	  
      world->real_last_update = now;
      world->sim_time += world->sim_interval;

      // for calculating the average later to debug:
      //fprintf(stderr, "%g\n", (double)world->sim_interval / (double)world->real_interval_measured);
    }

    // update the gui
    if( world->win && world->gui_enabled)
    {
      if( gui_world_update( world ) != 0 )
        stg_quit_request();      
    }

/*
    printf( " time:%lu simint:%lu realint:%lu  ratio:%.2f\n",
      world->sim_time,
      world->sim_interval,
      world->real_interval,
      (double)world->sim_interval / (double)world->real_interval_measured  );
    fflush(stdout);
*/


  }

  if(stg_log_stats_freq > 0)
  {
    stg_msec_t now = stg_timenow();
    if(now - world->last_stats_log_time >= stg_log_stats_freq)
    {
      stg_world_log_stats(world);
      world->last_stats_log_time = now;
    }
  }

  stg_world_unlock(world);

  return _stg_quit; // note may have been set by the GUI or someone else during this update
}

stg_msec_t stg_log_stats_freq = 0;

void stg_world_log_stats(stg_world_t *world)
{
//  stg_print_msg("At %lu time sim: %lu, real: %lu, ratio: %2.3f, update_miss_count: %lu",
//    (unsigned long)now,
//    (unsigned long)world->sim_interval, (unsigned long)world->real_interval_measured,
//    (double)world->sim_interval / (double)world->real_interval_measured,
//    world->update_miss_count);

  stg_print_msg("Desired simulated time interval=%lu, desired real time interval=%lu, last real interval measured=%lu, current ratio=%2.3f, average ratio=%2.3f, total real time elapsed=%lu.",
    (unsigned long) stg_world_get_sim_interval(world),
    (unsigned long) stg_world_get_real_interval(world),
    stg_world_get_last_interval(world),
    (double)stg_world_get_sim_interval(world) / (double)stg_world_get_last_interval(world),
    stg_world_get_avg_interval_ratio(world),
    (unsigned long) stg_timenow()
  );
  stg_print_msg("Update misses (update called between scheduled times): %lu", world->update_miss_count);
}

int stg_world_run(stg_world_t *world)
{
   int status = 0;
   while(status == 0)
      status = stg_world_update(world, TRUE, FALSE);
   return status;
}


stg_model_t* stg_world_get_model( stg_world_t* world, stg_id_t mid )
{
  STG_F()
  return( world ? g_hash_table_lookup( (gpointer)world->models, &mid ) : NULL );
}

void stg_world_add_model( stg_world_t* world, 
			  stg_model_t* mod  )
{
  STG_F()
  assert(world);
  assert(world->models);
  assert(world->models_by_name);

#if 1
  srand(time(NULL));
  stg_id_t rand_key_num = (stg_id_t) rand();
  while(stg_world_get_model( world, rand_key_num ))
    rand_key_num = (stg_id_t) rand();
  mod->unique_id = rand_key_num;
#else
  // This exits for debugging. Checking if a random num was already used in the hash was difficult, because the keys are already hashed, and don't look the same as the ids
  GHashTableIter iter;
  gpointer key, value;

  g_hash_table_iter_init (&iter, world->models);
  while (g_hash_table_iter_next (&iter, &key, &value))
  {
    stg_id_t hash_key = (stg_id_t) GPOINTER_TO_INT(key);
    printf("stg_world_add_model(): hash_key: %d\n", hash_key);
  }

  stg_id_t seq_id_num = 2;
  while(stg_world_get_model( world, seq_id_num ))
  {
    printf("already using id: %d\n", seq_id_num);
    seq_id_num++;
  }
  printf("stg_world_add_model(): seq_id_num: %d\n", seq_id_num);
  mod->unique_id = seq_id_num;
#endif
  //printf("stg_world_add_model(): mod->unique_id: %d\n", mod->unique_id);

  //g_hash_table_insert( world->models, &mod->id, mod );  // mod->id replaced by mod->unique_id (to decouple robot models from their 1-to-1 relationship with CEntity entries)
  g_hash_table_insert( world->models, &mod->unique_id, mod );  // mod->id replaced by mod->unique_id (to decouple robot models from their 1-to-1 relationship with CEntity entries)
  g_hash_table_insert( world->models_by_name, mod->token, mod );

  //printf("stg_world_add_model(): added model with unique_id (%d), world->models.size() = %u\n", mod->unique_id, g_hash_table_size (world->models));
}

void stg_world_remove_model(stg_world_t* world, stg_model_t* mod)
{
  STG_F()
  stg_model_map(mod, 0); // unmap
  //g_hash_table_remove(world->models, &(model->id)); // mod->id replace by mod->unique_id (to decouple robot models from their 1-to-1 relationship with CEntity entries)
  g_hash_table_remove(world->models, &(mod->unique_id));
  g_hash_table_remove(world->models_by_name, mod->token);

  //printf("stg_world_remove_model(): removed model with unique_id (%d), world->models.size() = %u\n", mod->unique_id, g_hash_table_size (world->models));
}


struct cb_package
{
  const char* propname;
  stg_property_callback_t callback;
  void* userdata;
};

void add_callback_wrapper( gpointer key, gpointer value, gpointer user )
{
  struct cb_package *pkg = (struct cb_package*)user;  
  stg_model_t* mod = (stg_model_t*)value;
  
  size_t dummy;
  if( stg_model_get_property( mod, pkg->propname, &dummy ) )
    stg_model_add_property_callback( mod,
				     pkg->propname,
				     pkg->callback,
				     pkg->userdata );
}			   

void remove_callback_wrapper( gpointer key, gpointer value, gpointer user )
{
  struct cb_package *pkg = (struct cb_package*)user;  
  stg_model_t* mod = (stg_model_t*)value;
  
  size_t dummy;
  if( stg_model_get_property( mod, pkg->propname, &dummy ) )
    stg_model_remove_property_callback( (stg_model_t*)value,
					pkg->propname,
					pkg->callback );
}			   
  
void stg_world_add_property_callback( stg_world_t* world,
				      char* propname,
				      stg_property_callback_t callback,
				      void* userdata )     
{  
  STG_F()
  struct cb_package pkg;

  assert( world );
  assert( propname );
  assert( callback );

  pkg.propname = propname;
  pkg.callback = callback;
  pkg.userdata = userdata;

  g_hash_table_foreach( world->models, add_callback_wrapper, &pkg );
}


void stg_world_remove_property_callback( stg_world_t* world,
					 char* propname,
					 stg_property_callback_t callback )
{  
  STG_F()
  struct cb_package pkg;

  assert( world );
  assert( propname );
  assert( callback );
  
  pkg.propname = propname;
  pkg.callback = callback;
  pkg.userdata = NULL;

  g_hash_table_foreach( world->models, remove_callback_wrapper, &pkg );
}


void stg_world_print( stg_world_t* world )
{
  printf( " world %d:%s (%d models)\n", 
	  world->id, 
	  world->token,
	  g_hash_table_size( world->models ) );
  
   g_hash_table_foreach( world->models, model_print_cb, NULL );
}

void world_print_cb( gpointer key, gpointer value, gpointer user )
{
  stg_world_print( (stg_world_t*)value );
}

stg_model_t* stg_world_model_name_lookup( stg_world_t* world, const char* name )
{
  STG_F()
  return (stg_model_t*)g_hash_table_lookup( world->models_by_name, name );
}


void stg_model_save_cb( gpointer key, gpointer data, gpointer user )
{
  stg_model_save( (stg_model_t*)data );
}

void stg_model_reload_cb( gpointer key, gpointer data, gpointer user )
{
  stg_model_load( (stg_model_t*)data );
}

void stg_world_save( stg_world_t* world )
{
  // ask every model to save itself
  g_hash_table_foreach( world->models, stg_model_save_cb, NULL );
  
  if( world->win )
  {
    gui_lock();
    gui_save( world->win );
    gui_unlock();
  }
  
  wf_save();
}

// reload the current worldfile
void stg_world_reload( stg_world_t* world )
{
  // can't reload the file yet - need to hack on the worldfile class. 
  //wf_load( NULL, 0 ); 

  // ask every model to save itself
  g_hash_table_foreach( world->models, stg_model_reload_cb, NULL );
}


/* void stg_world_add_property_toggles( stg_world_t* world,  */
/* 				     const char* propname,  */
/* 				     stg_property_callback_t callback_on, */
/* 				     void* arg_on, */
/* 				     stg_property_callback_t callback_off, */
/* 				     void* arg_off, */
/* 				     const char* label, */
/* 				     int enabled ) */
/* { */
/*   stg_world_property_callback_args_t* args =  */
/*     calloc(sizeof(stg_world_property_callback_args_t),1); */
  
/*   args->world = world; */
/*   strncpy(args->propname, propname, STG_PROPNAME_MAX ); */
/*   args->callback_on = callback_on; */
/*   args->callback_off = callback_off; */
/*   args->arg_on = arg_on; */
/*   args->arg_off = arg_off; */

/*   gui_add_view_item( propname, label, NULL,  */
/* 		     toggle_property_callback, enabled, args ); */
/* } */


typedef struct _usercallback_struct {
  void (*func) (stg_model_t*, char*, void*);
  void* data;
} _usercallback;

void _foreach_model_by_name_ghashtable_callback(gpointer key, gpointer val, gpointer user_data)
{
  STG_F()
  _usercallback* ucb = (_usercallback*)user_data;
  stg_model_t* model = (stg_model_t*)val;
  if(model->parent == NULL)
    ucb->func(model, (char*)key, ucb->data);
}


void stg_world_foreach_model_by_name(stg_world_t* world, void(*func)(stg_model_t*, char*, void*), void* user_data)
{
  STG_F()
  _usercallback ucb;
  ucb.func = func;
  ucb.data = user_data;
  g_hash_table_foreach(world->models_by_name, _foreach_model_by_name_ghashtable_callback,  &ucb);
}

void stg_world_set_window_title(stg_world_t* world, const char* str)
{
  STG_F()
  gui_lock();
  gui_window_set_title(world->win, str);
  gui_unlock();
}

void stg_world_window_request_maximize(stg_world_t* world)
{
  gui_lock();
  gui_window_request_maximize(world->win);
  gui_unlock();
}

void stg_world_window_request_minimize(stg_world_t* world)
{
  gui_lock();
  gui_window_request_minimize(world->win);
  gui_unlock();
}

void stg_world_window_request_fullscreen(stg_world_t* world)
{
  gui_lock();
  gui_window_request_fullscreen(world->win);
  gui_unlock();
}

void stg_world_window_get_geometry(stg_world_t* world, int *x, int *y, int *width, int *height)
{
  assert(x);
  assert(y);
  assert(width);
  assert(height);
  gui_lock();
  gui_window_get_geometry(world->win, x, y, width, height);
}

void stg_world_pause(stg_world_t* world, int p)
{
  world->paused = p;
}

void stg_world_set_cursor_busy(stg_world_t* world)
{
  STG_F()
  gui_lock();
  gui_window_set_cursor_busy(world->win);
  gui_unlock();
}

void stg_world_set_cursor_normal(stg_world_t* world)
{
  STG_F()
  gui_lock();
  gui_window_set_cursor_normal(world->win);
  gui_unlock();
}

void stg_world_display_message_v(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* fmt, va_list args)
{
  STG_F()
    char buffer[2048];
    memset(buffer, 0, 2048);
    vsnprintf(buffer, 2048, fmt, args);
    stg_world_display_message_s(w, timestamp, category, level, buffer);
}

gboolean stg_use_error_dialog = TRUE;


void stg_world_display_message_s(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* s)
{
  STG_F()
  if(s == NULL || strlen(s) == 0)
    return;

  // TODO: do something with the timestamp.
  switch(level)
  {
    case STG_MSG_CRITICAL:
      // Fatal error
      if(category && strlen(category) > 0)
        stg_print_error("%s: %s", category, s);
      else
        stg_print_error("%s", s);
      if(stg_use_error_dialog)
      {
        // error dialog, then exit code -1 or let them say continue (last arg turns that on)
        gui_lock();
        stg_rtk_fatal_error_dialog(category, s, w->win->canvas->frame, -1, 1);
        gui_unlock();
      }
      else
        // just print the error, then exit code -1.
        stg_quit_request_code(-1);
        break;
    case STG_MSG_WARNING:
      // Warning
      gui_lock();
      if(category && strlen(category) > 0)
      {
        stg_print_warning("%s: %s", category, s);
        stg_rtk_canvas_write_message(w->win->canvas, category, "bold");
        stg_rtk_canvas_write_message(w->win->canvas, ": ", "bold");
      }
      stg_print_warning("%s", s);
      stg_rtk_canvas_write_message(w->win->canvas, "Warning: ", "bold red");
      stg_rtk_canvas_write_message(w->win->canvas, s, "red");
      stg_rtk_canvas_write_message(w->win->canvas, "\n", NULL);
      gui_unlock();
      break;
    case STG_MSG_INFORMATION:
      // Information
      gui_lock();
      if(category && strlen(category) > 0)
      {
        stg_print_msg("%s: %s", category, s);
        stg_rtk_canvas_write_message(w->win->canvas, category, "bold");
        stg_rtk_canvas_write_message(w->win->canvas, ": ", "bold");
      }
      else
        stg_print_msg("%s", s);
      stg_rtk_canvas_write_message(w->win->canvas, s, NULL);
      stg_rtk_canvas_write_message(w->win->canvas, "\n", NULL);
      gui_unlock();
      break;
  }
}

void stg_world_display_message(stg_world_t* w, time_t timestamp, const char* category, stg_message_level_t level, const char* fmt, ...)
{
  STG_F()
    va_list args;
    va_start(args, fmt);
    stg_world_display_message_v(w, timestamp, category, level, fmt, args);
    va_end(args);
}


static void _model_render_polygons_cb( gpointer key, gpointer data, gpointer user )
{
  STG_F()
  // Call any callbacks associated with the polygons property.
  stg_model_property_changed( (stg_model_t*)data, "polygons" );
}



void stg_world_set_fill_polygons(stg_world_t* world, gboolean fill) 
{
  if(!world->win) return;	// can happen if called during world creation
  world->win->fill_polygons = fill;
  g_hash_table_foreach(world->models, _model_render_polygons_cb, NULL);
}

void stg_world_hide_all_graphics(stg_world_t* world)
{
  if(!world->win) return;	// can happen if called during world creation
  world->win->show_geom = FALSE;
  world->win->show_polygons = FALSE;
  world->win->show_grid = FALSE;
  gui_lock();
  gui_sync_menu_toggle_items(world->win);
  gui_unlock();
}

void stg_world_disable_gui(stg_world_t *world)
{
  world->gui_enabled = FALSE;
  stg_world_hide_all_graphics(world);
}

gboolean stg_world_gui_enabled(stg_world_t *world)
{
  return world->gui_enabled;
}

void stg_world_set_mouse_buttons(stg_world_t* world, int panbutton, int zoombutton)
{
  world->win->canvas->mouse_button_pan = panbutton;
  world->win->canvas->mouse_button_zoom = zoombutton;
}

struct _stg_world_file_loader_entry
{
  const char* pattern;
  stg_world_file_loader_callback_t callback;
  void *userdata;
  const char* name;
};

void stg_world_add_file_loader(stg_world_t* world, stg_world_file_loader_callback_t callback, const char* pattern, const char* name, void* userdata)
{
  /* Create new entry for list */
  struct _stg_world_file_loader_entry *entry = (struct _stg_world_file_loader_entry *)malloc( sizeof(struct _stg_world_file_loader_entry) );
  assert(entry);
  entry->pattern = pattern;
  entry->callback = callback;
  entry->userdata = userdata;
  entry->name = name;

  /* Add to list */
  g_hash_table_insert(world->file_loaders, entry->pattern, entry);
  assert(world->file_loaders);

  /* Enable the menu item if */
  gui_lock();
  gui_enable_load_file_menu_item(world->win, TRUE);
  gui_unlock();

}


void _stg_world_add_file_loader_filter_to_dialog(gpointer key, gpointer item, gpointer ud)
{
#if GTK_CHECK_VERSION(2, 4, 0)
  struct _stg_world_file_loader_entry* entry = (struct _stg_world_file_loader_entry*)item;
  GtkFileChooser* dlg = (GtkFileChooser*)ud;
  GtkFileFilter* filter = gtk_file_filter_new();
  gtk_file_filter_set_name(filter, entry->name);
  gtk_file_filter_add_pattern(filter, entry->pattern);
  gtk_file_chooser_add_filter(dlg, filter);
#else
  fprintf(stderr, "Internal warning: file type filter in file selection dialog not implemented for GTK < 2.4 yet.\n");
#endif
}

/* Return path to home directory if Linux, or My Documents directory if Windows. */
#ifdef WIN32
static char mydocs[512];
#endif

const char* user_docs_dir()
{
#ifdef WIN32
  snprintf(mydocs, 512, "%s\\My Documents", getenv("HOMEPATH"));
  return mydocs;
#else
  return getenv("HOME");
#endif
}

void stg_world_load_file_act(stg_world_t* world) 
{
#if GTK_CHECK_VERSION(2, 4, 0)
  GtkFileChooser* dlg = (GtkFileChooser*)gtk_file_chooser_dialog_new("Load File...", NULL, GTK_FILE_CHOOSER_ACTION_OPEN, "Cancel", GTK_RESPONSE_CANCEL, "Load", GTK_RESPONSE_ACCEPT, NULL);
  gtk_file_chooser_set_current_folder(dlg, user_docs_dir());
  g_hash_table_foreach(world->file_loaders, &_stg_world_add_file_loader_filter_to_dialog, dlg);
#else
  char prev_dir[512];
  GtkFileSelection* dlg;
  getcwd(prev_dir, 512);
  chdir(user_docs_dir());
  dlg = (GtkFileSelection*)gtk_file_selection_new("Load File...");
  gtk_button_set_label(GTK_BUTTON(dlg->ok_button), "Load");
  gtk_button_set_label(GTK_BUTTON(dlg->cancel_button), "Cancel");
#endif

  gtk_dialog_set_default_response(GTK_DIALOG(dlg), GTK_RESPONSE_ACCEPT);
  while(1)
  {
    // TODO instead of using gtk_dialog_run (which blocks), instead install a
    // signal handler for "response" and check it from the main stg_world_update
    // loop.
    int r = gtk_dialog_run(GTK_DIALOG(dlg));
    if(r == GTK_RESPONSE_ACCEPT || r == GTK_RESPONSE_OK) 
    {
      char* file;
      char* dot;
      char pat[32];
      struct _stg_world_file_loader_entry *entry;
      stg_world_set_cursor_busy(world);
#if GTK_CHECK_VERSION(2, 4, 0)
      file = (char*) gtk_file_chooser_get_filename(dlg);
#else
      file = (char*) gtk_file_selection_get_filename(dlg);
      chdir(prev_dir);
#endif
      dot = rindex(file, '.');
      if(!dot) {
        stg_world_set_cursor_normal(world);
        continue; // re-open dialog box
      }
      snprintf(pat, 32, "*%s", dot);
      entry = (struct _stg_world_file_loader_entry*)g_hash_table_lookup(world->file_loaders, pat);
      if(!entry) {
        stg_world_set_cursor_normal(world);
        continue; // re-open dialog box
      }
      (entry->callback)(world, file, entry->userdata);
      stg_world_set_cursor_normal(world);
      break;
    }
    break;
  }
  gtk_widget_hide(GTK_WIDGET(dlg));
  gtk_widget_destroy(GTK_WIDGET(dlg));
}

stg_msec_t stg_world_get_sim_interval(stg_world_t* world) {
  return world->sim_interval;
}

stg_msec_t stg_world_get_real_interval(stg_world_t* world) {
  return world->real_interval;
}

stg_msec_t stg_world_get_last_interval(stg_world_t* world) {
  return world->real_interval_measured;
}


// model update callbacks. Maybe these should be moved back into
// model.c so that stg_model_update() can be inlined?

void _world_model_update_cb( gpointer key, gpointer value, gpointer user )
{
  STG_F()
  //stg_model_update( (stg_model_t*)value );
  stg_model_update_tree( (stg_model_t*)value, NULL );
}

#ifdef STG_ENABLE_WORLD_LOCK
void _world_model_update_locking_cb( gpointer key, gpointer value, gpointer user )
{
  STG_F()
  stg_model_t* mod = (stg_model_t*)value;
  //printf("> updating model %s...\n", mod->token);
  if(stg_model_lock(mod))
  {
    stg_model_update(mod);
    stg_model_unlock(mod);
  }
  else
  {
    stg_print_warning("Internal (Debug) Warning: in _world_model_update_lock_cb: a model became invalid while waiting for its mutex lock.");
  }
}

void _world_model_update_trylock_cb( gpointer key, gpointer value, gpointer user )
{
  STG_F()
  // Only update the model if it's not locked.  May or may not be useful...
  stg_model_t* mod = (stg_model_t*)value;
  // TODO if model updates end up being skipped too many times, force a lock
  if(stg_model_try_lock(mod) == STG_LOCK_SUCCESS) {
    stg_model_update(mod);
    stg_model_unlock(mod);
  } 
    // DEBUGGING:
  else {
    _trylock_skip_update_count++;
  }

}


void stg_world_enable_world_lock(stg_world_t* world, gboolean enable)
{
  if(enable)
  {
    int err = pthread_mutex_init(&world->mutex, NULL);
    assert(err == 0);
  }
  // else clear the mutex?
  world->world_locking_enabled = enable;

  stg_matrix_enable_lock(world->matrix, enable);
  gui_enable_lock(enable);

#ifdef STG_DEBUG_WORLD_PENDING_LOCKS
  pthread_mutex_init(&_stg_world_pending_lock_count_mutex, NULL);
#endif
}

void stg_world_set_model_update_locking_policy(stg_world_t* world, stg_world_locking_policy_t policy)
{
  world->model_locking_policy = policy;
  switch(policy)
  {
    case STG_WORLD_NO_LOCKING:
      world->model_update_cb = _world_model_update_cb;
      break;
    case STG_WORLD_LOCK:
      world->model_update_cb = _world_model_update_locking_cb;
      break;
    case STG_WORLD_SKIP_IF_LOCKED:
      world->model_update_cb = _world_model_update_trylock_cb;
      break;
  }
}
#endif


#if 0 
  -- NOT USED --
void stg_world_load_model_macro_properties(stg_world_t *world, stg_model_t *mod, int macrodef) {
  /* Depth-first recursion so that they get loaded from initial macro def on
   * down the linneage */
  if(macrodef == -1) return;
  printf("entity %d (sectiontype=%s, macroname=%s) encountered...\n", macrodef, wf_get_section_type(macrodef), wf_get_macro_name(macrodef));
  stg_world_load_model_macro_properties(world, mod, wf_get_macro_parent(macrodef));
  stg_model_load_from_worldfile_section_id(mod, macrodef);
}
#endif


#ifdef STG_ENABLE_WORLD_LOCK

void _stg_world_lock(stg_world_t *world)
{
  STG_F()
  if(world->world_locking_enabled)
  {
#ifdef STG_DEBUG_WORLD_PENDING_LOCKS
    // TODO move into header file (with #ifdef)
    unsigned int pending;
    pthread_mutex_lock(&_stg_world_pending_lock_count_mutex);
    pending = _stg_world_pending_lock_count;
    ++_stg_world_pending_lock_count;
    pthread_mutex_unlock(&_stg_world_pending_lock_count_mutex);
    if(pending > 3)
      stg_print_msg("stg_world_lock: there are currently %u other pending locks on the world mutex.\n", pending);
#endif
    //puts(">>> Locking World >>>");
    pthread_mutex_lock(& world->mutex);
    world->lock_time = stg_timenow();
    //puts("             (Locked.)");
#ifdef STG_DEBUG_WORLD_PENDING_LOCKS
    pthread_mutex_lock(&_stg_world_pending_lock_count_mutex);
    --_stg_world_pending_lock_count;
    pthread_mutex_unlock(&_stg_world_pending_lock_count_mutex);
#endif
  }
}

void _stg_world_unlock(stg_world_t *world)
{
  STG_F()
  if(world->world_locking_enabled)
    pthread_mutex_unlock(& world->mutex);
}

#endif


// Expose part of the worldfile API so apps can add new type definitions:

gboolean stg_world_type_defined(stg_world_t *world, const char *type)
{
  return (wf_lookup_section(type) != -1);
}

int stg_world_add_type_def(stg_world_t* world, const char *type, int parent)
{
  return wf_create_entity(type, parent);
}

void stg_world_set_type_property_int(stg_world_t* world, int wf_section_id, char *name, int value)
{
  wf_write_int(wf_section_id, name, value);
}

void stg_world_set_type_property_float(stg_world_t* world, int wf_section_id, char *name, double value)
{
  wf_write_float(wf_section_id, name, value);
}

void stg_world_set_type_property_string(stg_world_t* world, int wf_section_id, char *name, char *value)
{
  wf_write_string(wf_section_id, name, value);
}

void stg_world_set_type_property_tuple_int(stg_world_t* world, int wf_section_id, char *name, int which, int value)
{
  wf_write_tuple_int(wf_section_id, name, which, value);
}

void stg_world_set_type_property_tuple_float(stg_world_t* world, int wf_section_id, char *name, int which, double value)
{
  wf_write_tuple_float(wf_section_id, name, which, value);
}

void stg_world_set_type_property_tuple_string(stg_world_t* world, int wf_section_id, char *name, int which, char* value)
{
  wf_write_tuple_string(wf_section_id, name, which, value);
}

double stg_world_get_avg_interval_ratio(stg_world_t* world)
{
  return world->avg_interval_ratio;
}

size_t stg_world_num_zero_interval_warnings(stg_world_t* world) 
{
  return world->zero_interval_warnings;
}

size_t stg_world_num_interval_too_long_warnings(stg_world_t* world)
{
  return world->real_interval_too_long_warnings;
}

void stg_world_set_quiet(stg_world_t* world, gboolean b)
{
  world->quiet = b;
}

gboolean stg_world_get_quiet(stg_world_t* world)
{
  return world->quiet;
}


#ifdef STG_ENABLE_WORLD_LOCK
stg_msec_t stg_world_get_lock_time(stg_world_t* world)
{
  return world->lock_time;
}
#endif

void stg_world_display_degrees(stg_world_t *world)
{
  world->display_degrees = TRUE;
}

void stg_world_display_radians(stg_world_t *world)
{
  world->display_degrees = FALSE;
}

