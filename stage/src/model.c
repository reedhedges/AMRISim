

#define _GNU_SOURCE

#include <limits.h> 
#include <assert.h>
#include <math.h>
#include <string.h> // for strdup(3)
#ifdef HAVE_PTHREAD_H // have pthread.h and thread locking enabled
#include <pthread.h>
#endif
#include <errno.h>

// #define DEBUG 1
// #define DEBUG_USERPROPERTIES 1

#include "stage_internal.h"
#include "gui.h"



// limits.h seems to always define MAX_PATH. 
// Sometimes it defines _POSIX_PATH_MAX.
#ifdef _POSIX_PATH_MAX
# define STG_MAX_PATH _POSIX_PATH_MAX
#else
# ifdef MAX_PATH
#  define STG_MAX_PATH MAX_PATH
# else
#  define STG_MAX_PATH (256)
# endif
#endif

/** @define STG_MODEL_MUTEX_LOCK_WARNING_TIME
 *  If a model's mutex is locked for more than this many ms, print a warning.
 */
#define STG_MODEL_MUTEX_LOCK_WARNING_TIME 150

// basic model defaults
#define STG_DEFAULT_MASS 10.0  // kg
#define STG_DEFAULT_POSEX 0.0  // start at the origin by default
#define STG_DEFAULT_POSEY 0.0
#define STG_DEFAULT_POSEA 0.0
#define STG_DEFAULT_GEOM_POSEX 0.0 // no origin offset by default
#define STG_DEFAULT_GEOM_POSEY 0.0
#define STG_DEFAULT_GEOM_POSEA 0.0
#define STG_DEFAULT_GEOM_SIZEX 1.0 // 1m square by default
#define STG_DEFAULT_GEOM_SIZEY 1.0
#define STG_DEFAULT_OBSTACLERETURN TRUE
#define STG_DEFAULT_LASERRETURN 1
#define STG_DEFAULT_RANGERRETURN TRUE
#define STG_DEFAULT_BLOBRETURN TRUE
#define STG_DEFAULT_COLOR (0xFF0000) // red
#define STG_DEFAULT_ENERGY_CAPACITY 1000.0
#define STG_DEFAULT_ENERGY_CHARGEENABLE 1
#define STG_DEFAULT_ENERGY_PROBERANGE 0.0
#define STG_DEFAULT_ENERGY_GIVERATE 0.0
#define STG_DEFAULT_ENERGY_TRICKLERATE 0.1
#define STG_DEFAULT_MASK (STG_MOVE_TRANS | STG_MOVE_ROT)
#define STG_DEFAULT_NOSE FALSE
#define STG_DEFAULT_GRID FALSE
#define STG_DEFAULT_OUTLINE TRUE

//extern int _stg_disable_gui;

#define PACKPOSE(P,X,Y,A) {P->x=X; P->y=Y; P->a=A;}

static stg_bool_t print_debug_for_jon = FALSE; // TODO: remove
static int tab_cnt_for_jon = 0; // TODO: remove
void print_tabs_for_jon()
{
  int i;
  for (i = 0; i < tab_cnt_for_jon; ++i)
    printf("\t");
}


/** @addtogroup stage 
    @{ 
*/

/** @defgroup model Models
    
The basic model simulates an object with basic properties; position,
size, velocity, color, visibility to various sensors, etc. The basic
model also has a body made up of a list of lines. Internally, the
basic model is used base class for all other model types. You can use
the basic model to simulate environmental objects.

<h2>Worldfile properties</h2>

@par Summary and default values

@verbatim
model
(
  pose [0 0 0]
  size [0 0]
  origin [0 0 0]
  velocity [0 0 0]

  color "red" # body color

  # determine how the model appears in various sensors
  obstacle_return 1
  laser_return 1
  ranger_return 1
  blobfinder_return 1
  fiducial_return 1

  # a few sensors use height info to detect other models (e.g. complex laser):
  height 0.5  # height dimension of this model
  height_offset 0 # vertical offset of this model in relation to its parent model, if any

  # GUI properties
  gui_nose 0
  gui_grid 0
  gui_boundary 0
  gui_movemask ?

  # body shape
  line_count 4
  line[0][0 0 1 0]
  line[1][1 0 1 1]
  line[2][1 1 0 1]
  line[3][0 1 0 0]

  # name of bitmap file describing body shape
  bitmap ""
)
@endverbatim

@par Details
TODO integrate some or all of these into the model struct (for speed)
- pose [x_pos:float y_pos:float heading:float]
  - specify the pose of the model in its parent's coordinate system
  - TODO integrate this into model struct
- size [x_size:float ysize:float]
  - specify the size of the model
  - TODO integrate this into model struct
- origin [x_pos:float y_pos:float heading:float]
  - specify the position of the object's center, relative to its pose
  - TODO integrate this into model struct
- velocity [x_speed:float y_speed:float rotation_speed:float]
  - specify the initial velocity of the model. Not that if the model hits an obstacle, its velocity will be set to zero.
  - TODO integrate this into model struct
- color [colorname:string]
  - specify the color of the object using a color name from the X11 database (rgb.txt)
  - TODO integrate this into model struct
- line_count [int]
  - specify the number of lines that make up the model's body
- line[index] [x1:float y1:float x2:float y2:float]
  - creates a line from (x1,y1) to (x2,y2). A set of line_count lines defines the robot's body for the purposes of collision detection and rendering in the GUI window.
- bitmap [filename:string}
  - alternative way to set the model's line_count and lines. The file must be a bitmap recognized by libgtkpixbuf (most popular formats are supported). The file is opened and parsed into a set of lines. Unless the bitmap_resolution option is used, the lines are scaled to fit inside the rectangle defined by the model's current size.
- gui_nose [bool]
  - if 1, draw a nose on the model showing its heading (positive X axis)
- gui_grid [bool]
  - if 1, draw a scaling grid over the model
- gui_movemask [bool]
  - define how the model can be moved by the mouse in the GUI window
- gui_boundary [bool]
  - if 1, draw a bounding box around the model, indicating its size
- obstacle_return [bool]
  - if 1, this model can collide with other models that have this property set
- blob_return [bool]
  - if 1, this model can be detected in the blob_finder (depending on its color)
- ranger_return [bool]
  - if 1, this model can be detected by ranger sensors
- laser_return [int]
  - if 0, this model is not detected by laser sensors. if 1, the model shows up in a laser sensor with normal (0) reflectance. If 2, it shows up with high (1) reflectance.
- fiducial_return [fiducial_id:int]
  - if non-zero, this model is detected by fiducialfinder sensors. The value is used as the fiducial ID.
- ranger_return [bool]
   - iff 1, this model can be detected by a ranger.

A few sensors use height info to detect other models (e.g. complex laser). These properties, however, can
be missing if a model's height info is not known or is not to be used (it should be considered to be "infinitely"
tall)
- height [float]
  - height dimension of this model (meters)
- height_offset [float]
  - vertical offset of this model (meters) in relation to its parent model, if any
*/

/** @} */  

/** 
@ingroup stg_model
@defgroup stg_model_props Model Properties

- "pose" stg_pose_t
- "geom" stg_geom_t
- "velocity" stg_velocity_t
- "color" stg_color_t
- "fiducial_return" stg_fiducial_return_t
- "laser_return" stg_laser_return_t
- "obstacle_return" stg_obstacle_return_t
- "ranger_return" stg_ranger_return_t
- "gripper_return" stg_gripper_return_t
- "height" stg_meters_t  (may be missing)
- "height_offset" stg_meters_t   (may be missing)
- "height_in_world" stg_meters_t
- "polygons" stg_poly_t array
- "points" stg_point_t array
*/


/*
  TODO
  
  - friction float [WARNING: Friction model is not yet implemented;
    details may change] if > 0 the model can be pushed around by other
    moving objects. The value determines the proportion of velocity
    lost per second. For example, 0.1 would mean that the object would
    lose 10% of its speed due to friction per second. A value of zero
    (the default) means this model can not be pushed around (infinite
    friction).
*/
  
  
  // callback functions that handle property changes, mostly for drawing stuff in the GUI
  
int model_render_polygons( stg_model_t* mod, char* name, 
			     void* data, size_t len, void* userp );
int model_render_points( stg_model_t* mod, char* name, 
			     void* data, size_t len, void* userp );
int model_handle_mask( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );
int model_handle_outline( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp );
int model_render_nose( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );
int model_render_grid( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );


/* This function is called when creating a new model. It looks in the
 * worldfile for properties the user has requested with
 * stg_model_user_property().
 * @param mod The new model
 */
void stg_model_load_user_properties(stg_model_t* mod, int id);



// convert a global pose into the model's local coordinate system
void stg_model_global_to_local( stg_model_t* mod, stg_pose_t* pose )
{
  double sx, sy, sa;
  stg_pose_t org;

  //printf( "g2l global pose %.2f %.2f %.2f\n",
  //  pose->x, pose->y, pose->a );

  // get model's global pose
  stg_model_get_global_pose( mod, &org );

  //printf( "g2l global origin %.2f %.2f %.2f\n",
  //  org.x, org.y, org.a );

  // compute global pose in local coords
  sx =  (pose->x - org.x) * cos(org.a) + (pose->y - org.y) * sin(org.a);
  sy = -(pose->x - org.x) * sin(org.a) + (pose->y - org.y) * cos(org.a);
  sa = pose->a - org.a;

  PACKPOSE(pose,sx,sy,sa);

  //printf( "g2l local pose %.2f %.2f %.2f\n",
  //  pose->x, pose->y, pose->a );
}


// generate the default name for a model, based on the name of its
// parent, and its type. This can be overridden in the world file.
/* int stg_model_create_name( stg_model_t* mod ) */
/* { */
/*   assert( mod ); */
/*   assert( mod->token ); */
  
/*   PRINT_DEBUG1( "model's default name is %s", */
/* 		mod->token ); */
/*   if( mod->parent ) */
/*     PRINT_DEBUG1( "model's parent's name is %s", */
/* 		  mod->parent->token ); */

/*   char buf[512];   */
/*   if( mod->parent == NULL ) */
/*     snprintf( buf, 255, "%s:%d",  */
/* 	      mod->token,  */
/* 	      mod->world->child_type_count[mod->type] ); */
/*   else */
/*     snprintf( buf, 255, "%s.%s:%d",  */
/* 	      mod->parent->token, */
/* 	      mod->token,  */
/* 	      mod->parent->child_type_count[mod->type] ); */
  
/*   free( mod->token ); */
/*   // return a new string just long enough */
/*   mod->token = strndup(buf,512); */

/*   return 0; //ok */
/* } */



/* store some data in a property, re-allocating space as needed and copying data
  into property. 
 Warning: any extant pointers to old data may become invalid
 after this function if reallocated (depending on platform implementation of 
   realloc)
  If 'len' is 0, any existing memory is freed, property data is nulled (will be allocated again on next call with data).
  If 'data' argument is NULL, then size for 'len' space is allocated and all space is set to 0.
  Otherwise, memmove() is used to set the data from the memory pointed to by the 'data' pointer.
*/
void storage_ordinary( stg_property_t* prop, 
		       void* data, size_t len )
{
  assert(prop);
  if(len == 0)
  {
    if(prop->data) free(prop->data);
    prop->data = NULL;
    return;
  }
  if (data == prop->data && len > 0) return;  // don't need to reallocate or copy
  prop->data = realloc( prop->data, len );
  assert(prop->data);
  prop->len = len;
  if(data == NULL)
    memset(prop->data, 0, len);
  else
    memmove(prop->data, data, len);
  // realloc can result in overlapping memory areas, can't use memcpy( prop->data, data, len );
}



// store a model's polygons into a property, and remap the model into the matrix
// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
void storage_polygons( stg_property_t* prop, 
		       void* data, size_t len ) 
{
  if(print_debug_for_jon)
  {
    tab_cnt_for_jon++;
    print_tabs_for_jon();
    printf("storage_polygons(): begin\n");
  }

  stg_polygon_t* polys;
  stg_geom_t* geom;
  size_t count;
 
  assert(prop);
  if( len > 0 ) assert(data);
  
  polys = (stg_polygon_t*)data;
  count = len / sizeof(stg_polygon_t);
  
  geom = 
    stg_model_get_property_fixed( prop->mod, "geom", sizeof(stg_geom_t));
    
  //printf( "model %d(%s) received %d polygons\n", 
  //  prop->mod->id, prop->mod->token, (int)count );
  
  if(print_debug_for_jon)
  {
    print_tabs_for_jon();
    printf("storage_polygons(): prop->mod->should_scale: %d\n", prop->mod->should_scale);
    print_tabs_for_jon();
    printf("storage_polygons(): geom: %p\n", geom);
  }


  // normalize the polygons to fit exactly in the model's body
  // size (if it has geom property)
  if(prop->mod->should_scale && geom)
  {
    if(print_debug_for_jon)
    {
      print_tabs_for_jon();
      printf("storage_polygons(): calling stg_polygons_normalize\n");
    }
    stg_polygons_normalize( polys, count, geom->size.x, geom->size.y );
  }
  
  stg_model_map( prop->mod, 0 ); // unmap the model from the matrix
  
  storage_ordinary( prop, polys, len );
  
  stg_model_map( prop->mod, 1 ); // map the model into the matrix with the new polys

  if(print_debug_for_jon)
  {
    print_tabs_for_jon();
    tab_cnt_for_jon--;
    printf("storage_polygons(): end\n");
  }
} 

// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
void storage_geom( stg_property_t* prop, 
		   void* data, size_t len )
{
  //printf("======== storage_geom callback for \"%s\" property of model \"%s\" (%p)... (will remap model)\n", prop->name, prop->mod->token, prop->mod);
  assert( len == sizeof(stg_geom_t));

  // unrender from the matrix
  stg_model_map( prop->mod, 0 );
  
  storage_ordinary( prop, data, len );
  
  // we probably need to scale and re-render our polygons
  stg_model_property_changed( prop->mod, "polygons" );
  stg_model_property_changed( prop->mod, "nose" );
  
  // re-render int the matrix
  stg_model_map( prop->mod, 1 );  
}



// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
void storage_color( stg_property_t* prop,
		    void* data, size_t len )
{
  assert( len == sizeof(stg_color_t));

  storage_ordinary( prop, data, len );
  
  // redraw the polygons in the new color
  stg_model_property_changed( prop->mod, "polygons" );
}


// set the pose of a model in its parent's CS 
// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
void storage_pose( stg_property_t* prop,
		   void* data, size_t len )
{
  //printf("======== storage_pose callback for \"%s\" property of model \"%s\" (%p)... (will remap model)\n", prop->name, prop->mod->token, prop->mod);

  assert( len == sizeof(stg_pose_t) );  
  assert( data );
  
  //if( memcmp( prop->data, data, sizeof(stg_pose_t) )) 
  { 

    // unrender from the matrix
    stg_model_map_with_children( prop->mod, 0 );
    
    storage_ordinary( prop, data, len );
    
    // render in the matrix
    stg_model_map_with_children( prop->mod, 1 );
    
    // move the stg_rtk figure to match
    if( prop->mod->world->win )
      gui_model_move( prop->mod );
  }
}


// store an array of stg_point_t and remap the model into the matrix
// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
void storage_points(stg_property_t *prop, void *data, size_t len)
{
    stg_geom_t *geom;

  //printf("======== storage_points callback for \"%s\" property of model \"%s\" (%p)... (will remap model)\n", prop->name, prop->mod->token, prop->mod);

    assert(prop);
    if(len > 0) assert(data);

    // scale to fit inside prop's model's size
    geom = stg_model_get_property_fixed(prop->mod, "geom", sizeof(stg_geom_t));
    if(prop->mod->should_scale && geom)
        stg_points_normalize((stg_point_t*)data, len/sizeof(stg_point_t), geom->size.x, geom->size.y);
    stg_model_map(prop->mod, 0); // unmap the model from the matrix
    storage_ordinary(prop, data, len); 
    stg_model_map(prop->mod, 1);  // remap
}

//static int _model_init = TRUE;

stg_model_t* stg_model_create( stg_world_t* world, 
			        stg_model_t* parent,
			        stg_id_t entity_id,
			        const char* token,
                    const char* base_type_name,
                    const char* instance_type_name,
                    size_t index,
			        stg_model_initializer_t initializer,
              stg_bool_t bg_fig )
{  
  int err;
  stg_model_t* mod = NULL;
  PRINT_DEBUG6("stg_model_create called with world=%p, parent=%p, id=%d, token=\"%s\", base_type_name=\"%s\", instance_type_name=\"%s\"", world, parent, entity_id, token, base_type_name, instance_type_name);

  //if(!strcmp(base_type_name, "ranger")) printf("======== XXX creating ranger with parent=%p (%s)\n", parent, parent?parent->token:"null");
  //mod = (stg_model_t*) calloc(1, sizeof(stg_model_t)); 
  PRINT_DEBUG1("allocating mod sizeof stg_model_t %lu\n", sizeof(stg_model_t));
  mod = (stg_model_t*) malloc(sizeof(stg_model_t));
  PRINT_DEBUG1("allocated mod %p.\n", mod)
  assert(mod);
  

  mod->initializer = initializer;

#ifdef STG_ENABLE_MODEL_LOCK
  err = pthread_mutex_init( &mod->mutex, NULL ); 
  if( err )
  { 
    PRINT_ERR3( "FATAL ERROR thread initialization failed for model %x (%s) with code %d\n", mod, mod->token, err ); 
    exit(-1);
  }
  mod->mutex_init = TRUE;
#endif
  mod->lock_time = 0;

  /* if( _model_init ) */
  /*     { */
  /*       g_datalist_init( mod->props ); */
  /*       _model_init = FALSE; */
  /*     } */
  mod->props = g_hash_table_new(g_str_hash, g_str_equal);
    
  //mod->id = id; // Replaced with entity_id and unique_id, so this is obsolete
  //mod->unique_id = ?; // This is initialized when the model is inserted in to the hash table in stg_world_add_model()
  mod->entity_id = entity_id;
  
  mod->subs = 0;
  mod->interval = 0; // unused
  mod->interval_elapsed = 0;
  mod->disabled = FALSE;
  mod->collision = FALSE;
  mod->world = world;
  mod->parent = parent; 
  if(parent)
    mod->root = parent->root;
  else
    mod->root = NULL;
  mod->token = strdup(token); // could use model_create_name() instead. freed in model_destroy

  // create a default name for the model that's derived from its
  // ancestors' names and its worldfile token
  //model_create_name( mod );
  
  mod->base_type_name = strdup(base_type_name);  // freed in model_destroy
  mod->instance_name = strdup(instance_type_name);// freed in model_destroy
  mod->type_instance_index = index;

  mod->property_toggles = NULL;

  mod->current_matrix_cell = NULL;

  mod->bg_fig = bg_fig;
  mod->should_scale = TRUE;

  PRINT_DEBUG4( "creating model %d.%d(%s) [type=%s]", 
		mod->world->id,
		mod->entity_id, 
		mod->token,
    mod->base_type_name);
  
  PRINT_DEBUG1( "original token: %s", token );

  // add this model to its parent's list of children (if any)
  if(parent) { 
    assert(parent->children);  
    g_ptr_array_add( parent->children, mod); 
  }
  

  // create this model's empty list of children
  mod->children = g_ptr_array_new();

  // install the default functions
  mod->f_startup = NULL;
  mod->f_shutdown = NULL;
  mod->f_update = _model_update;
  mod->f_destroy = NULL;


  /* Properties */


  {
    stg_geom_t geom;
    memset( &geom, 0, sizeof(geom));
    geom.size.x = 1.0;
    geom.size.y = 1.0;
    stg_model_set_property_ex( mod, "geom", &geom, sizeof(geom), storage_geom );
  }
 
  {
    stg_pose_t pose; 
    memset( &pose, 0, sizeof(pose));
    stg_model_set_property_ex( mod, "pose", &pose, sizeof(pose), storage_pose );
    stg_model_set_property_ex( mod, "initial_pose", &pose, sizeof(pose), storage_pose );
  }
    
  {
    stg_obstacle_return_t obs = 1;
    stg_model_set_property( mod, "obstacle_return", &obs, sizeof(obs));
  }
    
  {
    stg_ranger_return_t rng = 1;
    stg_model_set_property( mod, "ranger_return", &rng, sizeof(rng));
  }
    
  {
    stg_blob_return_t blb = 1;
    stg_model_set_property( mod, "blob_return", &blb, sizeof(blb));
  }
    
  {
    stg_laser_return_t lsr = 1;
    stg_model_set_property( mod, "laser_return", &lsr, sizeof(lsr));
  }
    
  {
    stg_gripper_return_t grp = 0;
    stg_model_set_property( mod, "gripper_return", &grp, sizeof(grp));
  }
    
  {
    stg_bool_t bdy = 0;
    stg_model_set_property( mod, "boundary", &bdy, sizeof(bdy));
  }
    
  {
    // body color
    stg_color_t col = 0xFF0000; // red;  
    stg_model_set_property_ex( mod, "color", &col, sizeof(col), storage_color );
  }
    
//  // create polygons model but set it to NULL 
  stg_model_set_property_ex( mod, "polygons", NULL, 0, storage_polygons );
        
  {
    int nose = STG_DEFAULT_NOSE;
    stg_model_set_property( mod, "nose", &nose, sizeof(nose) );
  }

  {
    int grid = STG_DEFAULT_GRID;
    stg_model_set_property( mod, "grid", &grid, sizeof(grid) );
  }
  
  {
    int outline = STG_DEFAULT_OUTLINE;
    stg_model_set_property( mod, "outline", &outline, sizeof(outline) );
  }
  
  {
    int mask = mod->parent ? 0 : STG_DEFAULT_MASK;
    stg_model_set_property( mod, "mask", &mask, sizeof(mask) );
  }

  // now it's safe to create the GUI components
  if( mod->world->win )
    gui_model_create( mod );
  
  // experimental: creates a menu of models
  gui_lock();
  gui_add_tree_item( mod );
  gui_unlock();
  
  if(mod->world->gui_enabled)
  {
    if(stg_model_property_exists(mod, "polygons"))
    {
        stg_model_add_property_callback( mod, "polygons", model_render_polygons, NULL );
        stg_model_add_property_callback( mod, "color", model_render_polygons, NULL );
        stg_model_property_changed( mod, "polygons" ); 
    }
    if(stg_model_property_exists(mod, "points"))
    {
        //printf("adding property callback for points property on model %s\n", mod->token);
        stg_model_add_property_callback( mod, "points", model_render_points, NULL );
        stg_model_add_property_callback( mod, "color", model_render_points, NULL );
        stg_model_property_changed( mod, "points" ); 
    }
    stg_model_add_property_callback( mod, "mask", model_handle_mask, NULL );
    stg_model_add_property_callback( mod, "nose", model_render_nose, NULL );
    stg_model_add_property_callback( mod, "grid", model_render_grid, NULL );
    stg_model_add_property_callback( mod, "outline", model_handle_outline, NULL );
    
    // force redraws
    stg_model_property_changed( mod, "polygons" ); 
    stg_model_property_changed( mod, "mask" ); 
    stg_model_property_changed( mod, "nose" ); 
    stg_model_property_changed( mod, "grid" ); 
  }
  
  PRINT_DEBUG3( "finished model %d.%d(%s)", 
		mod->world->id, 
		mod->entity_id, 
		mod->token );
  
  if( mod->initializer )
    mod->initializer(mod);


  return mod;
}

void stg_property_print_cb( gpointer key, gpointer data, gpointer user )
{
  stg_property_t* prop = (stg_property_t*)data;
  
  printf( "%s key: %s name: %s len: %d func: %p\n", 
	  user ? (char*)user : "", 
	  (char*)(key), 	  
	  prop->name,
	  (int)prop->len,
	  prop->storage_func );
}

void stg_model_print_properties( stg_model_t* mod )
{
  printf( "Model \"%s\" has  properties:\n",
	  mod->token );
  
  g_hash_table_foreach( mod->props, stg_property_print_cb, "   " );
  puts( "   <end>" );  
}

void stg_property_destroy_x(stg_property_t *prop, const char *propname);

void stg_property_delete_cb(gpointer key, gpointer itemdata, gpointer userdata)
{
  stg_property_t* prop = (stg_property_t*)itemdata;
  //if(prop) stg_property_destroy(prop);
  if(prop) stg_property_destroy_x(prop, prop->name);  // pass name as arg so name shows up in gdb
}

void stg_model_destroy_properties(stg_model_t* mod)
{
  g_hash_table_foreach(mod->props, stg_property_delete_cb, mod);
}


void stg_model_destroy(stg_model_t *mod)
{
  stg_model_destroy_ex(mod, FALSE);
}

void stg_model_destroy_tree(stg_model_t *mod)
{
  stg_model_destroy_ex(mod, TRUE);
}

// free the memory allocated for a model, and optionally, destroy children
/** @todo Implement destruction callbacks. */
void stg_model_destroy_ex( stg_model_t* mod, stg_bool_t destroy_children )
{
  PRINT_DEBUG2("destroying model %p (%s)", mod, mod?mod->token:"<na>");
  assert( mod );
  gui_remove_tree_item(mod);
  if(mod->f_destroy != NULL)
    (mod->f_destroy)(mod);

  // remove a mouse handler that may have been added by model_handle_mask (which
  // is called if a mask is added to this model)
  stg_rtk_fig_t *fig = stg_model_get_fig(mod, "top");
  if(fig) stg_rtk_fig_remove_mouse_handler(fig, gui_model_mouse );

  if( mod->world->win )
    gui_model_destroy( mod );
  if( mod->parent && mod->parent->children ) {
    g_ptr_array_remove( mod->parent->children, mod );
  }
  mod->parent = NULL;
  if( mod->children ) {
    int c;
    PRINT_DEBUG2("stg_model_destroy: clearing child list of %s (%p)...", mod->token, mod);
    for(c = 0; c < mod->children->len; ++c)
    {
      stg_model_t* child = g_ptr_array_index(mod->children, c);
      if (child)
      {
        if(destroy_children)
          stg_model_destroy_ex(child, TRUE);
        else
          child->parent = NULL; ///< @todo could reparent instead
      }
     }
    g_ptr_array_free( mod->children, TRUE );
  }
  mod->children = NULL;
  stg_model_destroy_polygons(mod);
  stg_model_destroy_properties(mod);

  g_hash_table_destroy(mod->props);
  mod->props = NULL;

  if( mod->token ) free( mod->token ); // allocated in model_create
  mod->token = NULL;
  if( mod->base_type_name ) free(mod->base_type_name); // allocated in model_create
  mod->base_type_name = NULL;
  if( mod->instance_name ) free(mod->instance_name); // allocated in model_create
  mod->instance_name = NULL;

#ifdef STG_ENABLE_MODEL_LOCK
  // TODO you're not supposed to destroy a mutex that's locked. They say bad things will
  // happen.  Generally destroying models from other threads is kind of broken
  // and we either need full reference counting, or a "mark for removal" and
  // "mark for deletion" flags and destroy models in stg_world_update.
  pthread_mutex_destroy(&mod->mutex);
  mod->mutex_init = FALSE;
#endif

  memset(mod, 0, sizeof(stg_model_t));
  free( mod );
}


void model_destroy_cb( gpointer mod )
{
  stg_model_destroy( (stg_model_t*)mod );
}

int stg_model_is_antecedent( stg_model_t* mod, stg_model_t* testmod )
{
  if( mod == NULL )
    return FALSE;

  if( mod == testmod )
    return TRUE;
  
  if( stg_model_is_antecedent( mod->parent, testmod ))
    return TRUE;
  
  // neither mod nor a child of mod matches testmod
  return FALSE;
}

// returns TRUE if model [testmod] is a descendent of model [mod]
int stg_model_is_descendent( stg_model_t* mod, stg_model_t* testmod )
{
  int ch;
  if( mod == testmod )
    return TRUE;

  assert( mod->children );

  for(ch=0; ch < mod->children->len; ch++ )
    {
      stg_model_t* child = g_ptr_array_index( mod->children, ch );
      if( stg_model_is_descendent( child, testmod ))
	return TRUE;
    }
  
  // neither mod nor a child of mod matches testmod
  return FALSE;
}

// returns 1 if model [mod1] and [mod2] are in the same model tree
int stg_model_is_related( stg_model_t* mod1, stg_model_t* mod2 )
{
  stg_model_t* t;

  if( mod1 == mod2 )
    return TRUE;

  // find the top-level model above mod1;
  t = mod1;
  while( t->parent )
    t = t->parent;

  // now seek mod2 below t
  return stg_model_is_descendent( t, mod2 );
}

// get the model's velocity in the global frame
void stg_model_global_velocity( stg_model_t* mod, stg_velocity_t* gvel )
{
  stg_pose_t gpose;
  double cosa, sina;
  stg_velocity_t* lvel;

  stg_model_get_global_pose( mod, &gpose );

  cosa = cos( gpose.a );
  sina = sin( gpose.a );
  
  lvel = 
    stg_model_get_property_fixed( mod, "velocity", sizeof(stg_velocity_t));
  
  if( lvel )
    {
      gvel->x = lvel->x * cosa - lvel->y * sina;
      gvel->y = lvel->x * sina + lvel->y * cosa;
      gvel->a = lvel->a;
    }
  else // no velocity property - we're not moving anywhere
    memset( gvel, 0, sizeof(stg_velocity_t));
      
  //printf( "local velocity %.2f %.2f %.2f\nglobal velocity %.2f %.2f %.2f\n",
  //  mod->velocity.x, mod->velocity.y, mod->velocity.a,
  //  gvel->x, gvel->y, gvel->a );
}

// set the model's velocity in the global frame
/* void stg_model_set_global_velocity( stg_model_t* mod, stg_velocity_t* gvel ) */
/* { */
/*   // TODO - do this properly */

/*   //stg_pose_t gpose; */
  
/*   stg_velocity_t lvel; */
/*   lvel.x = gvel->x; */
/*   lvel.y = gvel->y; */
/*   lvel.a = gvel->a; */

/*   stg_model_set_velocity( mod, &lvel ); */
/* } */

// get the model's position in the global frame
void  stg_model_get_global_pose( stg_model_t* mod, stg_pose_t* gpose )
{ 
  stg_pose_t parent_pose;
  
  stg_pose_t* pose = 
    stg_model_get_property_fixed( mod, "pose", sizeof(stg_pose_t));

  if(!pose)
  {
    printf("stg_model_get_global_pose: internal error: model %s (%p) has no pose. Using 0,0,0.\n", mod->token, mod);
    gpose->x = 0;
    gpose->y = 0;
    gpose->a = 0;
    return;
  }
  
  // find my parent's pose
  if( mod->parent )
    {
      stg_model_get_global_pose( mod->parent, &parent_pose );
      
      gpose->x = parent_pose.x + pose->x * cos(parent_pose.a) 
	- pose->y * sin(parent_pose.a);
      gpose->y = parent_pose.y + pose->x * sin(parent_pose.a) 
	+ pose->y * cos(parent_pose.a);
      gpose->a = NORMALIZE(parent_pose.a + pose->a);
    }
  else
    memcpy( gpose, pose, sizeof(stg_pose_t));
}
    
    

// convert a pose in this model's local coordinates into global
// coordinates
// should one day do all this with affine transforms for neatness?
void stg_model_local_to_global( stg_model_t* mod, stg_pose_t* pose )
{  
  stg_pose_t origin;   
  stg_model_get_global_pose( mod, &origin );
  stg_pose_sum( pose, &origin, pose );
}


// recursively map a model and all it's descendents
void stg_model_map_with_children(  stg_model_t* mod, gboolean render )
{
  // call this function for all the model's children
  int ch;
  for( ch=0; ch<mod->children->len; ch++ )
    stg_model_map_with_children( (stg_model_t*)g_ptr_array_index(mod->children, ch), 
				 render);  
  // now map the model
  stg_model_map( mod, render );
}

// if render is true, render the model into the matrix, else unrender
// the model
void stg_model_map( stg_model_t* mod, gboolean render )
{
  size_t polycount=0;
  size_t pointcount = 0;
  stg_geom_t geom;
  stg_polygon_t* polys;
  stg_point_t* points;
  stg_bool_t* boundary;
  stg_pose_t org;

  assert( mod );

  //printf("======== stg_model_map: mod=\"%s\" (%p), render=%d\n", mod->token, mod, render); fflush(stdout);

  if(render == FALSE)
  {
    // remove:
    stg_matrix_remove_object( mod->world->matrix, mod );
    stg_matrix_unlock(mod->world->matrix);
    mod->current_matrix_cell = NULL;
    return;
  }

  // add:
  polys = stg_model_get_polygons(mod, &polycount);
  points = stg_model_get_points(mod, &pointcount);
  
  stg_model_get_geom( mod, &geom );

  // get model's global pose
  memcpy( &org, &geom.pose, sizeof(org));
  stg_model_local_to_global( mod, &org );
  
  stg_matrix_lock(mod->world->matrix);


  if(polys && polycount > 0)
  {
    stg_matrix_polygons( mod->world->matrix, org.x, org.y, org.a, polys, polycount, mod );  
  }

  if(points && pointcount > 0)
  {
    stg_matrix_points(mod->world->matrix, org.x, org.y, org.a, points, pointcount, mod);
  }

  boundary = stg_model_get_property_fixed( mod, "boundary", sizeof(stg_bool_t));
  if( *boundary )    
  {
    stg_matrix_rectangle( mod->world->matrix,
           org.x, org.y, org.a,
           geom.size.x,
           geom.size.y,
           mod ); 
  }

  //printf("model_update (%s): current cell is 0x%x, searching for new one at %f, %f...\n", mod->token, mod->current_matrix_cell, org.x, org.y);
  //if(mod->current_matrix_cell)
  //  mod->current_matrix_cell = stg_cell_locate(mod->current_matrix_cell, org.x, org.y);
  //else if(mod->world && mod->world->matrix && mod->world->matrix->root)
    mod->current_matrix_cell = stg_cell_locate(mod->world->matrix->root, org.x, org.y);
  //printf("\t...ok found cell 0x%x\n", mod->current_matrix_cell);


  stg_matrix_unlock(mod->world->matrix);

}




void stg_model_subscribe( stg_model_t* mod )
{
  mod->subs++;
  mod->world->subs++;

  //printf( "subscribe %d\n", mod->subs );
  
  // if this is the first sub, call startup
  if( mod->subs == 1 )
    stg_model_startup(mod);
}

void stg_model_unsubscribe( stg_model_t* mod )
{
  mod->subs--;
  mod->world->subs--;

  //printf( "unsubscribe %d\n", mod->subs );

  // if this is the last sub, call shutdown
  if( mod->subs < 1 )
    stg_model_shutdown(mod);
}


void pose_invert( stg_pose_t* pose )
{
  pose->x = -pose->x;
  pose->y = -pose->y;
  pose->a = pose->a;
}

void stg_model_print( stg_model_t* mod )
{
  stg_model_print_ex(mod, FALSE, 0);
  //printf( "   model %d:%d:%s\n", mod->world->id, mod->id, mod->token );
}

void stg_model_print_tree(stg_model_t* mod)
{
  stg_model_print_ex(mod, TRUE, 0);
}

void stg_model_print_ex( stg_model_t* mod, stg_bool_t print_children, int indent)
{

  printf( "%*smodel %d:%d:%s\n", indent, "", mod->world->id, mod->unique_id, mod->token );  // TODO: not sure if the id print was meant to identify the model's unique_id or entity_id
  if(print_children && mod->children && mod->children->len > 0)
  {
    int i;
    printf( "%*s      %d children:\n", indent, "", mod->children->len);
    for(i = 0; i < mod->children->len; ++i)
      stg_model_print_ex( (stg_model_t*) g_ptr_array_index(mod->children, i), TRUE, indent+1+6 );
  }
}

void model_print_cb( gpointer key, gpointer value, gpointer user )
{
  stg_model_print( (stg_model_t*)value );
}

void model_print_pre_update( stg_model_t* mod)
{
  stg_print_msg ("%s: update", mod->token);
  stg_print_msg ("\t  self address: %p", mod);
  stg_print_msg ("\t  parent address: %p", mod->parent);
  if (mod->children != NULL)
  {
    int idx;
    for (idx = 0; idx < mod->children->len; ++idx)
    {
      stg_model_t* cur_child = g_ptr_array_index(mod->children,idx);
      stg_print_msg ("\t  child %i, name: %s, address: %p", idx, cur_child->token, cur_child);
    }
  }
}

void stg_get_default_pose( stg_pose_t* pose )
{
  assert(pose);
  pose->x = STG_DEFAULT_GEOM_POSEX;
  pose->y = STG_DEFAULT_GEOM_POSEY;
  pose->a = STG_DEFAULT_GEOM_POSEA;
}

void stg_get_default_geom( stg_geom_t* geom )
{
  assert(geom);
  geom->pose.x = STG_DEFAULT_GEOM_POSEX;
  geom->pose.y = STG_DEFAULT_GEOM_POSEY;
  geom->pose.a = STG_DEFAULT_GEOM_POSEA;  
  geom->size.x = STG_DEFAULT_GEOM_SIZEX;
  geom->size.y = STG_DEFAULT_GEOM_SIZEY;
}


// default update function that implements velocities for all objects
int _model_update( stg_model_t* mod )
{
  stg_velocity_t* vel;

  mod->interval_elapsed = 0;

  vel = 
    stg_model_get_property_fixed( mod, "velocity", sizeof(stg_velocity_t));

  // now check and maybe move the model if it has any velocity or was previously collided
  if( mod->collision || (vel && (vel->x || vel->y || vel->a ) ) )
    stg_model_update_pose( mod );

  return 0; //ok
}

int stg_model_update( stg_model_t* mod )
{
  return( mod->f_update ? mod->f_update(mod) : 0 );
}

int stg_model_update_tree( stg_model_t* mod, stg_model_t* parent )
{
  if (mod == NULL)
  {
    stg_print_msg( "stg_model_update_tree(): mod == NULL. Returning");
    return -1;
  }

  //model_print_pre_update( mod );

  if ( mod->parent != parent )
  {
    //stg_print_msg( "\tstg_model_update_tree(): mod->parent(%p) != parent(%p). Returning", mod->parent, parent );
    return -1;
  }

  int retval = 0;
  if( mod->f_update != NULL )
    retval = mod->f_update( mod );

  // Tree recursion for children
  if (mod->children != NULL)
  {
    int idx;
    for (idx = 0; idx < mod->children->len; ++idx)
    {
      stg_model_t* cur_child = g_ptr_array_index(mod->children,idx);
      stg_model_update_tree( cur_child, mod );
    }
  }

  return retval;
}


int stg_model_startup( stg_model_t* mod )
{
  if( mod->f_startup )
    return mod->f_startup(mod);
  else
    PRINT_WARN1( "model %s has no startup function registered", mod->token ); 
  
  //assert(mod->f_startup);

  return 0; //ok
}

int stg_model_shutdown( stg_model_t* mod )
{
  if( mod->f_shutdown )
    return mod->f_shutdown(mod);
  else
    PRINT_WARN1( "model %s has no shutdown function registered", mod->token ); 
  
  //assert(mod->f_shutdown );
  
  return 0; //ok
}



//------------------------------------------------------------------------
// basic model properties


/*
void stg_model_get_pose( stg_model_t* mod, stg_pose_t* pose )
{
  assert(mod);
  assert(pose);
  memcpy(pose, &mod->pose, sizeof(mod->pose));
  
  //stg_pose_t* p;
  //size_t len = stg_model_get_property_data( mod, "pose", &p );  
  //assert( len == sizeof(stg_pose_t));
  //memcpy( pose, p, len );
}
*/


void stg_model_get_velocity( stg_model_t* mod, stg_velocity_t* dest )
{
  stg_velocity_t* v;
  assert(mod);
  assert(dest);
  v = stg_model_get_property_fixed( mod,
						    "velocity", 
						    sizeof(stg_velocity_t));
  memcpy( dest, v, sizeof(stg_velocity_t));
}


// Warning: any extant pointers to old data may become invalid
// after this function if reallocated (depending on platform implementation of 
// realloc)
int stg_model_set_velocity( stg_model_t* mod, stg_velocity_t* vel )
{
  assert(mod);
  assert(vel);
  stg_model_set_property( mod, "velocity", vel, sizeof(stg_velocity_t));
  return 0; //ok
}

void stg_model_get_geom( stg_model_t* mod, stg_geom_t* dest )
{
  stg_geom_t* g;
  assert(mod);
  assert(dest);
  g = stg_model_get_property_fixed( mod,
						"geom", 
						sizeof(stg_geom_t));
  memcpy( dest, g, sizeof(stg_geom_t));
}


stg_polygon_t* stg_model_get_polygons( stg_model_t* mod, size_t* poly_count )
{
  size_t bytes = 0;
  stg_polygon_t* polys = (stg_polygon_t*)
    stg_model_get_property( mod, "polygons", &bytes );
  
  *poly_count = bytes / sizeof(stg_polygon_t);
  return polys;
}

stg_point_t* stg_model_get_points(stg_model_t* mod, size_t* count)
{
    size_t bytes = 0;
    stg_point_t* points = (stg_point_t*) stg_model_get_property(mod, "points", &bytes);
    *count = bytes / sizeof(stg_point_t);
    return points;
}

void stg_model_set_polygons( stg_model_t* mod,
			     stg_polygon_t* polys, 
			     size_t poly_count )
{
/*
  if(poly_count > 100)
  {
    tab_cnt_for_jon = 0;
    print_debug_for_jon = TRUE;
    printf("\n\n\n\nstg_model_set_polygons(): begin\n");
    printf("poly_count: %zu", poly_count);
  }
*/
  size_t bytes = poly_count * sizeof(stg_polygon_t);
  if(!stg_model_property_exists(mod, "polygons"))
  {
    if (print_debug_for_jon)
      printf("calling stg_model_add_property_callback()\n");
    stg_model_add_property_callback(mod, "polygons", model_render_polygons, NULL);
    stg_model_add_property_callback( mod, "color", model_render_polygons, NULL );
  }

  stg_msec_t n, t;
  n = stg_timenow();
  stg_model_set_property( mod, "polygons", polys, bytes );
  t = stg_timenow();
  if (print_debug_for_jon)
    printf("stg_model_set_property() took: %lu msec\n", (t-n));


  if (print_debug_for_jon)
  {
    printf("stg_model_set_polygons(): end\n\n\n\n\n");
    print_debug_for_jon = FALSE;
  }

}

void stg_property_callback_delete_cb(gpointer item_data, gpointer user_data)
{
  stg_cbarg_t* cb_record = (stg_cbarg_t*)item_data;
  if(cb_record) free(cb_record);
}



void stg_property_destroy( stg_property_t* prop )
{

  PRINT_DEBUG2("destroying property \"%s\" of model \"%s\".", prop->name, prop->mod->token);

  // empty the list
  if( prop->callbacks )
  {
    g_list_foreach(prop->callbacks, stg_property_callback_delete_cb, prop);
    g_list_free( prop->callbacks );
  }
  
  if( prop->data )
    free( prop->data );

  free( prop );
}

void stg_property_destroy_x( stg_property_t* prop, const char *propname )
{
  stg_property_destroy(prop);
}


// call a property's callback func
void stg_property_callback_cb( gpointer data, gpointer user )
{
  stg_cbarg_t* cba = (stg_cbarg_t*)data;
  stg_property_t* prop = (stg_property_t*)user;  

  if( ((stg_property_callback_t)cba->callback)( prop->mod, prop->name, prop->data, prop->len, cba->arg ) )
    {
      // the callback returned true, which means we should remove it
      stg_model_remove_property_callback( prop->mod, prop->name, cba->callback );
    }
}

#ifdef STG_ENABLE_MODEL_LOCK
gboolean _stg_model_lock( stg_model_t* mod ) 
{
  // TODO could make this an immediate return if the world does not have threading enabled (well, mutex_init will be false anyway)
//printf("@@ Locking model %s...\n", mod->token);
  if(mod->mutex_init == FALSE || pthread_mutex_lock(&mod->mutex) == EINVAL)
    return FALSE;
  if(mod->lock_time != 0) {
    stg_print_warning("(debug) in stg_model_lock: %s lock_time is not 0! was not unlocked with stg_model_unlock last time!", mod->token);
    assert(mod->lock_time != 0);
  }
  mod->lock_time = stg_timenow();
  return TRUE;
    
} 

void _stg_model_unlock( stg_model_t* mod ) 
{ 
  stg_msec_t n, t;
  // TODO could make this a noop if the world does not have threading enabled
  if(mod->mutex_init == FALSE)
    return;
  n = stg_timenow();
  if(mod->lock_time == 0)
  {
    stg_print_warning("(debug) in stg_model_unlock: model \"%s\" unlocked without lock!", mod->token);
    assert(mod->lock_time == 0);
  }
  else if(n > mod->lock_time && (t = (n - mod->lock_time)) > STG_MODEL_MUTEX_LOCK_WARNING_TIME)
  {
    stg_print_warning("(debug) model \"%s\" was locked for %lums!", mod->token, t);
  }
  mod->lock_time = 0;
  pthread_mutex_unlock(&mod->mutex); 
}

enum stg_model_lock_status stg_model_try_lock( stg_model_t* mod )
{
  // TODO could make this a noop if the world does not have threading enabled
  if(pthread_mutex_trylock(&mod->mutex) < 0)
  {
    if(errno == EBUSY)
      return STG_LOCK_ALREADY_LOCKED;
    else
      return STG_LOCK_ERROR;
  }
  return STG_LOCK_SUCCESS;
}
#endif

/** @warning any extant pointers to old data may become invalid
 after this function if reallocated (depending on platform implementation of 
 realloc)
    @warning only use this to replace property data from a copy, do not use this
      with a pointer to the model's real property data (use stg_model_property_changed() instead).
*/
void stg_model_set_property_ex( stg_model_t* mod, 
				const char* propname, 
				void* data, 
				size_t len,
				stg_property_storage_func_t func )
{
  if (print_debug_for_jon)
  {
    tab_cnt_for_jon++;
    print_tabs_for_jon();
    printf("stg_model_set_property_ex(): begin\n");
  }

  stg_property_t* prop;
  stg_model_set_property( mod, propname, data, len );
  
  prop = (stg_property_t*) g_hash_table_lookup( mod->props, propname );
  assert(prop);
  prop->storage_func = func;

  if (print_debug_for_jon)
  {
    print_tabs_for_jon();
    tab_cnt_for_jon--;
    printf("stg_model_set_property_ex(): end\n");
  }
}

void stg_model_property_changed_ex(stg_model_t *mod, stg_property_t *prop, void *data, size_t len);

/** @warning any extant pointers to old data may become invalid
 after this function if reallocated (depending on platform implementation of 
 realloc)
    @warning only use this to replace property data from a copy, do not use this
      with a pointer to the model's real property data (use stg_model_property_changed() instead).
*/
void stg_model_set_property( stg_model_t* mod, 
			     const char* propname, 
			     void* data, 
			     size_t len )
{
  if(print_debug_for_jon)
  {
    tab_cnt_for_jon++;
    print_tabs_for_jon();
    printf("stg_model_set_property(): begin\n");
  }

  stg_property_t* prop = (stg_property_t*) g_hash_table_lookup( mod->props, propname );
  
  if( prop == NULL )
    {
      // create a new property and stash it in the model with the right name
      if(print_debug_for_jon)
      {
        print_tabs_for_jon();
        printf( "* adding model %s property %s size %d\n", mod->token, propname, (int)len );
      }
      
      
      prop = (stg_property_t*)calloc(sizeof(stg_property_t),1);      // freed in stg_property_destroy()
      strncpy( prop->name, propname, STG_PROPNAME_MAX );
      prop->mod = mod;
      
      g_hash_table_insert( mod->props, (gpointer)propname, (gpointer)prop );
      
      // test to make sure the property is there
      //find = g_hash_table_lookup( mod->props, propname );
      //assert( find );
      //assert( find == prop );      
    }
  
  stg_model_property_changed_ex(mod, prop, data, len);

  if(print_debug_for_jon)
  {
    print_tabs_for_jon();
    printf("set_model_set_property(): end\n");
    tab_cnt_for_jon--;
  }
}

void stg_model_set_all_child_properties(stg_model_t *mod, const char *prop, void *data, size_t len)
{
  stg_model_set_property(mod, prop, data, len);
  GPtrArray* children = stg_model_get_children_ptr(mod);
  size_t i;
  for(i = 0; i < children->len; ++i)
  {
    stg_model_t *c = (stg_model_t*)g_ptr_array_index(children, i);
    if(c) stg_model_set_all_child_properties(c, prop, data, len);
  }
}

void stg_model_property_changed(stg_model_t *mod, const char *propname)
{
  if(print_debug_for_jon)
  {
    tab_cnt_for_jon++;
    print_tabs_for_jon();
    printf("stg_model_property_changed(): begin\n");
  }

  size_t len;
  stg_property_t* prop;
  void *data = stg_model_get_property(mod, propname, &len);
  if(data == NULL)
    return;
  prop = g_hash_table_lookup( mod->props, propname );
  if(prop == NULL)
    return;
  stg_model_property_changed_ex(mod, prop, data, len);

  if(print_debug_for_jon)
  {
    print_tabs_for_jon();
    printf("stg_model_property_changed(): end\n");
    tab_cnt_for_jon--;
  }
}

void stg_model_property_changed_ex(stg_model_t *mod, stg_property_t *prop, void *data, size_t len)
{
  if(print_debug_for_jon)
  {
    tab_cnt_for_jon++;
    print_tabs_for_jon();
    printf("stg_model_property_changed_ex(): begin\n");
  }

  stg_msec_t n, t;
  n = stg_timenow();
  // if there's a special storage function registered, call it
  if( prop->storage_func )
  {
    if(print_debug_for_jon)
    {
      print_tabs_for_jon();
      printf( "calling special storage function for model \"%s\" property \"%s\"\n",
            prop->mod->token,
            prop->name );
    }
    prop->storage_func( prop, data, len );
  }
  else
  {
    if(print_debug_for_jon)
    {
      print_tabs_for_jon();
      printf( "calling ordinary storage function for model \"%s\" property \"%s\"\n",
            prop->mod->token,
            prop->name );
    }
    storage_ordinary( prop, data, len );
  }
  if (print_debug_for_jon)
  {
    t = stg_timenow();
    print_tabs_for_jon();
    printf("stg_model_property_changed_ex(): storage function took: %lu msec\n", (t-n));
  }
  
  if( prop->callbacks ) 
  {
    n = stg_timenow();
    if(print_debug_for_jon)
    {
      print_tabs_for_jon();
      printf( "calling callback functions for model \"%s\" property \"%s\"\n",
            prop->mod->token,
            prop->name );
    }
    g_list_foreach( prop->callbacks, stg_property_callback_cb, prop );
    if (print_debug_for_jon)
    {
      t = stg_timenow();
      print_tabs_for_jon();
      printf("stg_model_property_changed_ex(): callbacks took: %lu msec\n", (t-n));
    }
  }

  if(print_debug_for_jon)
  {
    print_tabs_for_jon();
    printf("stg_model_property_changed_ex(): end\n");
    tab_cnt_for_jon--;
  }
}


int stg_model_add_property_callback( stg_model_t* mod, 
				     const char* propname, 
				     stg_property_callback_t callback,
				     void* user )
{
  stg_property_t* prop;
  stg_cbarg_t* record;
  assert(mod);
  assert(propname);
  prop = g_hash_table_lookup( mod->props, propname );
  
  
  if( ! prop )
    {
      PRINT_WARN2( "attempting to add a callback to a nonexistent property (%s:%s)",
		   mod->token, propname );
      return 1; // error
    }
  // else
  
  record = calloc(sizeof(stg_cbarg_t),1);
  record->callback = callback;
  record->arg = user;
  
  prop->callbacks = g_list_append( prop->callbacks, record );
  
  //printf( "added callback %p data %p (%s)\n",
  //  callback, user, (char*)user );

  return 0; //ok
}

int stg_model_remove_property_callback( stg_model_t* mod, 
					const char* propname, 
					stg_property_callback_t callback )
{
  GList *el;
  stg_cbarg_t* cba = NULL;
  if(!mod || !propname) return 1;
  stg_property_t* prop = g_hash_table_lookup( mod->props, propname );
  
  if( ! prop )
    {
      PRINT_WARN2( "attempting to remove a callback from a nonexistent property (%s:%s)",
		   mod->token, propname );
      return 1; // error
    }
  
  // else

  // find our callback in the list of stg_cbarg_t
  el = NULL;
  
  // scan the list for the first matching callback
  for( el = g_list_first( prop->callbacks); 
       el;
       el = el->next )
    {
      cba = (stg_cbarg_t*)(el->data);
      if(cba->callback == callback )
	break;      
    }

  if( el ) // if we found the matching callback, remove it
  {
    prop->callbacks = g_list_remove( prop->callbacks, cba);
    free(cba);
  }
 
  return 0; //ok
}



int stg_model_clear_property_callbacks( stg_model_t* mod,
					 const char* propname )
{
  stg_property_t* prop = g_hash_table_lookup( mod->props, propname );
  
  if( ! prop )
    {
      PRINT_WARN2( "attempting to remove all callbacks from a nonexistent property (%s:%s)",
		   mod->token, propname );
      return 1; // error
    }
  
  // else
  g_list_free( prop->callbacks );
  // XXX TODO need to also free each callback record
  prop->callbacks = NULL;
  return 0; //ok
}

/// set the pose of model in global coordinates 
/** @warning any extant pointers to old data may become invalid
 after this function if reallocated (depending on platform implementation of 
 realloc)
*/
int stg_model_set_global_pose( stg_model_t* mod, stg_pose_t* gpose )
{

  if( mod->parent == NULL )
    {
      //printf( "setting pose directly\n");
      stg_model_set_property( mod, "pose", gpose, sizeof(stg_pose_t));
    }  
  else
    {
      stg_pose_t lpose;
      memcpy( &lpose, gpose, sizeof(lpose) );
      stg_model_global_to_local( mod->parent, &lpose );
      stg_model_set_property( mod, "pose", &lpose, sizeof(lpose));
    }

  //printf( "setting global pose %.2f %.2f %.2f = local pose %.2f %.2f %.2f\n",
  //      gpose->x, gpose->y, gpose->a, lpose.x, lpose.y, lpose.a );

  return 0; //ok
}


int stg_model_set_parent( stg_model_t* mod, stg_model_t* newparent)
{
  printf("Setting model %p (%s)'s parent to %p (%s)...\n", mod, mod->token, newparent, newparent?newparent->token:"null"); fflush(stdout);
  // remove the model from its old parent (if it has one)
  if( mod->parent )
    g_ptr_array_remove( mod->parent->children, mod );

  if( newparent )
    g_ptr_array_add( newparent->children, mod );

  // link from the model to its new parent
  mod->parent = newparent;

  // completely rebuild the GUI elements - it's too complex to patch up the tree herea
  gui_model_destroy( mod );
  gui_model_create( mod );
  
  // forces a redraw
  //stg_model_property_changed( mod, "polygons" );

  return 0; //ok
}


extern stg_rtk_fig_t* fig_debug_rays; 

int lines_raytrace_match( stg_model_t* mod, stg_model_t* hitmod )
{
  stg_obstacle_return_t* obs = 
    stg_model_get_property_fixed( hitmod, 
				  "obstacle_return", 
				  sizeof(stg_obstacle_return_t));
  
  // Ignore myself, my children, and my ancestors.
  if(  *obs && (!stg_model_is_related(mod,hitmod)) ) 
    return 1;
  
  return 0; // no match
}	



// Check to see if moving to the given pose would result in a collision with obstacles.
// Returns a pointer to the first entity we are in collision with, and stores
// the location of the hit in hitx,hity (if non-null)
// Returns NULL if no collisions.
// This function is useful for writing position devices.
stg_model_t* stg_model_test_collision_at_pose( stg_model_t* mod, 
					   stg_pose_t* pose, 
					   double* hitx, double* hity )
{
  //return NULL;
  
  // raytrace along all our rectangles. expensive, but most vehicles
  // will just be a single rect, grippers 3 rects, etc. not too bad.
  
  int q;
  size_t count=0;
  stg_polygon_t* polys = stg_model_get_polygons(mod, &count);

  // no body? no collision
  if( count < 1 )
    return NULL;

  if( fig_debug_rays ) stg_rtk_fig_clear( fig_debug_rays );

  // loop over all polygons
  for( q=0; q<count; q++ )
    {
      stg_polygon_t* poly = &polys[q];
      
      int point_count = poly->points->len;

      // loop over all points in this polygon
      int p;
      for( p=0; p<point_count; p++ )
	{
	  stg_point_t* pt1 = &g_array_index( poly->points, stg_point_t, p );	  
	  stg_point_t* pt2 = &g_array_index( poly->points, stg_point_t, (p+1) % point_count);
	  stg_pose_t pp1;
	  stg_pose_t pp2;
	  stg_pose_t p1;
	  stg_pose_t p2;
    itl_t* itl;
    stg_model_t* hitmod;
	  
	  pp1.x = pt1->x;
	  pp1.y = pt1->y;
	  pp1.a = 0;
	  
	  pp2.x = pt2->x;
	  pp2.y = pt2->y;
	  pp2.a = 0;
	  
	  
	  // shift the line points into the global coordinate system
	  stg_pose_sum( &p1, pose, &pp1 );
	  stg_pose_sum( &p2, pose, &pp2 );
	  
	  //printf( "tracing %.2f %.2f   %.2f %.2f\n",  p1.x, p1.y, p2.x, p2.y );
	  
    stg_matrix_lock(mod->world->matrix);
	  itl = itl_create( p1.x, p1.y, p2.x, p2.y, 
				   mod->world->matrix, 
				   PointToPoint );
	  hitmod = itl_first_matching( itl, lines_raytrace_match, mod);
    stg_matrix_unlock(mod->world->matrix);
	  
	  
	  if( hitmod )
	    {
	      if( hitx ) *hitx = itl->x; // report them
	      if( hity ) *hity = itl->y;	  
	      itl_destroy( itl );
	      return hitmod; // we hit this object! stop raytracing
	    }

	  itl_destroy( itl );
	}
    }

  return NULL;  // done 
}



int stg_model_update_pose( stg_model_t* mod )
{ 
  //PRINT_DEBUG4( "pose update model %d (vel %.2f, %.2f %.2f)", 
	//	mod->id, mod->velocity.x, mod->velocity.y, mod->velocity.a );
 
  stg_velocity_t gvel;
  stg_pose_t gpose;
  double interval, hitx = 0, hity = 0;
  stg_model_t* hitthing;
  const double epsilon = 0.00002;

  stg_model_global_velocity( mod, &gvel );
      
  stg_model_get_global_pose( mod, &gpose );

  // convert msec to sec
  interval = (double)mod->world->sim_interval / 1000.0;
  
  // compute new global position
  gpose.x += gvel.x * interval;
  gpose.y += gvel.y * interval;
  gpose.a += gvel.a * interval;


  // check this model and all it's children at the new pose
  hitthing =
    stg_model_test_collision_at_pose( mod, &gpose, &hitx, &hity );
      


  // We can set the new pose only if there was no collision. 
  // (This makes no movement happen while collided, though it has the side 
  // effect of only really stopping the model when it gets really really
  // close to an obstacle, not when it's directly touching it. To get it
  // to stop closer to the obstacle than it does would be quite a bit
  // trickier though we might want to try it someday...)

  if( hitthing )
  {
    // note only stalls when we have translational velocity. TODO make this configurable
    stg_velocity_t *vel = stg_model_get_property_fixed(mod, "velocity", sizeof(stg_velocity_t));
    if( /*only_stall_if_trans_vel &&*/ vel && (fabs(vel->x) > epsilon || fabs(vel->y) > epsilon ))
    {
      stg_velocity_t zero_v;
      PRINT_DEBUG( "HIT something immovable!" );
      mod->collision = TRUE;

      // set velocity to zero
      memset( &zero_v, 0, sizeof(zero_v));
      stg_model_set_velocity( mod, &zero_v );
    }
    else
    {
      mod->collision = FALSE;
      stg_model_set_global_pose( mod, &gpose );
    }

  }
  else	  
  {
    mod->collision = FALSE;
    stg_model_set_global_pose( mod, &gpose );
  }      

  
  return 0; // ok
}



void* stg_model_get_property( stg_model_t* mod, 
			      const char* name,
			      size_t* size )
{
  stg_property_t* prop;

  if(!mod) return NULL;
  if(!name) return NULL;
  if(!mod->props) return NULL;

  prop = g_hash_table_lookup( mod->props, name );
  if( prop )
    {
      if(size)
        *size = prop->len;      

      return prop->data;
    }
  
  if(size)
    *size = 0;
  return NULL;
}

// get a property of a known size. gets a valid pointer iff the
// property exists and contaisn data of the right size, else returns
// NULL
void* stg_model_get_property_fixed( stg_model_t* mod, 
				    const char* name,
				    size_t size )
{
  size_t actual_size=0;
  void* p = stg_model_get_property( mod, name, &actual_size );
  
  if( size == actual_size )
    return p; // right size: serve up the data
  else
  {
    //XXX replace XX PRINT_DEBUG3("requested property \"%s\" to have size %d but stored property has actual size %d.", name, size, actual_size);
    return NULL; // no data if it wasn't the right size
  }
}



// re-set the named property with a copy of it's current data.
void stg_model_property_refresh( stg_model_t* mod, const char* propname )
{
  //stg_property_t* prop = g_hash_table_lookup( mod->props, propname );
  //if( prop )
    
  size_t len=0;
  void* data = stg_model_get_property( mod, propname, &len );

  void* buf = malloc(len);
  memcpy(buf,data,len);

  stg_model_set_property( mod, propname, buf, len );

  free(buf);
}

void stg_model_set_pose(stg_model_t* mod, stg_pose_t pose)
{
  stg_model_set_property(mod, "pose", &pose, sizeof(pose));
}


void stg_model_set_pose_position(stg_model_t* mod, stg_meters_t x, stg_meters_t y)
{
  stg_pose_t pose = stg_model_get_pose(mod);
  pose.x = x;
  pose.y = y;
  stg_model_set_pose(mod, pose);
}

void stg_model_set_pose_angle(stg_model_t* mod, stg_radians_t a)
{
  stg_pose_t pose = stg_model_get_pose(mod);
  pose.a = a;
  stg_model_set_pose(mod, pose);
}

stg_pose_t stg_model_get_pose(stg_model_t* mod)
{
  stg_pose_t* pose = stg_model_get_property_fixed(mod, "pose", sizeof(stg_pose_t));
  assert(pose);
  return *pose;
}

stg_pose_t stg_model_get_initial_pose(stg_model_t *mod)
{
  stg_pose_t *p = stg_model_get_property_fixed(mod, "initial_pose", sizeof(stg_pose_t));
  assert(p);
  return *p;
}

void stg_model_reset_pose(stg_model_t *mod) 
{ 
    stg_model_set_pose( mod, stg_model_get_initial_pose(mod) ); 
}


void stg_model_set_origin(stg_model_t* mod, stg_pose_t origin)
{
  stg_geom_t geom;
  stg_geom_t* now = stg_model_get_property_fixed( mod, "geom", sizeof(stg_geom_t));
  geom.pose = origin;
  geom.size.x = now ? now->size.x : 1;
  geom.size.y = now ? now->size.y : 1;
  stg_model_set_property( mod, "geom", &geom, sizeof(geom));
}

stg_pose_t stg_model_get_origin(stg_model_t* mod)
{
  stg_geom_t* geom = stg_model_get_property_fixed(mod, "geom", sizeof(stg_geom_t));
  return geom->pose;
}

void stg_model_set_size(stg_model_t* mod, stg_size_t size)
{
  stg_geom_t newgeom;
  stg_geom_t* oldgeom = stg_model_get_property_fixed( mod, "geom", sizeof(stg_geom_t));
  newgeom.pose.x = oldgeom ? oldgeom->pose.x : 0;
  newgeom.pose.y = oldgeom ? oldgeom->pose.y : 0;
  newgeom.pose.a = oldgeom ? oldgeom->pose.a : 0;
  newgeom.size = size;
  stg_model_set_property( mod, "geom", &newgeom, sizeof(newgeom));
}

stg_size_t stg_model_get_size(stg_model_t* mod)
{
  stg_geom_t* geom = stg_model_get_property_fixed(mod, "geom", sizeof(stg_geom_t));
  return geom->size;
}

void stg_model_set_scaling(stg_model_t *mod, stg_bool_t s)
{
  mod->should_scale = s;
}

void stg_model_init_polygons(stg_model_t* mod, stg_polygon_t* polys, size_t polycount)
{
/*
  if(polycount > 100)
  {
    tab_cnt_for_jon = 0;
    print_debug_for_jon = TRUE;
    printf("\n\n\n\nstg_model_init_polygons(): begin\n");
    printf("creating %d polygons.\n", polycount);
  }
*/

  PRINT_DEBUG1("creating %d polygons.", polycount);

  stg_msec_t n, t;
  n = stg_timenow();
  stg_model_set_property_ex( mod, "polygons",  
     polys, polycount*sizeof(stg_polygon_t), storage_polygons );
  t = stg_timenow();
  if (print_debug_for_jon)
    printf("stg_model_set_property_ex() took: %lu msec\n", (t-n));

  stg_model_remove_property_callback(mod, "polygons", model_render_polygons); // in case it already existed
  stg_model_add_property_callback(mod, "polygons", model_render_polygons, NULL);
  stg_model_add_property_callback(mod, "color", model_render_points, NULL);

  n = stg_timenow();
  stg_model_property_changed( mod, "polygons" );
  t = stg_timenow();
  if (print_debug_for_jon)
    printf("stg_model_property_changed() took: %lu msec\n", (t-n));

  if (print_debug_for_jon)
  {
    printf("stg_model_init_polygons(): end\n\n\n\n\n");
    print_debug_for_jon = FALSE;
  }
}

void stg_model_init_points(stg_model_t* mod, stg_point_t* points, size_t pointCount)
{
/*
  if(pointCount > 100)
  {
    tab_cnt_for_jon = 0;
    print_debug_for_jon = TRUE;
    printf("\n\n\n\nstg_model_init_points(): begin\n");\
    printf("putting %d points in \"points\" property.\n", pointCount);
  }
*/

  PRINT_DEBUG1("putting %d points in \"points\" property.\n", pointCount);

  stg_msec_t n, t;
  n = stg_timenow();
  stg_model_set_property_ex(mod, "points", points, pointCount * sizeof(stg_point_t), storage_points);
  t = stg_timenow();
  if (print_debug_for_jon)
    printf("stg_model_set_property_ex() took: %lu msec\n", (t-n));

  stg_model_remove_property_callback(mod, "points", model_render_points); // in case it was already added, e.g. in model_load
  stg_model_add_property_callback(mod, "points", model_render_points, NULL);
  stg_model_add_property_callback(mod, "color", model_render_points, NULL);

  n = stg_timenow();
  stg_model_property_changed(mod, "points");
  t = stg_timenow();

  if (print_debug_for_jon)
    printf("stg_model_property_changed() took: %lu msec\n", (t-n));

  if (print_debug_for_jon)
  {
    printf("stg_model_init_points(): end\n\n\n\n\n");
    print_debug_for_jon = FALSE;
  }
}

void stg_model_destroy_polygons(stg_model_t* mod)
{
  size_t datalen;
  stg_polygon_t* polys = (stg_polygon_t*)stg_model_get_property(mod, "polygons", &datalen);
  size_t count = datalen/sizeof(stg_polygon_t);
  PRINT_DEBUG2("destroying %d polygons from 0x%x", count, polys);
  if(polys != NULL) 
    stg_polygons_destroy(polys, count);
}


void stg_model_load( stg_model_t* mod )
{
  assert(mod);
  //printf("loading model \"%s\"'s own unique properties from it's own section %d...\n", mod->token, mod->id);
  stg_model_load_from_worldfile_section_id(mod, mod->entity_id);
}
  

void stg_model_load_from_worldfile_section_id(stg_model_t *mod, int id)
{
  //printf("loading properties for model \"%s\" from worldfile section id %d (entitytype=%s. It should NOT be a macro id! If it is, then macroname=%s).\n", mod->token, id, wf_get_section_type(id), wf_get_macro_name(id));
  if( wf_property_exists( id, "origin" ) )
    {
      stg_pose_t origin;
      origin = stg_model_get_origin(mod);
      origin.x = wf_read_tuple_length(id, "origin", 0, origin.x);
      origin.y = wf_read_tuple_length(id, "origin", 1, origin.y);
      origin.a = wf_read_tuple_angle(id, "origin", 2, origin.a);
      stg_model_set_origin(mod, origin);
    }

  if( wf_property_exists(id, "size" ) )
    {
      stg_size_t size;
      size = stg_model_get_size(mod);
      size.x = wf_read_tuple_length(id, "size", 0, size.x);
      size.y = wf_read_tuple_length(id, "size", 1, size.y);
      stg_model_set_size(mod, size);
    }

  if( wf_property_exists( id, "pose" ))
    {
      stg_pose_t pose;
      stg_pose_t* now = 
	stg_model_get_property_fixed( mod, "pose", sizeof(stg_pose_t));
      
      pose.x = wf_read_tuple_length(id, "pose", 0, now ? now->x : 0 );
      pose.y = wf_read_tuple_length(id, "pose", 1, now ? now->y : 0 ); 
      pose.a = wf_read_tuple_angle(id, "pose", 2,  now ? now->a : 0 );
      stg_model_set_property( mod, "pose", &pose, sizeof(pose));
      stg_model_set_property( mod, "initial_pose", &pose, sizeof(pose));
    }
  
  if( wf_property_exists( id, "velocity" ))
    {
      stg_velocity_t vel;
      stg_velocity_t* now = 
	stg_model_get_property_fixed( mod, "velocity", sizeof(stg_velocity_t));
      
      vel.x = wf_read_tuple_length(id, "velocity", 0, now ? now->x : 0 );
      vel.y = wf_read_tuple_length(id, "velocity", 1, now ? now->y : 0 );
      vel.a = wf_read_tuple_angle(id, "velocity", 2,  now ? now->a : 0 );      
      stg_model_set_property( mod, "velocity", &vel, sizeof(vel) );
    }
  
  if( wf_property_exists( id, "mass" ))
    {
      stg_kg_t* now = 
	stg_model_get_property_fixed( mod, "mass", sizeof(stg_kg_t));      
      stg_kg_t mass = wf_read_float(id, "mass", now ? *now : 0 );      
      stg_model_set_property( mod, "mass", &mass, sizeof(mass) );
    }
  
#ifdef ENABLE_FIDUCIAL_MODEL
  if( wf_property_exists( id, "fiducial_return" ))
    {
      stg_fiducial_return_t* now = 
	stg_model_get_property_fixed( mod, "fiducial_return", sizeof(stg_fiducial_return_t));      
      stg_fiducial_return_t fid = 
	wf_read_int( id, "fiducial_return", now ? *now : FiducialNone );     
      stg_model_set_property( mod, "fiducial_return", &fid, sizeof(fid) );
    }
#endif
  
  if( wf_property_exists( id, "obstacle_return" ))
    {
      stg_obstacle_return_t* now = 
	stg_model_get_property_fixed( mod, "obstacle_return", sizeof(stg_obstacle_return_t));      
      stg_obstacle_return_t obs = 
	wf_read_int( id, "obstacle_return", now ? *now : 0 );
      stg_model_set_property( mod, "obstacle_return", &obs, sizeof(obs) );
    }

  if( wf_property_exists( id, "ranger_return" ))
    {
      stg_ranger_return_t* now = 
	stg_model_get_property_fixed( mod, "ranger_return", sizeof(stg_ranger_return_t));      
      
      stg_ranger_return_t rng = 
	wf_read_int( id, "ranger_return", now ? *now : 0 );
      stg_model_set_property( mod, "ranger_return", &rng, sizeof(rng));
    }
  
#ifdef ENABLE_BLOBFINDER_MODEL
  if( wf_property_exists( id, "blob_return" ))
    {
      stg_blob_return_t* now = 
	stg_model_get_property_fixed( mod, "blob_return", sizeof(stg_blob_return_t));      
      stg_blob_return_t blb = 
	wf_read_int( id, "blob_return", now ? *now : 0 );
      stg_model_set_property( mod, "blob_return", &blb, sizeof(blb));
    }
#endif
  
  if( wf_property_exists( id, "laser_return" ))
    {
      stg_laser_return_t* now = 
	stg_model_get_property_fixed( mod, "laser_return", sizeof(stg_laser_return_t));      
      stg_laser_return_t lsr = 
	wf_read_int(id, "laser_return", now ? *now : 1 );      
      stg_model_set_property( mod, "laser_return", &lsr, sizeof(lsr));
    }
  
#ifdef ENABLE_GRIPPER_MODEL
  if( wf_property_exists( id, "gripper_return" ))
    {
      stg_blob_return_t* now = 
	stg_model_get_property_fixed( mod, "gripper_return", sizeof(stg_gripper_return_t));      
      stg_gripper_return_t grp = 
	wf_read_int( id, "gripper_return", now ? *now : 0 );
      stg_model_set_property( mod, "gripper_return", &grp, sizeof(grp));
    }
#endif
  
  if( wf_property_exists( id, "boundary" ))
    {
      stg_bool_t* now = 
	stg_model_get_property_fixed( mod, "boundary", sizeof(stg_bool_t));      
      stg_bool_t bdy =  
	wf_read_int(id, "boundary", now ? *now : 0  ); 
      stg_model_set_property( mod, "boundary", &bdy, sizeof(bdy));
    }
  
  if( wf_property_exists( id, "color" ))
  {
    //stg_color_t* now = 
    //stg_model_get_property_fixed( mod, "color", sizeof(stg_color_t));      
    
    stg_color_t col = 0xFF0000; // red;  
    const char* colorstr = wf_read_string( id, "color", NULL );
    if( colorstr )
    {
      col = stg_lookup_color( colorstr );  
      stg_model_set_property( mod, "color", &col, sizeof(col));
    }
  }      

  /* Make polygon and/or read bitmap */
  {
    //size_t polydata = 0;
    stg_polygon_t* polys = NULL;
    size_t polycount = -1 ;//polydata / sizeof(stg_polygon_t);;
    int wf_polycount;
  
    const char* bitmapfile = wf_read_string( id, "bitmap", NULL );
    if( bitmapfile )
    {
      char full[STG_MAX_PATH];
      
      if( bitmapfile[0] == '/' )
        strcpy( full, bitmapfile );
      else
      {
        char *tmp = strdup(wf_get_filename());
        snprintf( full, STG_MAX_PATH,
          "%s/%s",  dirname(tmp), bitmapfile );
        free(tmp);
      }
      
#ifdef DEBUG
      printf( "attempting to load image %s\n",
	      full );
#endif
      
      polys = stg_polygons_from_image_file( full, &polycount );
      
      if( ! polys )
        PRINT_ERR1( "Failed to load polygons from image file \"%s\"", full );
    }
  
    wf_polycount = wf_read_int( id, "polygons", 0 );
    if( wf_polycount > 0 )
    {
      char key[256];
      int l;

      polycount = wf_polycount;
      //printf( "expecting %d polygons\n", polycount );
      
      polys = stg_polygons_create( polycount );
      for(l=0; l<polycount; l++ )
      {	  	  
        int pointcount;
        int p;

        snprintf(key, sizeof(key), "polygon[%d].points", l);
        pointcount = wf_read_int(id,key,0);
	  
        //printf( "expecting %d points in polygon %d\n",
        //  pointcount, l );
	  
        for( p=0; p<pointcount; p++ )
        {
          stg_point_t pt;	      

          snprintf(key, sizeof(key), "polygon[%d].point[%d]", l, p );
	      
          pt.x = wf_read_tuple_length(id, key, 0, 0);
          pt.y = wf_read_tuple_length(id, key, 1, 0);

          //printf( "key %s x: %.2f y: %.2f\n",
          //      key, pt.x, pt.y );
	      
          // append the point to the polygon
          stg_polygon_append_points( &polys[l], &pt, 1 );
        }
      }
    }
 
    // if we created any polygons
    if( polycount != -1 )
    {
      stg_model_init_polygons(mod, polys, polycount);
      free(polys);
    }
  }

  // some simple integer properties
  {
    int *now = NULL;
    int val = 0;
  
    now = stg_model_get_property_fixed( mod, "nose", sizeof(int));
    val = wf_read_int(id, "gui_nose", now ? *now : STG_DEFAULT_NOSE );  
    stg_model_set_property( mod, "nose", &val, sizeof(val) );
  
    now = stg_model_get_property_fixed( mod, "grid", sizeof(int));
    val = wf_read_int(id, "gui_grid", now ? *now : STG_DEFAULT_GRID );  
    stg_model_set_property( mod, "grid", &val, sizeof(val) );
  
    now = stg_model_get_property_fixed( mod, "mask", sizeof(int));
    val = wf_read_int(id, "gui_movemask", now ? *now : STG_DEFAULT_MASK);  
    stg_model_set_property( mod, "mask", &val, sizeof(val) );
  
    now = stg_model_get_property_fixed( mod, "outline", sizeof(int));
    val = wf_read_int(id, "gui_outline", now ? *now : STG_DEFAULT_OUTLINE);  
    stg_model_set_property( mod, "outline", &val, sizeof(val) );
  }

  // height properties (it's ok for them to be missing)
  {
    stg_meters_t val;

    if(wf_property_exists(id, "height"))
    {
      val = wf_read_float(id, "height", 0.0);
      stg_model_set_property(mod, "height", &val, sizeof(val));
    }

    if(wf_property_exists(id, "height_offset"))
    {
      val = wf_read_float(id, "height_offset", 0.0);
      stg_model_set_property(mod, "height_offset", &val, sizeof(val));
    }

    // height_in_world property will be calculated when first needed
  }

  // create user properties
  stg_model_load_user_properties(mod, id);

  // if a type-specific load callback has been set
  if( mod->f_load )
    mod->f_load( mod, id ); // call the load function
}


void stg_model_save( stg_model_t* model )
{
  //stg_pose_t* pose = 
  //  stg_model_get_property_fixed( model, "pose", sizeof(stg_pose_t));
  
  PRINT_DEBUG4( "<UNIMPLEMENTED!> saving model %s pose %.2f %.2f %.2f",
		model->token,
		pose->x,
		pose->y,
		pose->a );
  
  /* just forget all this crap - we don't use it, and it shouldn't be used by any sane coder
  // right now we only save poses
  wf_write_tuple_length( model->id, "pose", 0, pose->x);
  wf_write_tuple_length( model->id, "pose", 1, pose->y);
  wf_write_tuple_angle( model->id, "pose", 2, pose->a);
  */
}

// find the top-level model above mod;
stg_model_t* stg_model_root( stg_model_t* mod )
{
  return mod->root;
/*
  while( mod->parent )
    mod = mod->parent;
  return mod;
*/
}

int stg_model_tree_to_ptr_array( stg_model_t* root, GPtrArray* array )
{
  int added = 1;
  int ch;

  g_ptr_array_add( array, root );
  
  //printf( " added %s to array at %p\n", root->token, array );

  
  for(ch=0; ch < root->children->len; ch++ )
    {
      stg_model_t* child = g_ptr_array_index( root->children, ch );
      added += stg_model_tree_to_ptr_array( child, array );
    }
  
  return added;
}


int model_render_polygons( stg_model_t* mod, char* name, 
			   void* data, size_t polys_len, void* userp )
{

  
  stg_rtk_fig_t*  fig;
  stg_color_t *cp;
  stg_color_t col;
  size_t count;
  stg_polygon_t* polys;
  stg_geom_t geom;

  gui_lock();  // overzealous locking?

  fig = stg_model_get_fig( mod, "top" );
  assert( fig );

  stg_rtk_fig_clear( fig );
  
  cp =  stg_model_get_property_fixed( mod, "color", sizeof(stg_color_t));
  
  col = cp ? *cp : 0; // black unless we were given a color

  count = polys_len / sizeof(stg_polygon_t);
  polys = (stg_polygon_t*)data;
  
  stg_model_get_geom(mod, &geom);
  
  stg_rtk_fig_color_rgb32( fig, col );
  
  if( polys && polys_len > 0 )
  {
    int p;
    for( p=0; p<count; ++p )
    {
      const size_t npoints = polys[p].points->len;

      // poyls[p].points is a GArray* of stg_point_t objects, cast the GArray's data member to a standard array of stg_rtk_point_t objects in order to pass to RTK.  stg_rtk_point_t is identical to stg_point_t.
      stg_rtk_point_t *pts = (stg_rtk_point_t*) (polys[p].points->data);

      // previously in stage we cast stg_point_t array to array of double[2] to pass through to RTK api, but let's avoid that now; RTK api now takes stg_rtk_point_t wich we assume is identical to stg_point_t.
      //but this alternate code copies it to make sure the data is in the right
      //format:
      /*
      stg_point_t* ptsarr = (stg_point_t*) polys[p].points->data;
      double (*pts)[2];
      pts = malloc(npoints * sizeof(double[2]));
      assert(pts);
      for(int i = 0; i < npoints; ++i)
      {
        pts[i][0] = ptsarr[i].x;
        pts[i][1] = ptsarr[i].y;
      }
      */


      stg_rtk_fig_polygon( fig,
         geom.pose.x,
         geom.pose.y,
         geom.pose.a,
         npoints,
         pts,
         mod->world->win->fill_polygons 
      );


      if(mod->world->win->fill_polygons )
      {
        int *outline = stg_model_get_property_fixed( mod, "outline", sizeof(int) );
        if( outline && *outline )
        {
          stg_rtk_fig_color_rgb32( fig, 0 ); // change to black
        
          for( p=0; p<count; p++ )
            stg_rtk_fig_polygon( fig,
               geom.pose.x,
               geom.pose.y,
               geom.pose.a,
               npoints,
               pts,
               0 );

          stg_rtk_fig_color_rgb32( fig, col ); // change back
	      }
      }

      // only do this if it was allocated above and copied:
      //free (pts);

    }
  }
  
  // boundary
  {
    stg_bool_t* boundary = 
      stg_model_get_property_fixed( mod, "boundary", sizeof(stg_bool_t));
  
    if( boundary && *boundary )
    {      
      stg_rtk_fig_rectangle( fig, 
			     geom.pose.x, geom.pose.y, geom.pose.a, 
			     geom.size.x, geom.size.y, 0 ); 
    }
  }

  // nose
  {
    int* nose = 
      stg_model_get_property_fixed( mod, "nose", sizeof(int) );
    if( nose && *nose )
    {       
      // draw an arrow from the center to the front of the model
      stg_rtk_fig_arrow( fig, geom.pose.x, geom.pose.y, geom.pose.a, 
			 geom.size.x/1.25, 0.20 );
    }
  }
  
  gui_unlock();
  
  return 0;
}

int model_render_points( stg_model_t* mod, char* name, 
			   void* data, size_t len, void* userp )
{
  
  stg_rtk_fig_t*  fig;
  stg_color_t *cp;
  stg_color_t col;
  size_t count;
  stg_point_t* points;
  stg_geom_t geom;

  gui_lock();  // overzealous locking?

  fig = stg_model_get_fig( mod, "points" );
  if(!fig)
  {
    fig = stg_model_fig_create(mod, "points", "top", (mod->bg_fig)?STG_LAYER_BACKGROUND:STG_LAYER_BODY);
  }
  assert( fig ); 

  stg_rtk_fig_clear( fig );
  
  cp =  stg_model_get_property_fixed( mod, "color", sizeof(stg_color_t));
  
  col = cp ? *cp : 0; // black unless we were given a color


  count = len / sizeof(stg_point_t);
  points = (stg_point_t*)data;
  
  stg_model_get_geom(mod, &geom);
  
  stg_rtk_fig_color_rgb32( fig, col );
  
  if( points && len > 0 )
  {
    int p;
    for(p = 0; p < count; ++p)
    {
      stg_rtk_fig_point(fig, geom.pose.x + points[p].x, geom.pose.y + points[p].y);
    }
  }

  
  gui_unlock();
  
  return 0;
}

int model_render_nose( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  // nose drawing is handled by the polygon handler
  stg_model_property_changed( mod, "polygons" );
  return 0;
}

int model_handle_mask( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  int* mask;
  stg_rtk_fig_t* fig;

  assert( len == sizeof(int));
  mask = (int*)data;

  gui_lock(); // overzealous locking?
  
  fig = stg_model_get_fig(mod, "top" );
  assert(fig);
  
  stg_rtk_fig_movemask( fig, *mask);  
  
  // only install a mouse handler if the object needs one
  //(TODO can we remove mouse handlers dynamically?)
  if( *mask )    
    stg_rtk_fig_add_mouse_handler( fig, gui_model_mouse );
    
  gui_unlock();
  
  return 0;
}

int model_handle_outline( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  // outline drawing is handled by the polygon handler
  stg_model_property_changed( mod, "polygons" );
  return 0;
}
  

int model_render_grid( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{
  stg_geom_t geom;
  stg_rtk_fig_t* grid;
  int* usegrid = (int*)data;
  assert( len == sizeof(int));
  
  gui_lock(); // overzealous locking?

  stg_model_get_geom(mod, &geom);
  
  grid = stg_model_get_fig(mod,"grid"); 
  
  // if we need a grid and don't have one, make one
  if( *usegrid  )
    {    
      if( ! grid ) 
	grid = stg_model_fig_create( mod, "grid", "top", STG_LAYER_GRID); 
      
      stg_rtk_fig_clear( grid );
      stg_rtk_fig_color_rgb32( grid, stg_lookup_color(STG_GRID_MAJOR_COLOR ) );      
      stg_rtk_fig_grid( grid, 
			geom.pose.x, geom.pose.y, 
			geom.size.x, geom.size.y, 1.0  ) ;
    }
  else
    if( grid ) // if we have a grid and don't need one, clear it but keep the fig around      
      stg_rtk_fig_clear( grid );

  gui_unlock();
  
  return 0;
}

stg_model_t* stg_model_find_first_child_with_type(stg_model_t* parent, char* type)
{
  size_t i;
  stg_model_t* c;
  for(i = 0; i < parent->children->len; i++)
  {
    c = g_ptr_array_index(parent->children, i);
    //printf("** stg_model_find_first_child_with_type: found child with base type %s, comparing to desired type %s...\n", c->base_type_name, type);
    if(strcmp(c->base_type_name, type) == 0)
      return c;
  }
  return NULL;
}

stg_model_t* stg_model_find_last_child_with_type(stg_model_t* parent, char* type)
{
  size_t i;
  stg_model_t* c;
  stg_model_t* ret_c = NULL;
  for(i = 0; i < parent->children->len; i++)
  {
    c = g_ptr_array_index(parent->children, i);
    //printf("** stg_model_find_first_child_with_type: found child with base type %s, comparing to desired type %s...\n", c->base_type_name, type);
    if(strcmp(c->base_type_name, type) == 0)
      ret_c = c;
  }
  return ret_c;
}


// Debugging tool

void _model_print_children_recursive(stg_model_t* model, int indent)
{
  stg_model_t* c;
  size_t i;
  for(i = 0; i < indent; i++) printf("  ");
  printf("Model token=\"%s\" type=%s instance=%s { \n", model->token, model->base_type_name, model->instance_name);
  for(i = 0; i < model->children->len; i++)
  {
    c = g_ptr_array_index(model->children, i);
    _model_print_children_recursive(c, indent+1);
    //printf("  child %lu: token=\"%s\" type=%s\n", i, c->token, c->base_type_name);
  }
  for(i = 0; i < indent; i++) printf("  ");
  puts("}");
}

void stg_model_print_children(stg_model_t *model)
{
  _model_print_children_recursive(model, 0);
}
 
char* stg_model_get_token(stg_model_t* mod) {
  return mod->token;
}
 
char* stg_model_get_base_type_name(stg_model_t* mod) {
  return mod->base_type_name;
}

char* stg_model_get_instance_type_name(stg_model_t* mod) {
  return mod->instance_name;
}

size_t stg_model_get_instance_index(stg_model_t* mod) {
  return mod->type_instance_index;
}

// static linked list of user worldfile properties
typedef struct _stg_model_user_property_struct {
  const char* model_type;
  const char* property_name;
  stg_datatype_t datatype;
  struct _stg_model_user_property_struct* next;
} stg_model_user_property_t;
static stg_model_user_property_t* _stg_model_user_properties = NULL;
static stg_model_user_property_t* _stg_model_user_properties_tail = NULL;


// internally used when loading the world and creating models
void stg_model_load_user_properties(stg_model_t* mod, int id)
{
  int wf_section = id;
  stg_model_user_property_t* item;
  PRINT_DEBUG2("stg_model_load_user_properties: new model type = %s, id=%d", mod->base_type_name, wf_section);
  for(item = _stg_model_user_properties;
      item != NULL;
      item = item->next)
  {
    if(strcmp(item->model_type, mod->base_type_name) == 0)
    {
      PRINT_DEBUG3("\tstg_model_load_user_properties: creating property %s for %s model wf_section=%d.", item->property_name, mod->base_type_name, wf_section);
      if(wf_property_exists(wf_section, item->property_name))
      {
        switch(item->datatype)
        {
          int i;
          double d;
          float f;
          const char* s;
          case STG_INT:
            i = wf_read_int(wf_section, item->property_name, 0);
            #ifdef DEBUG_USERPROPERTIES
            printf("-> setting int user property %s on model %s with value %d.\n", item->property_name, mod->token, i);
            #endif
            stg_model_set_property(mod, item->property_name, &i, sizeof(i));
            break;
          case STG_FLOAT:
            f = wf_read_float(wf_section, item->property_name, 0.0);
            #ifdef DEBUG_USERPROPERTIES
            printf("-> setting float user property %s on model %s with value %f.\n", item->property_name, mod->token, f);
            #endif
            stg_model_set_property(mod, item->property_name, &f, sizeof(f));
            break;
          case STG_DOUBLE:
            d = wf_read_float(wf_section, item->property_name, 0.0);
            #ifdef DEBUG_USERPROPERTIES
            printf("-> setting double user property %s on model %s with value %f.\n", item->property_name, mod->token, f);
            #endif
            stg_model_set_property(mod, item->property_name, &d, sizeof(d));
            break;
          case STG_STRING:
            s = wf_read_string(wf_section, item->property_name, "");
            #ifdef DEBUG_USERPROPERTIES
            printf("-> setting char* user property %s on model %s with value \"%s\". len=%d\n", item->property_name, mod->token, s, strlen(s));
            #endif
            stg_model_set_property(mod, item->property_name, (char*)s, strlen(s)+1);	// +1 to include '\0'
            break;
        }
      }
      /*
      else
      {
        stg_print_msg("Warning: property \"%s\" not given for a new %s model.\n", 
            item->property_name, mod->base_type_name);
      }
      */
    }
  }
}

void stg_model_user_property(const char* model_type, const char* property_name, stg_datatype_t datatype)
{
  ///@bug Data allocated for user property list is never freed (this could be done by a stg_cleanup function or something).
  stg_model_user_property_t* newitem = (stg_model_user_property_t*) malloc(sizeof(stg_model_user_property_t));
  newitem->model_type = model_type;
  newitem->property_name = property_name;
  newitem->datatype = datatype;
  newitem->next = NULL;
  PRINT_DEBUG2("stg_model_user_property: registering new user property \"%s\" for model type \"%s\"...\n", property_name, model_type);
  if(_stg_model_user_properties == NULL)
  {
    _stg_model_user_properties = newitem;
    _stg_model_user_properties_tail = newitem;
  }
  else
  {
    _stg_model_user_properties_tail->next = newitem;
    _stg_model_user_properties_tail = newitem;
  }
}


void model_property_toggles_off_cb( gpointer data, gpointer userdata )
{
  assert(data);
  model_do_property_toggle((stg_property_toggle_args_t*)data, FALSE);
}
void model_property_toggles_on_cb( gpointer data, gpointer userdata )
{
  assert(data);
  model_do_property_toggle((stg_property_toggle_args_t*)data, TRUE);
}


GPtrArray* stg_model_get_children_ptr(stg_model_t* model)
{
  return model->children;
}


void _recursive_child_array_copy(gpointer childgp, gpointer destgp)
{
  stg_model_t* child_model = (stg_model_t*)childgp;
  GPtrArray* dest = (GPtrArray*)destgp;
  if(!childgp) return;
  g_ptr_array_add(dest, (gpointer)child_model);
  _recursive_child_array_copy(child_model->children, dest);
}


GPtrArray* stg_model_get_all_descendents(stg_model_t* model)
{
  GPtrArray* array = g_ptr_array_new();
#if GTK_CHECK_VERSION(2, 4, 0)
  g_ptr_array_foreach(model->children, _recursive_child_array_copy, array);
#else
  gpointer p;
  int i;
  for(i = 0; (p = g_ptr_array_index(model->children, i)) != NULL ; i++)
  {
    _recursive_child_array_copy(p, (gpointer) array);
  }
#endif
  return array;
}

void stg_model_print_all_descendents(stg_model_t* model)
{
  stg_model_t* c;
  size_t i;
  GPtrArray* all = stg_model_get_all_descendents(model);
  printf("Model token=\"%s\" type=%s\n", model->token, model->base_type_name);
  for(i = 0; i < all->len; i++)
  {
    c = (stg_model_t*)g_ptr_array_index(all, i);
    printf("  descendent %zd: token=\"%s\" type=%s\n", i, c->token, c->base_type_name);
  }
}

void stg_model_recalc_height_in_world(stg_model_t *model) 
{
  stg_meters_t *hop;
  stg_meters_t hiw;
  stg_meters_t *hp = (stg_meters_t*)stg_model_get_property_fixed(model, "height", sizeof(stg_meters_t));
  if(!hp)
  {
    stg_meters_t zh = 0.0;
    stg_model_set_property(model, "height", &zh, sizeof(stg_meters_t));
    hp = (stg_meters_t*)stg_model_get_property_fixed(model, "height", sizeof(stg_meters_t));
  }
  hop = (stg_meters_t*)stg_model_get_property_fixed(model, "height_offset", sizeof(stg_meters_t));
  if(!hop)
  {
    stg_meters_t zh = 0.0;
    stg_model_set_property(model, "height_offset", &zh, sizeof(stg_meters_t));
    hop = (stg_meters_t*)stg_model_get_property_fixed(model, "height_offset", sizeof(stg_meters_t));
  }
  hiw = *hp + *hop;
  if(model->parent)
    hiw += stg_model_get_height_in_world(model->parent);
  stg_model_set_property(model, "height_in_world", &hiw, sizeof(stg_meters_t));
}


void stg_model_set_height(stg_model_t *model, stg_meters_t h)
{
  stg_model_set_property(model, "height", &h, sizeof(stg_meters_t));
  stg_model_recalc_height_in_world(model);
}

stg_meters_t stg_model_get_height(stg_model_t *model)
{
  stg_meters_t *hp = stg_model_get_property_fixed(model, "height", sizeof(stg_meters_t));
  if(hp) return *hp;
  return 0.0;
}

stg_meters_t stg_model_get_height_in_world(stg_model_t *model)
{
  stg_meters_t *hp = (stg_meters_t*)stg_model_get_property_fixed(model, "height_in_world", sizeof(stg_meters_t));
  if(!hp) {
    stg_model_recalc_height_in_world(model);
    hp = (stg_meters_t*)stg_model_get_property_fixed(model, "height_in_world", sizeof(stg_meters_t));
  }
  return *hp;
}

/*
gboolean stg_model_has_height(stg_model_t *model)
{
  size_t s;
  return (stg_model_get_property(model, "height", &s) != NULL);
}
*/

void stg_model_set_bg_figure(stg_model_t *mod)
{
  //printf("model %s will have a background-layer figure\n", mod->token);
  mod->bg_fig = TRUE;
}

stg_world_t* stg_model_get_world(stg_model_t *mod)
{
  return mod->world;
}

