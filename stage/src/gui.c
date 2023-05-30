
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

//#define DEBUG 1
//#undef DEBUG

#include "stage_internal.h"
#include "gui.h"

#define STG_DEFAULT_WINDOW_WIDTH 700
#define STG_DEFAULT_WINDOW_HEIGHT 740

// only models that have fewer rectangles than this get matrix
// rendered when dragged
#define STG_POLY_THRESHOLD 10

// single static application visible to all funcs in this file
static stg_rtk_app_t *app = NULL; 

static GdkCursor* stg_gui_busy_cursor = NULL;
static GdkCursor* stg_gui_normal_cursor = NULL;

// some global figures anyone can draw in for debug purposes
stg_rtk_fig_t* fig_debug_rays = NULL;
stg_rtk_fig_t* fig_debug_geom = NULL;
stg_rtk_fig_t* fig_debug_matrix = NULL;
stg_rtk_fig_t* fig_trails = NULL;

int _render_matrix_deltas = FALSE;


#ifdef STG_ENABLE_GUI_LOCK
pthread_mutex_t _stg_gui_mutex;
gboolean _stg_gui_enable_locking = FALSE;
volatile gboolean _stg_gui_in_gtk_event_loop = FALSE;
#endif


/** @addtogroup stage 
    @{
*/


int stg_show_menubar = TRUE;

/** @defgroup window Window

The Stage window consists of a menu bar, a view of the simulated
world, an optional log messages pane, and a status bar.

The world view shows part of the simulated world. You can zoom the
view in and out, and scroll it to see more of the world. Simulated
robot devices, obstacles, etc., are rendered as colored polygons. The
world view can also show visualizations of the data and configuration
of various sensor and actuator models. The View menu has options to
control which data and configurations are rendered.

<h2>Worldfile Properties</h2>

Some GUI properties can be customized in the world file.

@par Summary and default values

@verbatim
window
(
  # gui properties
  center [0 0]
  size [700 740]
  scale 1.0
)
@endverbatim

"window" is not a type of model, so the common model properties do not apply here.

@par Details
- size [width:int width:int]
  - size of the window in pixels
- center [x:float y:float]
  - location of the center of the window in world coordinates (meters)
- scale [double]
  - ratio of world to pixel coordinates (window zoom)
- mouse_button_pan int
  - which mouse button pans the background in the window
- mouse_button_zoom int
  - which mouse button zooms the background in the window




<h2>Using the Stage window</h2>


<h3>Scrolling the view</h3>

<p>Left-click and drag on the background to move your view of the world.
(Unless mouse_button_pan was used to select a different mouse button.)
</p>

<h3>Zooming the view</h3>

<p>Right-click and drag on the background to zoom your view of the
world. When you press the right mouse button, a circle appears. Moving
the mouse adjusts the size of the circle; the current view is scaled
with the circle.
(Unless mouse_button_zoom was used to select a different mouse button.)
</p>

<h3>Saving the world</h3>

<P>You can save the current pose of everything in the world, using the
File/Save menu item. <b>Warning: the saved poses overwrite the current
world file.</b> Make a copy of your world file before saving if you
want to keep the old poses.


<h3>Saving a screenshot</h3>

<p> The File/Export menu allows you to export a screenshot of the
current world view in JPEG or PNG format. The frame is saved in the
current directory with filename in the format "stage-(frame
number).(jpg/png)". 

 You can also save sequences of screen shots. To start saving a
sequence, select the desired time interval from the same menu, then
select File/Export/Sequence of frames. The frames are saved in the
current directory with filenames in the format "stage-(sequence
number)-(frame number).(jpg/png)".

The frame and sequence numbers are reset to zero every time you run
Stage, so be careful to rename important frames before they are
overwritten.

<h3>Pausing and resuming the clock</h3>

<p>The Clock/Pause menu item allows you to stop the simulation clock,
freezing the world. Selecting this item again re-starts the clock.


<h3>View options</h3>

<p>The View menu allows you to toggle rendering of a 1m grid, to help
you line up objects (View/Grid). You can control whether polygons are
filled (View/Fill polygons); turning this off slightly improves
graphics performance. The rest of the view menu contains options for
rendering of data and configuration for each type of model, and a
debug menu that enables visualization of some of the innards of Stage.

*/

/** @} */

void gui_load( gui_window_t* win, int section )
{
  int window_width, window_height;
  double window_center_x, window_center_y, window_scale;

  // remember the section for saving later
  win->wf_section = section;

  window_width = 
    (int)wf_read_tuple_float(section, "size", 0, STG_DEFAULT_WINDOW_WIDTH);
  window_height = 
    (int)wf_read_tuple_float(section, "size", 1, STG_DEFAULT_WINDOW_HEIGHT);
  
  window_center_x = 
    wf_read_tuple_float(section, "center", 0, 0.0 );
  window_center_y = 
    wf_read_tuple_float(section, "center", 1, 0.0 );
  
  window_scale = 
    wf_read_float(section, "scale", 1.0 );
  
  win->fill_polygons = wf_read_int(section, "fill_polygons", 1 );
  win->show_grid = wf_read_int(section, "show_grid", 0 );

  win->canvas->mouse_button_pan = wf_read_int(section, "mouse_button_pan", win->canvas->mouse_button_pan);
  win->canvas->mouse_button_zoom = wf_read_int(section, "mouse_button_zoom", win->canvas->mouse_button_zoom);

  PRINT_DEBUG2( "window width %d height %d", window_width, window_height );
  PRINT_DEBUG2( "window center (%.2f,%.2f)", window_center_x, window_center_y );
  PRINT_DEBUG1( "window scale %.2f", window_scale );
  
  // ask the canvas to comply
  gtk_window_resize( GTK_WINDOW(win->canvas->frame), window_width, window_height );
  stg_rtk_canvas_scale( win->canvas, window_scale, window_scale );
  stg_rtk_canvas_origin( win->canvas, window_center_x, window_center_y );
}

void gui_save( gui_window_t* win )
{
  int width, height;
  gtk_window_get_size(  GTK_WINDOW(win->canvas->frame), &width, &height );
  
  wf_write_tuple_float( win->wf_section, "size", 0, width );
  wf_write_tuple_float( win->wf_section, "size", 1, height );

  wf_write_tuple_float( win->wf_section, "center", 0, win->canvas->ox );
  wf_write_tuple_float( win->wf_section, "center", 1, win->canvas->oy );

  wf_write_float( win->wf_section, "scale", win->canvas->sx );

  wf_write_int( win->wf_section, "fill_polygons", win->fill_polygons );
  wf_write_int( win->wf_section, "show_grid", win->show_grid );

  // save the flags that control visibility of data and configs
  /*
  wf_write_int(win->wf_section, "laser_data", win->render_data_flag[STG_MODEL_LASER]);
  wf_write_int(win->wf_section, "laser_config", win->render_cfg_flag[STG_MODEL_LASER]);
  wf_write_int(win->wf_section, "gripper_data", win->render_data_flag[STG_MODEL_GRIPPER]);
  wf_write_int(win->wf_section, "gripper_config", win->render_cfg_flag[STG_MODEL_GRIPPER]);
  wf_write_int(win->wf_section, "ranger_data", win->render_data_flag[STG_MODEL_RANGER]);
  wf_write_int(win->wf_section, "ranger_config", win->render_cfg_flag[STG_MODEL_RANGER]);
  wf_write_int(win->wf_section, "fiducial_data", win->render_data_flag[STG_MODEL_FIDUCIAL]);
  wf_write_int(win->wf_section, "fiducial_config", win->render_cfg_flag[STG_MODEL_FIDUCIAL]);
  wf_write_int(win->wf_section, "blobfinder_data", win->render_data_flag[STG_MODEL_BLOB]);
  wf_write_int(win->wf_section, "blobfinder_config", win->render_cfg_flag[STG_MODEL_BLOB]);
  wf_write_int(win->wf_section, "position_data", win->render_data_flag[STG_MODEL_POSITION]);
  wf_write_int(win->wf_section, "position_config", win->render_cfg_flag[STG_MODEL_POSITION]);
  wf_write_int(win->wf_section, "energy_data", win->render_data_flag[STG_MODEL_ENERGY]);
  wf_write_int(win->wf_section, "energy_config", win->render_cfg_flag[STG_MODEL_ENERGY]);
  wf_write_int(win->wf_section, "gripper_data", win->render_data_flag[STG_MODEL_GRIPPER]);
  wf_write_int(win->wf_section, "gripper_config", win->render_cfg_flag[STG_MODEL_GRIPPER]);
  */
}

void gui_startup( int* argc, char** argv[] )
{
  PRINT_DEBUG( "gui startup" );

  stg_rtk_initxx(argc, argv);
  
  app = stg_rtk_app_create();
  stg_rtk_app_main_init( app );
}

void gui_poll()
{
  STG_F()
  //PRINT_DEBUG( "gui poll" );
  gui_lock_entering_gtk_event_loop();
  if( stg_rtk_app_main_loop( app ) )
    stg_quit_request();
  gui_unlock_leaving_gtk_event_loop();
}

void gui_shutdown( void )
{
  PRINT_DEBUG( "gui shutdown" );

  if(stg_gui_busy_cursor)
  {
    gdk_cursor_destroy(stg_gui_busy_cursor);
    stg_gui_busy_cursor = NULL;
  }

  if(stg_gui_normal_cursor)
  {
    gdk_cursor_destroy(stg_gui_normal_cursor);
    stg_gui_normal_cursor = NULL;
  }

  if(app)
  {
    stg_rtk_app_main_term( app );  
    stg_rtk_app_destroy(app);
    app = NULL;
  }
}

gui_window_t* gui_window_create( stg_world_t* world, int xdim, int ydim )
{
  char txt[256];
  GString* titlestr;
  double width; //, height;
  double mark_sz;
  gui_window_t* win = calloc( sizeof(gui_window_t), 1 );

  win->canvas = stg_rtk_canvas_create( app );
  gtk_window_set_default_size( GTK_WINDOW(win->canvas->frame), xdim, ydim ); 
  
  win->world = world;
  
  // enable all objects on the canvas to find our window object
  win->canvas->userdata = (void*)win; 

  if(stg_about_info_appname)
    snprintf( txt, 256, "%s %s (Stage v%s)", stg_about_info_appname, stg_about_info_appversion?stg_about_info_appversion:"", stg_about_info_stageversion );
  else
    snprintf( txt, 256, "Stage v%s", stg_about_info_stageversion );
  gtk_statusbar_push( win->canvas->status_bar, 0, txt ); 

  //stg_rtk_canvas_size( win->canvas, xdim, ydim );
  
  titlestr = g_string_new( "Stage: " );
  g_string_append_printf( titlestr, "%s", world->token );
  
  stg_rtk_canvas_title( win->canvas, titlestr->str );
  g_string_free( titlestr, TRUE );
  
  width = 10;//world->size.x;
  //height = 10;//world->size.y;

  win->bg = stg_rtk_fig_create( win->canvas, NULL, 0, "<canvas>" );
  
  // draw a mark for the origin
  stg_rtk_fig_color_rgb32( win->bg, stg_lookup_color(STG_GRID_MAJOR_COLOR) );    
  mark_sz = 0.4;
  stg_rtk_fig_ellipse( win->bg, 0,0,0, mark_sz/3, mark_sz/3, 0 );
  stg_rtk_fig_arrow( win->bg, -mark_sz/2, 0, 0, mark_sz, mark_sz/4 );
  stg_rtk_fig_arrow( win->bg, 0, -mark_sz/2, M_PI/2, mark_sz, mark_sz/4 );
  //stg_rtk_fig_grid( win->bg, 0,0, 100.0, 100.0, 1.0 );
  
  win->show_geom = TRUE;
  win->fill_polygons = TRUE;
  win->show_polygons = TRUE;
  win->frame_interval = 500; // ms
  win->frame_format = STK_IMAGE_FORMAT_PNG;

  win->poses = stg_rtk_fig_create( win->canvas, NULL, 0, "<win.poses>" );

  
  // start in the center, fully zoomed out
  stg_rtk_canvas_scale( win->canvas, 1.1*width/xdim, 1.1*width/xdim );
  //stg_rtk_canvas_origin( win->canvas, width/2.0, height/2.0 );

  win->selection_active = NULL;
  
  // show the limits of the world
  gui_window_draw_world_extent(win, world->width, world->height);
 
  if(stg_show_menubar)
    gui_window_menus_create( win );
 
  return win;
}

void gui_window_draw_world_extent(gui_window_t* win, stg_meters_t width, stg_meters_t height) 
{
  // recreate bg figure
  /*
  stg_rtk_canvas_bgcolor( win->canvas, 0.9, 0.9, 0.9 ); // grey background
  stg_rtk_fig_color_rgb32( win->bg, 0xE9E9E9 ); // grey old world
  stg_rtk_fig_rectangle( win->bg, 0,0,0, win->bg->sx, win->bg->sy, 1 );
  */
  stg_rtk_fig_clear(win->bg);
  stg_rtk_canvas_bgcolor( win->canvas, 0.9, 0.9, 0.9 ); // grey background
  stg_rtk_fig_color_rgb32( win->bg, 0xFFFFFF ); // white world size
  stg_rtk_fig_rectangle( win->bg, 0,0,0, width, height, 1 );
  stg_rtk_fig_dirty(win->bg);
}

void gui_window_destroy( gui_window_t* win )
{
  PRINT_DEBUG( "gui window destroy" );

  //g_hash_table_destroy( win->guimods );

  stg_rtk_canvas_destroy( win->canvas );
  stg_rtk_fig_destroy( win->bg );
  stg_rtk_fig_destroy( win->poses );
}

gui_window_t* gui_world_create( stg_world_t* world )
{
  gui_window_t* win;
  PRINT_DEBUG( "gui world create" );
  
  win = gui_window_create( world, 
					 STG_DEFAULT_WINDOW_WIDTH,
					 STG_DEFAULT_WINDOW_HEIGHT );
  
  // show the window
  gtk_widget_show_all(win->canvas->frame);

  return win;
}


void gui_world_render_cell( stg_rtk_fig_t* fig, stg_cell_t* cell )
{
  assert( fig );
  assert( cell );
  
  stg_rtk_fig_rectangle( fig,
			 cell->x, cell->y, 0.0,
			 cell->size, cell->size, 0 );
  
  // if this is a leaf node containing data, draw a little cross
  if( 0 )//cell->data && g_slist_length( (GSList*)cell->data ) )
    {
      stg_rtk_fig_line( fig, 
			cell->x-0.01, cell->y-0.01,
			cell->x+0.01, cell->y+0.01 );

      stg_rtk_fig_line( fig, 
			cell->x-0.01, cell->y+0.01,
			cell->x+0.01, cell->y-0.01 );
    }
  
  if( cell->children[0] )
    {
      int i;
      for( i=0; i<4; i++ )
	gui_world_render_cell( fig, cell->children[i] );  
    }
}

void gui_world_render_cell_occupied( stg_rtk_fig_t* fig, stg_cell_t* cell )
{
  assert( fig );
  assert( cell );
  
  // if this is a leaf node containing data, draw a 
  if( cell->data && g_slist_length( (GSList*)cell->data ) )
    stg_rtk_fig_rectangle( fig,
			   cell->x, cell->y, 0.0,
			   cell->size, cell->size, 0 );            
  
  if( cell->children[0] )
    {
      int i;
      for( i=0; i<4; i++ )
	gui_world_render_cell_occupied( fig, cell->children[i] );  
    }
}

void gui_world_render_cell_cb( gpointer cell, gpointer fig )
{
  gui_world_render_cell( (stg_rtk_fig_t*)fig, (stg_cell_t*)cell );
}


void render_matrix_object( gpointer key, gpointer value, gpointer user )
{
  STG_F()
  // value is a list of cells
  GSList* list = (GSList*)value;
  
  while( list )
    {
      stg_cell_t* cell = (stg_cell_t*)list->data;
      stg_rtk_fig_rectangle( (stg_rtk_fig_t*)user,
			     cell->x, cell->y, 0.0,
			     cell->size, cell->size, 0 );
      
      list = list->next;
    }
}

// useful debug function allows plotting the matrix
void gui_world_matrix_table( stg_world_t* world, gui_window_t* win )
{
  if( win->matrix )
    {
      stg_matrix_lock(world->matrix);
      if( world->matrix->ptable )
	g_hash_table_foreach( world->matrix->ptable, 
			  render_matrix_object, 
			      win->matrix );   
      stg_matrix_unlock(world->matrix);
    }
}


void gui_pose( stg_rtk_fig_t* fig, stg_model_t* mod )
{
  stg_pose_t* pose = 
    stg_model_get_property_fixed( mod, "pose", sizeof(stg_pose_t));
  
  if( pose )
    stg_rtk_fig_arrow_ex( fig, 0,0, pose->x, pose->y, 0.05 );
}


void gui_pose_cb( gpointer key, gpointer value, gpointer user )
{
  gui_pose( (stg_rtk_fig_t*)user, (stg_model_t*)value );
}

// returns 1 if the world should be destroyed
int gui_world_update( stg_world_t* world )
{
  STG_F()
  //PRINT_DEBUG( "gui world update" );
  
  char clock_str[256];
  char status_str[256];
  char timeavg_str[256];
  gui_window_t* win = world->win;
  //static double fraction_avg = 1.0; 

  gui_lock();
  
  if( stg_rtk_canvas_isclosed( win->canvas ) )
    {
      //puts( "<window closed>" );
      //stg_world_destroy( world );  -> the user should do this I think? (rh)
      return 1;
    }

  if( win->matrix )
    {
      stg_rtk_fig_clear( win->matrix );
      stg_matrix_lock(world->matrix);
      gui_world_render_cell_occupied( win->matrix, world->matrix->root );
      stg_matrix_unlock(world->matrix);
      //gui_world_matrix_table( world, win );
    }  
  
  if( win->matrix_tree )
    {
      stg_rtk_fig_clear( win->matrix_tree );
      stg_matrix_lock(world->matrix);
      gui_world_render_cell( win->matrix_tree, world->matrix->root );
      stg_matrix_unlock(world->matrix);
    }  


  // calculate smoothed average of how closely to desired interval we are doing (weight current average over last measured time)
  //fraction_avg = fraction_avg * 0.95 +  (double)world->sim_interval / (double)world->real_interval_measured * 0.05;
  snprintf( clock_str, 255, "Time: %lu:%lu:%02lu:%02lu.%03lu",
    world->sim_time / (24*3600000), // days
    world->sim_time / 3600000, // hours
    (world->sim_time % 3600000) / 60000, // minutes
    (world->sim_time % 60000) / 1000, // seconds
    world->sim_time % 1000 // milliseconds
  );
  //printf("avg ratio update %lu = %f (sim=%f, real=%f)\n", foo++, world->avg_interval_ratio, (double)(world->sim_interval), (double)(world->real_interval_measured));
  const double r = (double)(world->real_interval_measured);
  const double s = (double)(world->sim_interval);
  if(r != 0 && s != 0) world->avg_interval_ratio = (world->avg_interval_ratio * 0.80) +  ( (s/i) * 0.20 );
#ifdef DEBUG
  snprintf(timeavg_str, 225, "(inst. sim/real time: %2.2f, avg: %2.2f, sim_int=%lu, desired real_int=%lu, real_meas=%lu)",
      (double)world->sim_interval/(double)world->real_interval_measured,
      world->avg_interval_ratio, world->sim_interval, world->real_interval, world->real_interval_measured);
#else
  //snprintf(timeavg_str, 225, "(avg sim/real time: %2.2f, sim_int=%lu, real_meas=%lu)", world->avg_interval_ratio, world->sim_interval, world->real_interval_measured);
  snprintf(timeavg_str, 225, "(avg sim/real time: %2.2f)", world->avg_interval_ratio);
#endif
  snprintf( status_str, 255, "%d active devices %s", world->subs, world->paused ? "--PAUSED--" : "" );

  
  gtk_label_set_text( win->canvas->clock_label, clock_str );
  gtk_label_set_text( win->canvas->timeavg_label, timeavg_str );
  gtk_label_set_text( win->canvas->status_label, status_str );
  
  if(world->avg_interval_ratio < 0.7)
  {
    win->canvas->status_bar_blinker.active = 1;
    if(stg_blinker_update(&win->canvas->status_bar_blinker) == 1)
    {
      gtk_widget_modify_fg(GTK_WIDGET(win->canvas->timeavg_label), GTK_STATE_NORMAL, &win->canvas->status_bar_alert_color);
    }
    else
    {
      gtk_widget_modify_fg(GTK_WIDGET(win->canvas->timeavg_label), GTK_STATE_NORMAL, NULL);
    }
  }
  else
  {
    if(win->canvas->status_bar_blinker.active)
    {
      gtk_widget_modify_fg(GTK_WIDGET(win->canvas->timeavg_label), GTK_STATE_NORMAL, NULL);
      win->canvas->status_bar_blinker.active = 0;
    }
  }

  // smooth the performance avg a little
/*   static double fraction_avg = 1.0; */
/*   fraction_avg = fraction_avg * 0.9 + */
/*     (double)world->wall_interval / (double)world->real_interval_measured * 0.1; */
/*   gtk_progress_bar_set_fraction( win->canvas->perf_bar, fraction_avg ); */
  
/*   static double rt_avg = 1.0; */
/*   rt_avg = rt_avg * 0.9 + */
/*     (double)world->real_interval_measured / */
/*     (double)world->sim_interval * 0.1; */
/*   gtk_progress_bar_set_fraction( win->canvas->rt_bar,  */
/* 				 rt_avg ); */
  
  if( win->show_geom )
    gui_world_geom( world );

  {
    static int trail_interval = 5;
  
    if( fig_trails )
      if( trail_interval++ > 4 )
      {
        void gui_world_trails(stg_world_t *); // forward declaration
        gui_world_trails( world );
        trail_interval = 0;
      }
  }
  
  if( win->selection_active )
    gui_model_display_pose( win->selection_active, "Selection:" ); 
  


  if(world->win->show_polygons)  // no need no rerender if polygons disabled
    stg_rtk_canvas_render( win->canvas );      

  gui_unlock();
  return 0;
}

void gui_world_destroy( stg_world_t* world )
{
  PRINT_DEBUG( "gui world destroy" );
  
  if( world->win && world->win->canvas ) 
    stg_rtk_canvas_destroy( world->win->canvas );
  else
    PRINT_WARN1( "can't find a window for world %d", world->id );
}


// draw trails behing moving models
void gui_model_trail( stg_model_t* mod )
{ 
  stg_color_t* col;
  stg_rtk_fig_t* spot;
  stg_geom_t* geom ;
  stg_pose_t bbox_pose;

  assert( fig_trails );

  if( mod->parent ) // only trail top-level objects
    return; 

  col = 
    stg_model_get_property_fixed( mod, "color", sizeof(stg_color_t));
  
  spot = stg_rtk_fig_create( mod->world->win->canvas,
					    fig_trails, STG_LAYER_BODY-1, "<trails.spot>" );      
  
  stg_rtk_fig_color_rgb32( spot, *col );
  
  geom = 
    stg_model_get_property_fixed( mod, "geom", sizeof(stg_geom_t));
  
  // draw the bounding box
  memcpy( &bbox_pose, &geom->pose, sizeof(bbox_pose));
  stg_model_local_to_global( mod, &bbox_pose );
  stg_rtk_fig_rectangle( spot, 
			 bbox_pose.x, bbox_pose.y, bbox_pose.a, 
			 geom->size.x,
			 geom->size.y, 0 );  
}


// wrapper 
void gui_model_trail_cb( gpointer key, gpointer value, gpointer user )
{
  gui_model_trail( (stg_model_t*)value );
}

// render a trail cell for all models
void gui_world_trails( stg_world_t* world )
{
  assert( fig_trails );
  g_hash_table_foreach( world->models, gui_model_trail_cb, NULL ); 
}



const char* gui_model_describe(  stg_model_t* mod )
{
  static char txt[256];
  
  stg_pose_t* pose = 
    stg_model_get_property_fixed( mod, "pose", sizeof(stg_pose_t));
  
  snprintf(txt, sizeof(txt), "model \"%s\" pose: [%.2fm,%.2fm,%.2fdeg]",  
	   //stg_model_type_string(mod->type), 
	   mod->token, 
	   //mod->world->id, 
	   //mod->id,  
	   pose->x, pose->y, RTOD(pose->a)  );
  
  return txt;
}


void gui_model_display_pose( stg_model_t* mod, char* verb )
{
  char txt[256];
  guint cid;
  gui_window_t* win = mod->world->win;

  // display the pose
  snprintf(txt, sizeof(txt), "%s %s", 
	   verb, gui_model_describe(mod)); 
  cid = gtk_statusbar_get_context_id( win->canvas->status_bar, "on_mouse" );
  gtk_statusbar_pop( win->canvas->status_bar, cid ); 
  gtk_statusbar_push( win->canvas->status_bar, cid, txt ); 
  //printf( "STATUSBAR: %s\n", txt );
}

// Process mouse events 
void gui_model_mouse(stg_rtk_fig_t *fig, int event, int mode)
{
  STG_F()
  //PRINT_DEBUG2( "ON MOUSE CALLED BACK for %p with userdata %p", fig, fig->userdata );
    // each fig links back to the Entity that owns it
  stg_model_t* mod = (stg_model_t*)fig->userdata;
  if(!mod)
    return;
  if(!mod->world)
    return;
  gui_window_t* win = mod->world->win;
  if(!win)
    return;
  guint cid=0;
  static stg_velocity_t capture_vel;
  stg_pose_t pose;  
  stg_velocity_t zero_vel;

  memset( &capture_vel, 0, sizeof(capture_vel) );
  memset( &zero_vel, 0, sizeof(zero_vel) );

  switch (event)
    {
    case STK_EVENT_MOUSE_OVER:
      win->selection_active = mod;      
      break;

    case STK_EVENT_MOUSE_NOT_OVER:
      win->selection_active = NULL;      
      
      // get rid of the selection info
      cid = gtk_statusbar_get_context_id( win->canvas->status_bar, "on_mouse" );
      gtk_statusbar_pop( win->canvas->status_bar, cid ); 
      break;
      
    case STK_EVENT_PRESS:
      {
	// store the velocity at which we grabbed the model (if it has a velocity)
	stg_velocity_t* v = stg_model_get_property_fixed( mod, "velocity", sizeof(stg_velocity_t));
	if( v )
	  {
	    memcpy( &capture_vel, v, sizeof(capture_vel));
	    stg_model_set_property( mod, "velocity", &zero_vel, sizeof(zero_vel));
	  }
      }
      // DELIBERATE NO-BREAK      

    case STK_EVENT_MOTION:       
      // move the object to follow the mouse
      stg_rtk_fig_get_origin(fig, &pose.x, &pose.y, &pose.a );
      
      // TODO - if there are more motion events pending, do nothing.
      //if( !gtk_events_pending() )
      
      // only update simple objects on drag
      //if( mod->polygons->len < STG_POLY_THRESHOLD )
      //stg_model_set_pose( mod, &pose );

      stg_model_set_property( mod, "pose", &pose, sizeof(pose));
      
      // display the pose
      //gui_model_display_pose( mod, "Dragging:" );
      break;
      
    case STK_EVENT_RELEASE:
      // move the entity to its final position
      stg_rtk_fig_get_origin(fig, &pose.x, &pose.y, &pose.a );
      //stg_model_set_pose( mod, &pose );
      stg_model_set_property( mod, "pose", &pose, sizeof(pose));
      
      // and restore the velocity at which we grabbed it
      stg_model_set_property( mod, "velocity", &capture_vel, sizeof(capture_vel) );
      break;      
      
    default:
      break;
    }

  return;
}

int stg_fig_clear_cb(  stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp )
{
  STG_F()
  PRINT_DEBUG1( "clear CB %p\n", userp );
  
  if( userp )
    stg_rtk_fig_clear((stg_rtk_fig_t*)userp);
  
  return 1; // cancels callback
}



void _gui_model_remove_fig_reference(stg_rtk_fig_t* fig)
{
  STG_F()
  stg_model_t* mod = (stg_model_t*)(fig->userdata);
  PRINT_DEBUG2("_gui_model_remove_fig_reference: removing fig %s from %s's datalist.\n", fig->name, mod->token);
  g_datalist_remove_data(&mod->figs, fig->name);
}

void gui_model_create( stg_model_t* mod )
{
  STG_F()
  //gui_window_t* win;
  stg_rtk_fig_t* parent = NULL;
  stg_rtk_fig_t* top = NULL;

  if( mod->parent )
    parent = stg_model_get_fig( mod->parent, "top" );
  else
    parent = mod->world->win->bg;

  assert(parent);
  
  top = stg_rtk_fig_create(  mod->world->win->canvas, parent, (mod->bg_fig)?STG_LAYER_BACKGROUND:STG_LAYER_BODY, "top" );
  PRINT_DEBUG3("created top model fig (%p) for model %s (parent fig %p).\n", top, mod->token, parent);
  
  top->userdata = mod;
  top->username = mod->token;
  top->predeletefn = &_gui_model_remove_fig_reference;
		     
  // install the figure 
  //printf("adding model figure to top mod %p (figs %p top %p) token %s...", mod, mod->figs, top, mod->token); fflush(stdout);
  g_datalist_init(&(mod->figs));
  g_datalist_set_data( &(mod->figs), "top", top ); 
}

void gui_model_destroy( stg_model_t* mod )
{
  STG_F()
  stg_rtk_fig_t *fig;

  PRINT_DEBUG1("destroying gui components for model %s", mod->token);

  gui_window_t *win = mod->world->win;
  if(win && win->selection_active == mod)
    win->selection_active = NULL;
  
  fig = stg_model_get_fig( mod, "top" );
  //assert(fig);
  if( fig ) 
    //stg_rtk_fig_destroy(fig); // rh
    stg_rtk_fig_and_descendents_destroy( fig );

  /// XXX is thi safe to do after stg_rtk_fig_..._destroy ?? 
  stg_model_fig_clear(mod, "top");
  g_datalist_clear(&mod->figs);
}


/// render a model's global pose vector
void gui_model_render_geom_global( stg_model_t* mod, stg_rtk_fig_t* fig )
{
  stg_pose_t glob;
  stg_geom_t geom;
  stg_pose_t localpose;
  stg_pose_t bbox_pose;
  stg_model_get_global_pose( mod, &glob );
  
  stg_rtk_fig_color_rgb32( fig, 0x0000FF ); // blue
  //stg_rtk_fig_line( fig, 0, 0, glob.x, glob.y );
  stg_rtk_fig_arrow( fig, glob.x, glob.y, glob.a, 0.15, 0.05 );  
  
  if( mod->parent )
    {
      stg_pose_t parentpose;
      stg_model_get_global_pose( mod->parent, &parentpose );
      stg_rtk_fig_line( fig, parentpose.x, parentpose.y, glob.x, parentpose.y );
      stg_rtk_fig_line( fig, glob.x, parentpose.y, glob.x, glob.y );
    }
  else
    {
      stg_rtk_fig_line( fig, 0, 0, glob.x, 0 );
      stg_rtk_fig_line( fig, glob.x, 0, glob.x, glob.y );
    }
  
  stg_model_get_geom( mod, &geom );
  
  memcpy( &localpose, &geom.pose, sizeof(localpose));
  stg_model_local_to_global( mod, &localpose );
  
  // draw the local offset
  stg_rtk_fig_line( fig, 
		glob.x, glob.y, 
		localpose.x, glob.y );

  stg_rtk_fig_line( fig, 
		localpose.x, glob.y, 
		localpose.x, localpose.y );
  
  // draw the bounding box
  memcpy( &bbox_pose, &geom.pose, sizeof(bbox_pose));
  stg_model_local_to_global( mod, &bbox_pose );
  stg_rtk_fig_rectangle( fig, 
		     bbox_pose.x, bbox_pose.y, bbox_pose.a, 
		     geom.size.x,
		     geom.size.y, 0 );  
}

/// move a model's figure to the model's current location
void gui_model_move( stg_model_t* mod )
{ 
  STG_F()
  stg_pose_t* pose = 
    stg_model_get_property_fixed( mod, "pose", sizeof(stg_pose_t));
  
  if( pose )
    stg_rtk_fig_origin( stg_model_get_fig(mod,"top"), 
			pose->x, pose->y, pose->a );   
}


///  render a model's geometry if geom viewing is enabled
void gui_model_render_geom( stg_model_t* mod )
{
  STG_F()
  if( fig_debug_geom )
    gui_model_render_geom_global( mod, fig_debug_geom ); 
}

/// wrapper for gui_model_render_geom for use in callbacks
void gui_model_render_geom_cb( gpointer key, gpointer value, gpointer user )
{
  STG_F()
  gui_model_render_geom( (stg_model_t*)value );
}

/// render the geometry of all models
void gui_world_geom( stg_world_t* world )
{
  STG_F()
  if( fig_debug_geom )
    {
      stg_rtk_fig_clear( fig_debug_geom );
      g_hash_table_foreach( world->models, gui_model_render_geom_cb, NULL ); 
    }
}


void stg_model_fig_clear( stg_model_t* mod, const char* figname )
{
  STG_F()
  stg_rtk_fig_t* fig = stg_model_get_fig( mod, figname );
  if( fig )
    stg_rtk_fig_clear( fig );
}

int stg_model_fig_clear_cb( stg_model_t* mod, void* data, size_t len, 
			    void* userp )
{
  STG_F()
  PRINT_DEBUG2("figure %s of model %s.\n", (char*)userp, mod->token);
  if( userp )
    stg_model_fig_clear( mod, (char*)userp );
  return 1;  // remove callback
}


stg_rtk_fig_t* stg_model_get_fig( stg_model_t* mod, const char* figname ) 
{
  return( (stg_rtk_fig_t*)g_datalist_get_data( &mod->figs, figname ));  
}

stg_rtk_fig_t* stg_model_fig_create( stg_model_t* mod, 
				     const char* figname, 
				     const char* parentname, 
				     int layer )
{
  STG_F()
  stg_rtk_fig_t* fig;
  stg_rtk_fig_t* parent = NULL;
  
  if( parentname )
    parent = stg_model_get_fig( mod, parentname );
  
  fig = stg_rtk_fig_create( mod->world->win->canvas, parent, layer, figname );  
  fig->userdata = (void*)mod;
  fig->username = mod->token;
  PRINT_DEBUG3("created fig %s (%p) for model %s.\n", figname, fig, mod->token);
  g_datalist_set_data( &mod->figs, figname, (void*)fig );

  return fig;
}

void gui_window_set_title(gui_window_t* win, const char* str)
{
  STG_F()
  stg_rtk_canvas_title(win->canvas, str);
}



void gui_window_set_cursor_busy(gui_window_t* win)
{
  STG_F()
  if(!stg_gui_busy_cursor)
    stg_gui_busy_cursor = gdk_cursor_new(GDK_WATCH);
  //gdk_window_set_cursor(gtk_widget_get_root_window(win->canvas->frame), gdk_cursor_new(GDK_WATCH));
  gdk_window_set_cursor(win->canvas->frame->window, stg_gui_busy_cursor);
}

void gui_window_set_cursor_normal(gui_window_t* win)
{
  STG_F()
  ///@bug Should remember what the cursor was before changing it to busy -- but there is no gdk_window_get_cursor!
  if(!stg_gui_normal_cursor)
    stg_gui_normal_cursor = gdk_cursor_new(GDK_LEFT_PTR);
  //gdk_window_set_cursor(gtk_widget_get_root_window(win->canvas->frame), gdk_cursor_new(GDK_LEFT_PTR));
  gdk_window_set_cursor(win->canvas->frame->window, stg_gui_normal_cursor);
}

void stg_gui_fatal_error_dialog(const char* primary_text, const char* secondary_text, int exit_code, gboolean continue_button)
{
  stg_rtk_fatal_error_dialog(primary_text, secondary_text, NULL, exit_code, continue_button);
}

void stg_gui_error_dialog(const char* primary_text, const char* secondary_text, int exit_code)
{
  stg_gui_fatal_error_dialog(primary_text, secondary_text, exit_code, TRUE);
}

void gui_window_request_minimize(gui_window_t* win)
{
  gtk_window_iconify( GTK_WINDOW(win->canvas->frame) );
}

void gui_window_request_maximize(gui_window_t* win)
{
  gtk_window_maximize( GTK_WINDOW(win->canvas->frame) );
}

void gui_window_request_fullscreen(gui_window_t* win)
{
#if GTK_CHECK_VERSION(2, 2, 0)
  gtk_window_fullscreen( GTK_WINDOW(win->canvas->frame) );
#else
  gtk_window_set_decorated(win->canvas->frame, 0);
  gtk_window_maximize(GTK_WINDOW(win->canvas->frame));
#endif
}

void gui_window_get_geometry(gui_window_t* win, int *x, int *y, int *width, int *height)
{
  gtk_window_get_position(GTK_WINDOW(win->canvas->frame), x, y);
  gtk_window_get_size(GTK_WINDOW(win->canvas->frame), width, height);
}

#ifdef STG_ENABLE_GUI_LOCK
void gui_enable_lock(gboolean enable)
{
  if(enable)
    pthread_mutex_init(&_stg_gui_mutex, NULL);
  _stg_gui_enable_locking = enable;
}
#endif


