#ifndef _STAGE_INTERNAL_H
#define _STAGE_INTERNAL_H

// internal function declarations that are not part of the external
// interface to Stage

#include "stage.h"
#include "math.h" // for lrint() in macros


/** 
    @ingroup libstage
    @defgroup libstage_internal Internals - not intended for the libstage end user 
    @{ 
*/


#ifdef __cplusplus
extern "C" {
#endif 
  
  /** Defines the GUI window */
  typedef struct 
  {
    stg_rtk_canvas_t* canvas;
    
    stg_world_t* world; // every window shows a single world
    
    // stg_rtk doesn't support status bars, so we'll use gtk directly
    GtkStatusbar* statusbar;
    GtkLabel* timelabel;
    
    int wf_section; // worldfile section for load/save
    
    stg_rtk_fig_t* bg; // background
    stg_rtk_fig_t* matrix;
    stg_rtk_fig_t* matrix_tree;
    stg_rtk_fig_t* poses;

    // Flags set by the menus:
    gboolean show_matrix;  
    gboolean show_geom;
    gboolean fill_polygons;
    gboolean show_grid;      
    gboolean show_polygons;  // A blanket "draw nothing" flag, if false

    int frame_series;
    int frame_index;
    int frame_callback_tag;
    int frame_interval;
    int frame_format;

    stg_model_t* selection_active;

#if GTK_CHECK_VERSION(2, 4, 0)
    GtkActionGroup* menu_action_group;
#else
    GtkItemFactory* menu_item_factory;
#endif
    
  } gui_window_t;

  void gui_startup( int* argc, char** argv[] ); 
  void gui_poll( void );
  void gui_shutdown( void );

  void gui_load( gui_window_t* win, int section );
  void gui_save( gui_window_t* win );
  void gui_enable_load_file_menu_item(gui_window_t* win, gboolean enable);
  
  gui_window_t* gui_world_create( stg_world_t* world );
  void gui_world_destroy( stg_world_t* world );
  int gui_world_update( stg_world_t* world );
  void gui_world_geom( stg_world_t* world );
  void stg_world_reload( stg_world_t* world );
  void stg_world_load_file_act(stg_world_t* world);

  void gui_model_create( stg_model_t* model );
  void gui_model_destroy( stg_model_t* model );
  void gui_model_display_pose( stg_model_t* mod, char* verb );
  void gui_model_features( stg_model_t* mod );
  void gui_model_geom( stg_model_t* model );
  void gui_model_mouse(stg_rtk_fig_t *fig, int event, int mode);
  void gui_model_move( stg_model_t* mod );
  void gui_model_nose( stg_model_t* model );
  void gui_model_polygons( stg_model_t* model );
  void gui_model_render_command( stg_model_t* mod );
  void gui_model_render_config( stg_model_t* mod );
  //void gui_model_render_data( stg_model_t* mod );
  void gui_window_menus_create( gui_window_t* win );
  void gui_window_menus_destroy( gui_window_t* win );
  void gui_window_set_title(gui_window_t* win, const char* str);
  void gui_window_set_cursor_busy(gui_window_t* win);
  void gui_window_set_cursor_normal(gui_window_t* win);
  void gui_window_request_minimize(gui_window_t* win);
  void gui_window_request_maximize(gui_window_t* win);
  void gui_window_request_fullscreen(gui_window_t* win);
  void gui_window_get_geometry(gui_window_t* win, int *x, int *y, int *width, int *height);

  void gui_add_view_item( const gchar *name,
			  const gchar *label,
			  const gchar *tooltip,
			  GCallback callback,
			  gboolean  is_active,
			  void* userdata );

  /** Indicate the extent of the world (@a width and @a height) by setting the
      window background to grey and drawing the world extents as a white rectangle.
  */
  void gui_window_draw_world_extent(gui_window_t* win, stg_meters_t width, stg_meters_t height);

  // rtv experimental
  void gui_add_tree_item(stg_model_t* mod);
  void gui_remove_tree_item(stg_model_t *mod);

  void gui_sync_menu_toggle_items(gui_window_t* win);

#ifdef STG_ENABLE_GUI_LOCK
  void gui_enable_lock(gboolean enable);
#else
#define gui_enable_lock(e)
#endif


  /*** This is a hack to protect data shared by GTK and Stage via glib-- mainly 
   *   matrix cell data lists. Really we ought to make an API layer for accesing
   *   cell data, which locks the world, maybe? The problem with that is that
   *   we can't just lock during gtk_main_iteration(), since both GTK internal
   *   stuff, and our callbacks, might need to access the shared data.
   *   As it is, calls to "gui_lock()" are sprinkled all over the place, as I
   *   deemed neccesary (and may be some bugs- crashes or deadlocks!)
   *   Mabye it could be part of RTK.
   *   - reed
   */

#ifdef STG_ENABLE_GUI_LOCK
  extern pthread_mutex_t _stg_gui_mutex;
  extern gboolean _stg_gui_enable_locking;
  extern volatile gboolean _stg_gui_in_gtk_event_loop;
  #define gui_lock() \
  { \
    if(_stg_gui_enable_locking && !_stg_gui_in_gtk_event_loop) \
    { \
      /*printf("+++ gui_lock @ %s (%s:%d) +++\n", __PRETTY_FUNCTION__, __FILE__, __LINE__);*/ \
      pthread_mutex_lock(&_stg_gui_mutex); \
      gdk_threads_enter(); \
    } \
  }

  #define gui_lock_entering_gtk_event_loop() { \
    if(_stg_gui_enable_locking) { \
      /*printf("+++++ gui_lock_entering_gtk @ %s (%s:%d) ++++++\n", __PRETTY_FUNCTION__, __FILE__, __LINE__);*/ \
      pthread_mutex_lock(&_stg_gui_mutex); \
      _stg_gui_in_gtk_event_loop = TRUE; \
      gdk_threads_enter(); \
    }\
  }

  #define gui_unlock() \
  { \
    if(_stg_gui_enable_locking && !_stg_gui_in_gtk_event_loop) \
    { \
      /*printf("___ gui_unlock @ %s (%s:%d) ___\n", __PRETTY_FUNCTION__, __FILE__, __LINE__);*/ \
      pthread_mutex_unlock(&_stg_gui_mutex); \
      gdk_threads_leave(); \
    } \
  }

  #define gui_unlock_leaving_gtk_event_loop() { \
    if(_stg_gui_enable_locking) { \
      /*printf("______ gui_unlock_leaving_gtk @ %s (%s:%d) _____\n", __PRETTY_FUNCTION__, __FILE__, __LINE__);*/ \
      _stg_gui_in_gtk_event_loop = FALSE; \
      pthread_mutex_unlock(&_stg_gui_mutex); \
      gdk_threads_leave(); \
    } \
  }
#else
#define gui_lock()
#define gui_unlock()
#define gui_lock_entering_gtk_event_loop()
#define gui_unlock_leaving_gtk_event_loop()
#endif

  /**** end reed's hack ****/

  // callback functions
  //typedef void(*func_init_t)(struct _stg_model*);
  typedef int(*func_update_t)(struct _stg_model*);
  typedef int(*func_startup_t)(struct _stg_model*);
  typedef int(*func_shutdown_t)(struct _stg_model*);
  typedef void(*func_load_t)(struct _stg_model*, int);
  typedef void(*func_save_t)(struct _stg_model*);
  typedef void(*func_destroy_t)(struct _stg_model*);
  

  struct _stg_property;

  /** define a callback function type that can be used to override the 
      default data storage mechanism for a property. This is useful when
      you want to process some data before storing it, or if changing the
      property has side effects. For example, a storage callback is
      attached to the "geom" property: when the property is set the
      robot's size may change, so this the storage function makes sure the
      model's polygons are re-normalized and re-rendered. 
  */
  typedef void (*stg_property_storage_func_t)( struct _stg_property* prop, 
					       void* data, size_t len );
  
  /** defines a property of a model.The property is uniquely
      identified by the string [name]. You probably should not access
      these fields directly - use stg_model_get_property() and
      stg_model_set_property() instead.
  */
  typedef struct _stg_property
  {
    char name[STG_PROPNAME_MAX];
    void* data;
    size_t len;
    stg_property_storage_func_t storage_func;
    GList* callbacks; // functions called when this property is set
    stg_model_t* mod; // the model to which this property belongs
  } stg_property_t;

  typedef struct 
  {
    stg_model_t* mod;
    char propname[STG_PROPNAME_MAX];
    void* data;
    size_t len;
    void* user;
  } stg_property_callback_args_t;
  
  typedef struct 
  {
    stg_model_t* mod;
    //char propname[STG_PROPNAME_MAX];
    const char *propname;
    stg_property_callback_t callback_on;
    stg_property_callback_t callback_off;
    void* arg_on; // argument to callback_on
    void* arg_off; // argument to callback_off
  } stg_property_toggle_args_t;
    
  struct stg_cell;
  
  struct _stg_model
  {
    //stg_id_t id; // used as hash table key  // Obsolete: this has been split to allow unique id without 1-to-1 relationship with CEntity entries
    stg_id_t unique_id; // Unique id, used as a hash table key for world->models
    stg_id_t entity_id; // Index of the entry in CWorldFile::*entities, so the model knows which data to initialize from


    stg_world_t* world; // pointer to the world in which this model exists
    char* token; // automatically-generated unique ID string
    //int type; // what kind of a model am I?

    struct _stg_model *parent; // the model that owns this one, possibly NULL
    struct _stg_model *root;   // the root of the model tree this model is in

    GPtrArray* children; // the models owned by this model

    // a table that can contain arbitrary named data items. Can be used
    // by derived model types to store properties, and for user code
    // to associate arbitrary items with a model.
    GHashTable* props;

    // a datalist of stg_rtk_figs, indexed by name (string)
    GData* figs; 

    // the number of children of each type is counted so we can
    // automatically generate names for them
    int child_type_count[256];

    int subs;     // the number of subscriptions to this model

    stg_msec_t interval; // time between updates in ms (unused, global world interval is used)
    stg_msec_t interval_elapsed; // time since last update in ms

    stg_bool_t disabled; // if non-zero, the model is disabled
    
    // type-dependent functions for this model, implementing simple
    // polymorphism
    stg_model_initializer_t initializer;
    func_startup_t f_startup;
    func_shutdown_t f_shutdown;
    func_update_t f_update;
    func_load_t f_load;
    func_save_t f_save;
    func_destroy_t f_destroy; ///< hook for model destruction (called by stg_model_destroy()). This function can remove property toggles from gui, or any other memory allocation (except properties, those are automatically destroyed)

#ifdef STG_ENABLE_MODEL_LOCK
    // allow exclusive access to this model's properties
    pthread_mutex_t mutex;
    gboolean mutex_init;
#endif
    // base model type string (corresponds to 'type' value)
    char* base_type_name;

    // index used to uniquely identify this model type when there are
    // more than one of the same type (combined with base type name
    // to create unique token above)
    size_t type_instance_index;

    // Type name given when creating this model in the world file.
    char* instance_name;

    // in a collided state
    stg_bool_t collision;

    // this model's property gui toggle data (list of stg_property_toggle_args_t*)
    GList* property_toggles;

    // Matrix cell containing this model's center, or NULL if unknown. This is
    // a cache to improve performance when doing raycasting from near this
    // location.
    struct stg_cell *current_matrix_cell;

    stg_msec_t lock_time;  ///< Used to warn if model is locked for too long

    stg_bool_t bg_fig;
    stg_bool_t should_scale;
  };
  
  typedef struct {
    /** Name of the type. */
    const char* keyword;

    /** Function to call that initializes a model of this type. */
    stg_model_initializer_t initializer;

    /** Datalist of stg_property_t to supply default values 
     *  for properties of the type, created when a new model 
     *  of this type is created by model_new. 
     *  May be NULL if none.
     */
    GData *init_properties; 
  } stg_type_record_t;


  extern stg_type_record_t* stg_user_typetable;  // Custom types added by stg_model_register_custom_type()

  extern stg_type_record_t* worldfile_typetable; // defined in world.c

  // internal functions
  
  int _model_update( stg_model_t* mod );
  int _model_startup( stg_model_t* mod );
  int _model_shutdown( stg_model_t* mod );

  void stg_model_update_velocity( stg_model_t* model );
  int stg_model_update_pose( stg_model_t* model );
  void stg_model_energy_consume( stg_model_t* mod, stg_watts_t rate );
  void stg_model_map( stg_model_t* mod, gboolean render );
  void stg_model_map_with_children( stg_model_t* mod, gboolean render );
  
  stg_rtk_fig_t* stg_model_prop_fig_create( stg_model_t* mod, 
				    stg_rtk_fig_t* array[],
				    stg_id_t propid, 
				    stg_rtk_fig_t* parent,
				    int layer );

  void stg_model_render_geom( stg_model_t* mod );
  void stg_model_render_pose( stg_model_t* mod );
  void stg_model_render_polygons( stg_model_t* mod );
  
  int stg_fig_clear_cb(  stg_model_t* mod, char* name, 
			 void* data, size_t len, void* userp );
  
  stg_rtk_fig_t* stg_model_fig_create( stg_model_t* mod, 
				       const char* figname, 
				       const char* parentname,
				       int layer );
  
  /** 
   * (in gui.c, by the way, not model.c)
   */
  stg_rtk_fig_t* stg_model_get_fig( stg_model_t* mod, const char* figname );
  void stg_model_fig_clear( stg_model_t* mod, const char* figname );

  void stg_property_refresh( stg_property_t* prop );
  void stg_property_destroy( stg_property_t* prop );

  /** extended version of stg_model_set_property() which allows you to
      install a storage function for this property 
  */
  void stg_model_set_property_ex( stg_model_t* mod, 
				  const char* prop, 
				  void* data, 
				  size_t len,
				  stg_property_storage_func_t func );
         
  // defines a simulated world
  struct _stg_world
  {
    stg_id_t id; ///< Stage's unique identifier for this world
    
    GHashTable* models; ///< the models that make up the world, indexed by id
    GHashTable* models_by_name; ///< the models that make up the world, indexed by name
    
    stg_meters_t width; ///< x size of the world 
    stg_meters_t height; ///< y size of the world

    /** the number of models of each type is counted so we can
	automatically generate names for them
    */
    int child_type_count[256];
    
    struct _stg_matrix* matrix; ///< occupancy quadtree for model raytracing

    char* token; ///< the name of this world

    stg_msec_t sim_time; ///< the current time in this world
    stg_msec_t sim_interval; ///< this much simulated time elapses each step.
   
    /** real-time interval between updates - set this to zero for 'as fast as possible' 
     */
    stg_msec_t real_interval;

    stg_msec_t real_last_update; ///< the wall-clock time of the last update
    
    /** the wallclock-time interval elapsed between the last two
	updates - compare this with sim_interval to see the ratio of
	sim to real time 
    */
    stg_msec_t real_interval_measured;

    double avg_interval_ratio;

    size_t real_interval_too_long_warnings;  ///< Incremented each time an interval is more than 10% larger than the requested real_interval
    size_t zero_interval_warnings;    ///< incremented each time an update call-interval is less than 5 ms.

    double ppm; ///< the resolution of the world model in pixels per meter

    gboolean paused; ///< the world only updates when this is zero
   
    gboolean destroy; ///< this world should be destroyed ASAP

    gui_window_t* win; ///< the gui window associated with this world

    int subs; ///< the total number of subscriptions to all models

    GList* model_property_toggles;  ///< list of all gui property toggles for all models (stg_model_property_toggles*)

    GHashTable* file_loaders; ///< Hash of { char* (pattern) ,  _stg_world_file_loader_entry struct (entry) }

    stg_world_locking_policy_t model_locking_policy;  ///< Whether to lock, trylock, or not
    void (*model_update_cb)(gpointer, gpointer, gpointer); ///< Function to call for each model in the @a models GHashTable to update it.  There are different update callbacks for different locking policies.
    gboolean world_locking_enabled; ///< should stg_world_lock and unlock actually try locking the mutexes? (by default, FALSE)
#ifdef STG_ENABLE_WORLD_LOCK
    pthread_mutex_t mutex;
#endif

    gboolean use_model_height;  ///< whether the user has enabled or disabled model height checks (used by some sensors like laser)

    stg_msec_t lock_time;     ///< used to warn if 'mutex' is locked for a long time
    gboolean quiet;
    gboolean gui_enabled;

    gboolean display_degrees;
    gboolean display_mm;

    stg_msec_t last_stats_log_time;
    unsigned long update_miss_count;
  };

  // ROTATED RECTANGLES -------------------------------------------------

  /** @defgroup rotrect Rotated Rectangles
      @{ 
  */
  
  /** defines a rectangle of [size] located at [pose] */
  typedef struct
  {
    stg_pose_t pose;
    stg_size_t size;
  } stg_rotrect_t; // rotated rectangle
  
  /** normalizes the set [rects] of [num] rectangles, so that they fit
      exactly in a unit square.
  */
  void stg_rotrects_normalize( stg_rotrect_t* rects, int num );
  
  /** load the image file [filename] and convert it to an array of
      rectangles, filling in the number of rects, width and
      height. Memory is allocated for the rectangle array [rects], so
      the caller must free [rects].
  */
  int stg_rotrects_from_image_file( const char* filename, 
				    stg_rotrect_t** rects,
				    int* rect_count,
				    int* widthp, int* heightp );
  
  int stg_rotrects_from_image_pixbuf(GdkPixbuf *pb, stg_rotrect_t** rects, int *rect_count, int *widthp, int *heightp);

  /** load [filename], an image format understood by gdk-pixbuf, and
      return a set of rectangles that approximate the image. Caller
      must free the array of rectangles. If width and height are
      non-null, they are filled in with the size of the image in pixels 
  */
  stg_polygon_t* stg_polygons_from_rotrects( stg_rotrect_t* rects, size_t count );

  /**@}*/

  
  // MATRIX  -----------------------------------------------------------------------
  
  /** @defgroup stg_matrix Matrix occupancy quadtree
      Occupancy quadtree underlying Stage's sensing and collision models. 
      @{ 
  */
  
  /** A node in the occupancy quadtree */
  typedef struct stg_cell
  {
    void* data;
    double x, y;
    double size;
    
    // bounding box
    double xmin,ymin,xmax,ymax;
    
    stg_rtk_fig_t* fig; // for debugging

    struct stg_cell* children[4];
    struct stg_cell* parent;

    gboolean is_line;       // if the cell is a leaf for a line (rather than a raster/bitmap cell)
    double line_angle;  // if the cell contains line data, this is the line's angle in global space. 
  } stg_cell_t;
  
  /** in the cell-tree which contains cell, return the smallest cell that contains the point x,y. cell need not be the root of the tree
   */
  stg_cell_t* stg_cell_locate( stg_cell_t* cell, double x, double y );
  
  void stg_cell_unrender( stg_cell_t* cell );
  void stg_cell_render( stg_cell_t* cell );
  void stg_cell_render_tree( stg_cell_t* cell );
  void stg_cell_unrender_tree( stg_cell_t* cell );
  void stg_cell_print(stg_cell_t* cell, char* prefix);

  /** Occupancy quadtree structure */
  typedef struct _stg_matrix
  {
    double ppm; // pixels per meter (1/resolution)
    double width, height;
    
    // A quad tree of cells. Each leaf node contains a list of
    // pointers to objects located at that cell
    stg_cell_t* root;
    
    // hash table stores all the pointers to objects rendered in the
    // quad tree, each associated with a list of cells in which it is
    // rendered. This allows us to remove objects from the tree
    // without doing the geometry again
    GHashTable* ptable;

    /* TODO */
    // lists of cells that have changed recently. This is used by the
    // GUI to render cells very quickly, and could also be used by devices
    //GSList* cells_changed;
    
    // debug figure. if this is non-NULL, debug info is drawn here
    stg_rtk_fig_t* fig;

    // todo - record a timestamp for matrix mods so devices can see if
    //the matrix has changed since they last peeked into it
    // stg_msec_t last_mod_time;
    
    // for locking the matrix. TODO: remove these and make things work using
    // just world mutex 
#ifdef STG_ENABLE_WORLD_LOCK
    pthread_mutex_t mutex;   // used by world to protect this matrix (and only used by world); the mutex is initialized by stg_matrix_enable_lock()
#endif
    gboolean locking_enabled;

  } stg_matrix_t;
  

  /** Create a new matrix structure
   */
  stg_matrix_t* stg_matrix_create( double ppm, double width, double height );

  /** If neccesary, expand the matrix root cell to encompass the given width &
   * height (meters)
   */
  void stg_matrix_resize(stg_matrix_t* matrix, double new_wedith, double new_height);
  
  /** Frees all memory allocated by the matrix; first the cells, then the   
      cell array.
  */
  void stg_matrix_destroy( stg_matrix_t* matrix );
  
  /** removes all pointers from every cell in the matrix
   */
  void stg_matrix_clear( stg_matrix_t* matrix );
  
  /** Get the pointer from the cell at x,y (in meters).
   */
  void* stg_matrix_cell_get( stg_matrix_t* matrix, int r, double x, double y);

  /** append the pointer [object] to the list of pointers in the [matrix] cell at location [x,y]
   */
  void stg_matrix_cell_append(  stg_matrix_t* matrix, 
				double x, double y, void* object );
  
  /** remove [object] from all cells it occupies in the matrix
   */
  void stg_matrix_remove_obect( stg_matrix_t* matrix, void* object );
  
  /** if [object] appears in the cell's list, remove it
   */
  void stg_matrix_cell_remove(  stg_matrix_t* matrix,
				double x, double y, void* object );
  
  /** Append to the [object] pointer to the cells on the edge of a
      rectangle, described in meters about a center point.
  */
  void stg_matrix_rectangle( stg_matrix_t* matrix,
			     double px, double py, double pth,
			     double dx, double dy, 
			     void* object );
  
  /** Render [object] as a line in the matrix.
  */
  void stg_matrix_line( stg_matrix_t* matrix, 
			double x1, double y1, 
			double x2, double y2,
			void* object );

  /** specify a line from (x1,y1) to (x2,y2), all in meters
   */
  typedef struct
  {
    stg_meters_t x1, y1, x2, y2;
  } stg_line_t;
  

  /** Find or create the smallest (according to leaf_cell_size -- aka
   * resolution) cell for the given point.
   */
  stg_cell_t* stg_find_or_create_leaf_cell(stg_cell_t *startcell, double x, double y, double leaf_cell_size);

  /** Call stg_matrix_line for each of [num_lines] lines 
   */
  void stg_matrix_lines( stg_matrix_t* matrix, 
			 stg_line_t* lines, int num_lines,
			 void* object );
    
  /** render an array of polygons into the matrix
   */
  void stg_matrix_polygons( stg_matrix_t* matrix,
			    double x, double y, double a,
			    stg_polygon_t* polys, int num_polys,
			    void* object );

  /** project a set of points into the matrix.
      @bug @arg a is ignored currently
  */
  void stg_matrix_points(stg_matrix_t *matrix, 
    double x, double y, double a,
    stg_point_t *points, size_t npoints, void *object);

  /** remove all reference to an object from the matrix
   */
  void stg_matrix_remove_object( stg_matrix_t* matrix, void* object );

  /** Only for use from stg_world functions. TODO remove this and make things
   * work using just the world mutex. Does nothing if locking is not enabled at
   * compile time. */
#ifdef STG_ENABLE_MATRIX_LOCK
  void stg_matrix_lock(stg_matrix_t *matrix);
#else
#define stg_matrix_lock(matrix)
#endif

  /** Only for use from stg_world functions. TODO remove this and make things
   * work using just the world mutex. Does nothing if locking is not enabled at
   * compile time. */
#ifdef STG_ENABLE_MATRIX_LOCK
  void stg_matrix_unlock(stg_matrix_t *matrix);
#else
#define stg_matrix_unlock(matrix)
#endif

  /** Only for use from stg_world functions. TODO remove this and make things
   * work using just the world mutex. Does nothing if locking is not enabled at
   * compile time. */
#ifdef STG_ENABLE_MATRIX_LOCK
  void stg_matrix_enable_lock(stg_matrix_t *matrix, gboolean enable);
#else
#define stg_matrix_enable_lock(m, e)
#endif


  /**@}*/

  // RAYTRACE ITERATORS -------------------------------------------------------------
  
  /** @defgroup stg_itl Raytracing in a Matrix
      Iterators for raytracing in a matrix
      @{ */
  
  typedef struct
  {
    double x, y, a;
    double cosa, sina, tana;
    double range;
    double max_range;
    double obs_angle; /// If the other model hit is a line edge, this is its angle in world coordinates
    double* incr;

    GSList* models;
    int index;
    stg_matrix_t* matrix;    
  
  } itl_t;
  
  typedef enum { PointToPoint=0, PointToBearingRange } itl_mode_t;
  
  typedef int(*stg_itl_test_func_t)(stg_model_t* finder, stg_model_t* found );
  
  /** create an itl_t object that can be used for raytracing
   * @param x Starting location X
   * @param y Starting location Y
   * @param a In PointToBearing mode, bearing angle from (x,y) to send the ray.
   * In PointToPoint mode, ending location X?
   * @param b In PointToBearing mode, maximum range to stop the ray. In
   * PointToPoint mode, ending location Y?
   * @param matrix Matrix to search
   * @param pmode Raytracing mode
   */
  itl_t* itl_create( double x, double y, double a, double b, 
		     stg_matrix_t* matrix, itl_mode_t pmode );

  /** Reset members of an existing itl_t object. Use to avoid reallocating
   * objects in a loop. */
  itl_t* itl_reset(itl_t* itl, double x, double y, double a, double b, itl_mode_t pmode);
  
  void itl_destroy( itl_t* itl );
  void itl_raytrace( itl_t* itl );
  
  /** Return a pointer to the first model that the ray intersects for which @a
   * func returns TRUE.
      @param itl an itl_t object returned by itl_create.
      @param func a function to test a candidate model for ray intersection. If it returns TRUE, then the intersected model is returned. If FALSE, it is not.
      @param finder Pointer to the model to pass to @a func
      @param startcell Matrix cell to begin searching from, or NULL to use matrix root
    @return the model found, or NULL if no model found.
  */
  stg_model_t* itl_first_matching( itl_t* itl, 
				   stg_itl_test_func_t func, 
				   stg_model_t* finder);

  
  /** Return a GLib GSList of stg_model_t* that a ray crosses and the function
      returns true, or until the end of the ray is reached.
      The GSList will be in order from nearest to farthest.
      @param itl an itl_t object returned by itl_create.
      @param func a function to test a candidate model for ray intersection. If it returns TRUE, then the intersected model is added to the return list. If FALSE, it is not.
      @param finder Pointer to the model to pass to @a func
     @return GSList of models found, or NULL if no model found. You must free the GSList with g_slist_free().
    */
  GSList* itl_all_matching(itl_t *itl, stg_itl_test_func_t func, stg_model_t *finder);

  /** @} */

  /** @defgroup worldfile worldfile C wrappers
      @{
  */
  
  // C wrappers for C++ worldfile functions
  int wf_property_exists( int section, const char* token );
  int wf_read_int( int section, const char* token, int def );
  double wf_read_length( int section, const char* token, double def );
  double wf_read_angle( int section, const char* token, double def );
  double wf_read_float( int section, const char* token, double def );
  const char* wf_read_tuple_string( int section, const char* token, int index, const char* def );
  double wf_read_tuple_float( int section, const char* token, int index, double def );
  double wf_read_tuple_length( int section, const char* token, int index, double def );
  double wf_read_tuple_angle( int section, const char* token, int index, double def );
  const char* wf_read_string( int section, const char* token, const char* def );

  void wf_write_int( int section, const char* token, int value );
  void wf_write_length( int section, const char* token, double value );
  void wf_write_angle( int section, const char* token, double value );
  void wf_write_float( int section, const char* token, double value );
  void wf_write_string(int section, const char* token, const char *value);
  void wf_write_tuple_string( int section, const char* token, int index, const char* value );
  void wf_write_tuple_float( int section, const char* token, int index, double value );
  void wf_write_tuple_length( int section, const char* token, int index, double value );
  void wf_write_tuple_angle( int section, const char* token, int index, double value );
  void wf_write_tuple_int(int section, const char* token, int index, int value);

  void wf_save( void );
  int wf_load( const char* path, int echo );  ///< @return 1 if ok, 0 if error
  int wf_section_count( void );
  const char* wf_get_section_type( int section );
  int wf_get_parent_section( int section );
  const char* wf_get_filename( void);

  /// @return section id for the named section, or -1 if not found
  int wf_lookup_section(const char *name);

  /// @return id for the parent of the given section, or -1 if it has no parent or invalid
  int wf_get_parent_section(int childsection);

  /// @return id for a macro definition (from "define" keyword) with the given name, or -1 if not found
  int wf_lookup_macro_def(const char *name);

  /// @return name of the macro definition with the given id
  const char *wf_get_macro_name(int macro);

  /// @return name of the macro's "parent" macro. This is the same as its section name, actually.
  const char *wf_get_macro_parent_name(int macro);

  /// @return the id of the parent of a macro definition (if the macro is defined based on a previous macro)
  int wf_get_macro_parent(int macro);

  /// @return name of the parent macro definition (if the macro is defined basen a previous macro)
  const char *wf_get_macro_parent_name(int macro);

  /// @return name of the parent macro definition (if the macro is defined basen a previous macro)
  const char *wf_get_macro_section_name(int macro);  


  /// @return the base model type name of a macro
  const char * wf_get_macro_base_type(int macro); 


  int wf_create_entity(const char *type, int parentent);
  int wf_get_next_child_entity(int parent, int start);
  const char *wf_get_section_immediate_type(int section);

  void wf_set_entity_id_for_model_type(char *typestr, stg_id_t entity_id);
  int wf_find_entity_id_for_model_type(char *typestr);

  /** @} */


  // switch a gui property callback toggle on or off
  void model_do_property_toggle(stg_property_toggle_args_t* args, gboolean on);
  void stg_model_destroy_polygons(stg_model_t* mod);

  // CALLBACK WRAPPERS ------------------------------------------------------------

  // callback wrappers for other functions
  void model_print_cb( gpointer key, gpointer value, gpointer user );
  void model_destroy_cb( gpointer mod );
  void model_property_toggles_off_cb(gpointer mod, gpointer userdata);
  void model_property_toggles_on_cb(gpointer mod, gpointer userdata);



  

// Macros for printing output, deprecated except for the ones used for debugging.

#define PRINT_ERR(m) stg_print_error(m);
#define PRINT_ERR1(m,a) stg_print_error(m, a);
#define PRINT_ERR2(m,a,b) stg_print_error(m, a, b);
#define PRINT_ERR3(m,a,b,c) stg_print_error(m, a, b, c);
#define PRINT_ERR4(m,a,b,c,d) stg_print_error(m, a, b, c, d);
#define PRINT_ERR5(m,a,b,c,d,e) stg_print_error(m, a, b, c, d, e);

/* Not used:
#define PRINT_ERR_SRCFILE(m) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", __FILE__, __FUNCTION__)
#define PRINT_ERR1_SRCFILE(m,a) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
#define PRINT_ERR2_SRCFILE(m,a,b) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
#define PRINT_ERR3_SRCFILE(m,a,b,c) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
#define PRINT_ERR4_SRCFILE(m,a,b,c,d) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
#define PRINT_ERR5_SRCFILE(m,a,b,c,d,e) fprintf( stderr, "\033[41mError\033[0m: "m" (%s %s)\n", a, b, c, d, e, __FILE__, __FUNCTION__)
*/

  // Warning macros
#define PRINT_WARN(m) stg_print_warning(m); 
#define PRINT_WARN1(m,a) stg_print_warning(m, a);
#define PRINT_WARN2(m,a,b) stg_print_warning(m, a, b);
#define PRINT_WARN3(m,a,b,c) stg_print_warning(m, a, b, c);
#define PRINT_WARN4(m,a,b,c,d) stg_print_warning(m, a, b, c, d);
#define PRINT_WARN5(m,a,b,c,d,e) stg_print_warning(m, a, b, c, d, e);

/* Not used:
#define PRINT_WARN_SRCFILE(m) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", __FILE__, __FUNCTION__)
#define PRINT_WARN1_SRCFILE(m,a) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
#define PRINT_WARN2_SRCFILE(m,a,b) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
#define PRINT_WARN3_SRCFILE(m,a,b,c) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
#define PRINT_WARN4_SRCFILE(m,a,b,c,d) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
#define PRINT_WARN5_SRCFILE(m,a,b,c,d,e) printf( "\033[44mWarning\033[0m: "m" (%s %s)\n", a, b, c, d, e, __FILE__, __FUNCTION__)
*/

  // Message macros
#ifdef DEBUG
#define PRINT_MSG(m) printf( "stage: "m" (%s %s)\n", __FILE__, __FUNCTION__)
#define PRINT_MSG1(m,a) printf( "stage: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
#define PRINT_MSG2(m,a,b) printf( "stage: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
#define PRINT_MSG3(m,a,b,c) printf( "stage: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
#define PRINT_MSG4(m,a,b,c,d) printf( "stage: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
#define PRINT_MSG5(m,a,b,c,d,e) printf( "stage: "m" (%s %s)\n", a, b, c, d, e,__FILE__, __FUNCTION__)
#else
#define PRINT_MSG(m) stg_print_msg(m);
#define PRINT_MSG1(m,a) stg_print_msg(m, a);
#define PRINT_MSG2(m,a,b) stg_print_msg(m, a, b);
#define PRINT_MSG3(m,a,b,c) stg_print_msg(m, a, b, c);
#define PRINT_MSG4(m,a,b,c,d) stg_print_msg(m, a, b, c, d);
#define PRINT_MSG5(m,a,b,c,d,e) stg_print_msg(m, a, b, c, d, e);
#endif

  // DEBUG macros
#ifdef DEBUG
#define PRINT_DEBUG(m) printf( "debug: "m" (%s %s)\n", __FILE__, __FUNCTION__)
#define PRINT_DEBUG1(m,a) printf( "debug: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
#define PRINT_DEBUG2(m,a,b) printf( "debug: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
#define PRINT_DEBUG3(m,a,b,c) printf( "debug: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
#define PRINT_DEBUG4(m,a,b,c,d) printf( "debug: "m" (%s %s)\n", a, b, c ,d, __FILE__, __FUNCTION__)
#define PRINT_DEBUG5(m,a,b,c,d,e) printf( "debug: "m" (%s %s)\n", a, b, c ,d, e, __FILE__, __FUNCTION__)
#define PRINT_DEBUG6(m,a,b,c,d,e,f) printf( "debug: "m" (%s %s)\n", a, b, c ,d, e, f, __FILE__, __FUNCTION__)
#else
#define PRINT_DEBUG(m)
#define PRINT_DEBUG1(m,a)
#define PRINT_DEBUG2(m,a,b)
#define PRINT_DEBUG3(m,a,b,c)
#define PRINT_DEBUG4(m,a,b,c,d)
#define PRINT_DEBUG5(m,a,b,c,d,e)
#define PRINT_DEBUG6(m,a,b,c,d,e,f)
#endif

extern char stg_last_function[128];
//#define STG_F_INIT() { strncpy(stg_last_function, "none", 127); }
//#define STG_F() { printf("%s [STG_F()]\n", __PRETTY_FUNCTION__); strncpy(stg_last_function, __PRETTY_FUNCTION__, 127); stg_last_function[127] = '\0'; }
//#define STG_XF(f) { printf("%s [STG_XF()]\n", f); strncpy(stg_last_function, f, 127); stg_last_function[127] = '\0'; }

#define STG_F_INIT() {}
#define STG_F() { }
#define STG_XF(f) { }

// end documentation group stage
/**@}*/




// TODO - some of this needs to be implemented, the rest junked.

/*   //  -------------------------------------------------------------- */

/*   // standard energy consumption of some devices in W. */
/*   // */
/*   // The MOTIONKG value is a hack to approximate cost of motion, as */
/*   // Stage does not yet have an acceleration model. */
/*   // */
/* #define STG_ENERGY_COST_LASER 20.0 // 20 Watts! (LMS200 - from SICK web site) */
/* #define STG_ENERGY_COST_FIDUCIAL 10.0 // 10 Watts */
/* #define STG_ENERGY_COST_RANGER 0.5 // 500mW (estimate) */
/* #define STG_ENERGY_COST_MOTIONKG 10.0 // 10 Watts per KG when moving  */
/* #define STG_ENERGY_COST_BLOB 4.0 // 4W (estimate) */

/*   typedef struct */
/*   { */
/*     stg_joules_t joules; // current energy stored in Joules/1000 */
/*     stg_watts_t watts; // current power expenditure in mW (mJoules/sec) */
/*     int charging; // 1 if we are receiving energy, -1 if we are */
/*     // supplying energy, 0 if we are neither charging nor */
/*     // supplying energy. */
/*     stg_meters_t range; // the range that our charging probe hit a charger */
/*   } stg_energy_data_t; */

/*   typedef struct */
/*   { */
/*     stg_joules_t capacity; // maximum energy we can store (we start fully charged) */
/*     stg_meters_t probe_range; // the length of our recharge probe */
/*     //stg_pose_t probe_pose; // TODO - the origin of our probe */

/*     stg_watts_t give_rate; // give this many Watts to a probe that hits me (possibly 0) */
  
/*     stg_watts_t trickle_rate; // this much energy is consumed or */
/*     // received by this device per second as a */
/*     // baseline trickle. Positive values mean */
/*     // that the device is just burning energy */
/*     // stayting alive, which is appropriate */
/*     // for most devices. Negative values mean */
/*     // that the device is receiving energy */
/*     // from the environment, simulating a */
/*     // solar cell or some other ambient energy */
/*     // collector */

/*   } stg_energy_config_t; */


/*   // BLINKENLIGHT ------------------------------------------------------------ */

/*   // a number of milliseconds, used for example as the blinkenlight interval */
/* #define STG_LIGHT_ON UINT_MAX */
/* #define STG_LIGHT_OFF 0 */

  //typedef int stg_interval_ms_t;


/*   // TOKEN ----------------------------------------------------------------------- */
/*   // token stuff for parsing worldfiles - this may one day replace
the worldfile c++ code */

/* #define CFG_OPEN '(' */
/* #define CFG_CLOSE ')' */
/* #define STR_OPEN '\"' */
/* #define STR_CLOSE '\"' */
/* #define TPL_OPEN '[' */
/* #define TPL_CLOSE ']' */

/*   typedef enum { */
/*     STG_T_NUM = 0, */
/*     STG_T_BOOLEAN, */
/*     STG_T_MODELPROP, */
/*     STG_T_WORLDPROP, */
/*     STG_T_NAME, */
/*     STG_T_STRING, */
/*     STG_T_KEYWORD, */
/*     STG_T_CFG_OPEN, */
/*     STG_T_CFG_CLOSE, */
/*     STG_T_TPL_OPEN, */
/*     STG_T_TPL_CLOSE, */
/*   } stg_token_type_t; */




/* typedef struct stg_token  */
/* { */
/*   char* token; ///< the text of the token */
/*   stg_token_type_t type; ///< the type of the token */
/*   unsigned int line; ///< the line on which the token appears */
  
/*   struct stg_token* next; ///< linked list support */
/*   struct stg_token* child; ///< tree support */
  
/* } stg_token_t; */

/*   stg_token_t* stg_token_next( stg_token_t* tokens ); */
/*   /// print [token] formatted for a human reader, with a string [prefix] */
/*   void stg_token_print( char* prefix,  stg_token_t* token ); */

/*   /// print a token array suitable for human reader */
/*   void stg_tokens_print( stg_token_t* tokens ); */
/*   void stg_tokens_free( stg_token_t* tokens ); */
  
/*   /// create a new token structure from the arguments */
/*   stg_token_t* stg_token_create( const char* token, stg_token_type_t type, int line ); */

/*   /// add a token to a list */
/*   stg_token_t* stg_token_append( stg_token_t* head, */
/* 				 char* token, stg_token_type_t type, int line ); */

/*   const char* stg_token_type_string( stg_token_type_t type ); */

/*   const char* stg_model_type_string( stg_model_type_t type ); */
  
/*   //  functions for parsing worldfiles */
/*   stg_token_t* stg_tokenize( FILE* wf ); */
/*   //stg_world_t* sc_load_worldblock( stg_client_t* cli, stg_token_t** tokensptr ); */
/*   //stg_model_t* sc_load_modelblock( stg_world_t* world, stg_model_t* parent, */
/*   //			   stg_token_t** tokensptr ); */


  /** Configure a model by reading from some other section of a world file
   * (rather than the model's own section). 
   * This is mainly to facilitate loading of properties from "uninserted" macro definitions.
   * Normally you would just use stg_model_load() which calls this function
   * using the model's own id.
   */
  void stg_model_load_from_worldfile_section_id(stg_model_t *mod, int worldfile_section_id);

  /** Load a model's properties from all it's contributing macros */
  void stg_world_load_model_macro_properties(stg_world_t *world, stg_model_t *mod, int macrodef);

#ifdef __cplusplus
}
#endif 

/** @} */  
// end of libstage_internal documentation  

#endif // _STAGE_INTERNAL_H

