//#define DEBUG

#include <stdlib.h>
#include "stage_internal.h"
#include "gui.h"

extern stg_rtk_fig_t* fig_debug_geom;
extern stg_rtk_fig_t* fig_debug_rays;
extern stg_rtk_fig_t* fig_debug_matrix;
extern stg_rtk_fig_t* fig_trails;

extern int _render_matrix_deltas;
extern int _stg_quit;



/* Strings used in About box, can be changed by the application: */

#ifndef PACKAGE_VERSION
# ifdef VERSION
#  define PACKAGE_VERSION VERSION
# else
#  error Neither PACKAGE_VERSION or VERSION defined by autoconf!
# endif
#endif

#ifndef PACKAGE_NAME
# ifdef PACKAGE
#  define PACKAGE_NAME PACKAGE
# else
#  error Neither PACKAGE_NAME or PACKAGE defined by autoconf!
# endif
#endif

const char * stg_about_info_appname = "Stage";
const char * stg_about_info_description = "Stage robot simulator, Part of the Player/Stage Project";
const char * stg_about_info_url = "http://playerstage.sourceforge.net/stage/stage.html";
const char * stg_about_info_copyright = "Copyright Richard Vaughan, Andrew Howard, Brian Gerkey\n"\
    " and contributors 2000-2005.\n\n"\
    "Released under the GNU General Public License.";
const char * stg_about_info_appversion = NULL; // use plain stageversion instead by default
const char * stg_about_info_stageversion = PACKAGE_VERSION;

const char * stg_help_link = "http://playerstage.sourceforge.net/index.php?src=stage";

/* Horrid macros to make stuff work in both GTK 2.0 and 2.4 with a minimum of
copy & paste.
   All action functions can expect to have an argument (gpointer userdata), a
   pointer to the Stage world struct. Otherwise, toggle actions must use a 
   macro to determine whether the item was toggled on or off, and radio 
   actions must use a macro to get the value for the selected item.
 */
#if GTK_CHECK_VERSION(2, 4, 0)
  static GtkUIManager *ui_manager = NULL;
  #define _MENU_ACTION_FUNCTION_ARGS GtkAction *action, gpointer userdata
  #define _MENU_TOGGLE_ACTION_FUNCTION_ARGS GtkToggleAction *action, gpointer userdata
  #define _MENU_TOGGLE_PROPERTY_ACTION_FUNCTION_ARGS GtkToggleAction *action, gpointer userdata
  #define _MENU_RADIO_ACTION_FUNCTION_ARGS GtkRadioAction* action, GtkRadioAction *current, gpointer userdata
  #define _MENU_CHECKITEM_IS_ACTIVE (gtk_toggle_action_get_active(action))
  #define _MENU_RADIOITEM_VALUE (gtk_radio_action_get_current_value(action))
#else
  #define _MENU_ACTION_FUNCTION_ARGS gpointer userdata, guint action, GtkWidget* mitem
  #define _MENU_TOGGLE_ACTION_FUNCTION_ARGS gpointer userdata, guint action, GtkWidget* mitem
  #define _MENU_TOGGLE_PROPERTY_ACTION_FUNCTION_ARGS GtkCheckMenuItem* mitem, gpointer userdata
  #define _MENU_RADIO_ACTION_FUNCTION_ARGS gpointer userdata, guint action, GtkWidget *mitem
  #define _MENU_CHECKITEM_IS_ACTIVE (GTK_CHECK_MENU_ITEM(mitem)->active)
  #define _MENU_RADIOITEM_VALUE (action)
#endif

// regular actions
void gui_action_about( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_open_help_link( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_save( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_reload_world( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_reset_poses( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_exit( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_exportframe( _MENU_ACTION_FUNCTION_ARGS );
void gui_action_load_file( _MENU_ACTION_FUNCTION_ARGS );

// toggle actions
void gui_action_exportsequence( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_pause( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_polygons( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_trails( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_disable_polygons( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_grid( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_raytrace( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_geom( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_matrixtree( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_matrixocc( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );
void gui_action_matrixdelta( _MENU_TOGGLE_ACTION_FUNCTION_ARGS );

// radio actions
void gui_action_export_interval( _MENU_RADIO_ACTION_FUNCTION_ARGS );
void gui_action_export_format( _MENU_RADIO_ACTION_FUNCTION_ARGS );




#if GTK_CHECK_VERSION(2, 4, 0)


    /* * * * * * * GTK 2.4 * * * * */


/* Normal items */
static GtkActionEntry entries[] = {
  { "File", NULL, "_File" },
  { "View", NULL, "_View" },
  { "Debug", NULL, "_Debug" },
  { "Clock", NULL, "_Clock" },
  { "Help", NULL, "_Help" },
  { "Model", NULL, "_Models" },
  { "Export", GTK_STOCK_HARDDISK, "_Export" },
  { "ExportInterval", NULL, "Export interval" },
  { "ExportFormat", NULL, "Export bitmap format" },
  { "ExportFrame", NULL, "Single frame", "<ctrl>f", "Export a screenshot to disk", G_CALLBACK(gui_action_exportframe) },
  { "About", GTK_STOCK_ABOUT, "_About", NULL, NULL, G_CALLBACK(gui_action_about) },
  { "Documentation", GTK_STOCK_HELP, "_Documentation", NULL, NULL, G_CALLBACK(gui_action_open_help_link) },
  { "Save", GTK_STOCK_SAVE, "_Save", "<control>S", "Save world state", G_CALLBACK(gui_action_save) },
  { "ReloadWorld", GTK_STOCK_UNDO, "_Reload", "<control>R", "Reload from last saved world state", G_CALLBACK(gui_action_reload_world) },
  { "ResetPositions", GTK_STOCK_REFRESH, "Reset _Positions", "<control>P", "Reset position models to originial locations", G_CALLBACK(gui_action_reset_poses) },
  { "Load", GTK_STOCK_OPEN, "_Load File...", "<control>L", "Load data from a file into the world", G_CALLBACK(gui_action_load_file) },
  { "Exit", GTK_STOCK_QUIT, "E_xit", "<control>Q", "Exit the program", G_CALLBACK(gui_action_exit) },
};

/* Toggle items */
static GtkToggleActionEntry toggle_entries[] = {
  { "ShowPolygons", NULL, "Show polygons", "D", NULL, G_CALLBACK(gui_action_disable_polygons), 1 },
  { "FillPolygons", NULL, "Fill _polygons", "F", "Toggle drawing of filled or outlined polygons", G_CALLBACK(gui_action_polygons), 1 },
  { "Trails", NULL, "Show trails", "T", NULL, G_CALLBACK(gui_action_trails), 0 },
  { "Grid", NULL, "_Grid", "G", "Toggle drawing of 1-metre grid", G_CALLBACK(gui_action_grid), 0 },
  { "Pause", NULL, "_Pause", "P", "Pause the simulation clock", G_CALLBACK(gui_action_pause), 0 },
  { "DebugRays", NULL, "_Raytrace", "<alt>R", "Draw sensor rays", G_CALLBACK(gui_action_raytrace), 0 },
  { "DebugGeom", NULL, "_Geometry", "<alt>G", "Draw model geometry", G_CALLBACK(gui_action_geom), 0 },
  { "DebugMatrixTree", NULL, "Matrix _Tree", "<alt>T", "Show occupancy quadtree", G_CALLBACK(gui_action_matrixtree), 0 },
  { "DebugMatrixOccupancy", NULL, "Matrix _Occupancy", "<alt>M", "Show occupancy grid", G_CALLBACK(gui_action_matrixocc), 0 },
  { "DebugMatrixDelta", NULL, "Matrix _Delta", "<alt>D", "Show changes to quadtree", G_CALLBACK(gui_action_matrixdelta), 0 },
  { "ExportSequence", NULL, "Sequence of frames", "<control>G", "Export a sequence of screenshots at regular intervals", G_CALLBACK(gui_action_exportsequence), 0 },
};

/* Radio items */
static GtkRadioActionEntry export_format_entries[] = {
  { "ExportFormatPNG", NULL, "PNG", NULL, "Portable Network Graphics format", STK_IMAGE_FORMAT_PNG },
  { "ExportFormatJPEG", NULL, "JPEG", NULL, "JPEG format", STK_IMAGE_FORMAT_JPEG },
};
 
static GtkRadioActionEntry export_freq_entries[] = {
  { "ExportInterval1", NULL, "0.1 seconds", NULL, NULL, 100 },
  { "ExportInterval2", NULL, "0.2 seconds", NULL, NULL, 200 },
  { "ExportInterval5", NULL, "0.5 seconds", NULL, NULL, 500 },
  { "ExportInterval10", NULL, "1 second", NULL, NULL, 1000 },
  { "ExportInterval50", NULL, "5 seconds", NULL, NULL, 5000 },
  { "ExportInterval100", NULL, "10 seconds", NULL, NULL, 10000 },
};

static const char *ui_description =
"<ui>"
"  <menubar name='Main'>"
"    <menu action='File'>"
"      <menuitem action='Load'/>"
#ifdef ENABLE_SAVE_WORLD
"      <menuitem action='Save'/>"
#endif
#ifdef ENABLE_RELOAD_WORLD
"      <menuitem action='ReloadWorld'/>"
#endif
"      <menuitem action='ResetPositions'/>"
"      <separator/>"
"        <menu action='Export'>"
"          <menuitem action='ExportFrame'/>" 
"          <menuitem action='ExportSequence'/>" 
"          <separator/>" 
"          <menuitem action='ExportFormat'/>"
"          <menuitem action='ExportFormatPNG'/>"
"          <menuitem action='ExportFormatJPEG'/>"
"          <separator/>"
"          <menuitem action='ExportInterval'/>"
"          <menuitem action='ExportInterval1'/>"
"          <menuitem action='ExportInterval2'/>"
"          <menuitem action='ExportInterval5'/>"
"          <menuitem action='ExportInterval10'/>"
"          <menuitem action='ExportInterval50'/>"
"          <menuitem action='ExportInterval100'/>"
"        </menu>"
"      <separator/>"
"      <menuitem action='Exit'/>"
"    </menu>"
"    <menu action='View'>"
"      <menuitem action='ShowPolygons'/>"
"      <menuitem action='FillPolygons'/>"
"      <menuitem action='Grid'/>"
"      <menuitem action='Trails'/>"
"      <separator/>"
"        <menu action='Debug'>"
"          <menuitem action='DebugRays'/>"
"          <menuitem action='DebugGeom'/>"
"          <menuitem action='DebugMatrixTree'/>"
"          <menuitem action='DebugMatrixOccupancy'/>"
"          <menuitem action='DebugMatrixDelta'/>"
"          <separator/>"
"            <menu action='Model'>"
"              <separator/>"
"            </menu>"
"        </menu>"
"      <separator/>"
"      <placeholder name='AllPropTypeTogglesPlace' />"
"      <separator/>"
"      <placeholder name='ModelPropTogglesPlace' />"
"    </menu>"
"    <menu action='Clock'>"
"      <menuitem action='Pause'/>"
"    </menu>"
"    <menu action='Help'>"
"      <menuitem action='About'/>"
"      <menuitem action='Documentation'/>"
"    </menu>"
"  </menubar>"
"</ui>";



#else		

    /* * * * * * GTK 2.0: * * * * * */


/// @todo XXX need to check certain menu items at startup which begin enabled (grid, polygons, fill)



static GtkItemFactoryEntry menu_entries[] = {
/* 
   -----------------------------------------------------------------------------------------
    MENU ENTRY PATH             KEY        CB FUNCTION                 CB VAL TYPE   ...ICON etc.
   ----------------------------------------------------------------------------------------- 
*/

  /* File Menu: */

	{ "/_File",			              NULL,		   NULL,		                   0,  "<Branch>" 	},
	{ "/_File/tear1", 		        NULL,		   NULL,		                   0,  "<Tearoff>"	},
	{ "/_File/_Load File...",     "<ctrl>l", gui_action_load_file,       0,  "<StockItem>", GTK_STOCK_OPEN },
#ifdef ENABLE_SAVE_WORLD
	{ "/_File/_Save",		            "<ctrl>s", gui_action_save,            1, "<StockItem>",	GTK_STOCK_SAVE },
#endif
#ifdef ENABLE_RELOAD_WORLD
	{ "/_File/_Reload World",            "<ctrl>r", gui_action_reload_world,           1,  "<Item>" },
#endif
	{ "/_File/Reset _Positions",            "<ctrl>p", gui_action_reset_poses,           1,  "<Item>" },
	{ "/File/sep1",               NULL,      NULL,                       0,  "<Separator>" },
	{ "/File/_Export",		        NULL,	     NULL,	                     0,  "<Branch>"	},
	{ "/File/_Export/tear1",	    NULL,	     NULL,	                     0,  "<Tearoff>"	},
	{ "/File/Export/Single frame","<ctrl>f", gui_action_exportframe,     1,  "<Item>" },
	{ "/File/Export/Sequence of frames",NULL,NULL,                       0,  "<CheckItem>" },
	{ "/File/Export/sep2",        NULL,      NULL,                       0,  "<Separator>" },
	{ "/File/Export/Export bitmap format",NULL,NULL,                     0,  "<Title>" },
	{ "/File/Export/PNG",         NULL,      gui_action_export_format,   STK_IMAGE_FORMAT_PNG, "<RadioItem>" },
	{ "/File/Export/JPEG",        NULL,      gui_action_export_format,   STK_IMAGE_FORMAT_JPEG, "/File/Export/PNG" },
	{ "/File/Export/sep3",        NULL,      NULL,                       0,  "<Separator>" },
	{ "/File/Export/Export interval", NULL,  NULL,                       0,  "<Title>" },
	{ "/File/Export/0.1 seconds", NULL,      gui_action_export_interval, 1,  "<RadioItem>" },
	{ "/File/Export/0.2 seconds", NULL,      gui_action_export_interval, 2,  "/File/Export/0.1 seconds" },
	{ "/File/Export/0.5 seconds", NULL,      gui_action_export_interval, 5,  "/File/Export/0.1 seconds" },
	{ "/File/Export/1.0 seconds", NULL,      gui_action_export_interval, 10,  "/File/Export/0.1 seconds" },
	{ "/File/Export/2.0 seconds", NULL,      gui_action_export_interval, 20,  "/File/Export/0.1 seconds" },
	{ "/File/Export/5.0 seconds", NULL,      gui_action_export_interval, 50,  "/File/Export/0.1 seconds" },
	{ "/File/Export/10.0 seconds",NULL,      gui_action_export_interval, 100, "/File/Export/0.1 seconds" },
	{ "/File/Export/sep4",        NULL,      NULL,                       0,   "<Separator>" },
	{ "/File/E_x_it",             "<CTRL>Q", gui_action_exit,            0,   "<StockItem>", GTK_STOCK_QUIT },

	/* View Menu: */
	{ "/_View",                   NULL,      NULL, 0, "<Branch>" },
	{ "/View/tear1",              NULL,      NULL, 0, "<Tearoff>" },
	{ "/View/Show polygons",      "S",      gui_action_disable_polygons,  1,  "<CheckItem>", 1 },
	{ "/View/Fill polygons",      "F" ,     gui_action_polygons,          1,  "<CheckItem>", 1 },
#ifdef ENABLE_GRID_CONTROL
	{ "/View/Grid",               "G",      gui_action_grid,              0,  "<CheckItem>", 0 },
#endif
	{ "/View/Trails",             "T",      gui_action_trails,            0,  "<CheckItem>" },
	{ "/View/sep1",               NULL,     NULL,                         0,  "<Separator>" },
	{ "/View/Debug/Raytrace",     "<ALT>R", gui_action_raytrace,          0, "<CheckItem>" },
	{ "/View/Debug/Geometry",     "<ALT>G", gui_action_geom,              0, "<CheckItem>" },
	{ "/View/Debug/Matrix Tree",  "<ALT>T", gui_action_matrixtree,        0, "<CheckItem>" },
	{ "/View/Debug/Matrix Occupancy", "<ALT>O", gui_action_matrixocc,     0, "<CheckItem>" },
	{ "/View/Debug/Matrix Delta", "<ALT>D", gui_action_matrixdelta,      0, "<CheckItem>" },
	{ "/View/sep3",               NULL,      NULL, 0, "<Separator>" },


	/* Clock Menu: */

	{ "/_Clock",                  NULL,     NULL, 0, "<Branch>" },
	{ "/Clock/tear1",             NULL,     NULL, 0, "<Tearoff>" },
	{ "/Clock/Pause",             "P",      gui_action_pause, 1, "<CheckItem>" },

	/* Help Menu: */


	{ "/_About", NULL, NULL, 0, "<Branch>" },
	{ "/About/_About...",         NULL,     gui_action_about,  0, "<Item>" },
  { "/About/_Documentation...", NULL,     gui_action_open_help_link, 0, "<Item>" },

	{ NULL, NULL, NULL, 0, NULL }

};


#endif	/* end check of GTK 2.0 or 2.4+ for menu action entries */


void gui_action_open_help_link( _MENU_ACTION_FUNCTION_ARGS )
{
  //if(!gtk_show_uri(NULL, stg_help_link, GDK_CURRENT_TIME, NULL))
  //  printf("Stage: error opening help link %s.\n", stg_help_link);
}

void gui_action_about( _MENU_ACTION_FUNCTION_ARGS )
{
	int comments_len = strlen(stg_about_info_description) 
		+ strlen(stg_about_info_stageversion) + strlen("\n\nBuilt with Stage x and GTK xx.xx.xx (running GTK xx.xx.xx)\nhttp://playerstage.sourceforge.net\n");
#if GTK_CHECK_VERSION(2, 6, 0)
	GtkAboutDialog *about = GTK_ABOUT_DIALOG(gtk_about_dialog_new());
	gtk_about_dialog_set_name(about, stg_about_info_appname);
	gtk_about_dialog_set_version(about, stg_about_info_appversion ?  stg_about_info_appversion : PACKAGE_VERSION);
	if(stg_about_info_appversion != NULL)
	{
		char* comments = malloc(comments_len+1); 
		snprintf(comments, comments_len, "%s\n\nBuilt with Stage %s and GTK %d.%d.%d (running GTK %d.%d.%d)\nhttp://playerstage.sourceforge.net",  
				stg_about_info_description, stg_about_info_stageversion, 
				GTK_MAJOR_VERSION, GTK_MINOR_VERSION, GTK_MICRO_VERSION, gtk_major_version, gtk_minor_version, gtk_micro_version);
		gtk_about_dialog_set_comments(about, comments);
		free(comments); ///<@todo is this safe?
	}
	else
	{
		gtk_about_dialog_set_comments(about, stg_about_info_description);
	}
	gtk_about_dialog_set_copyright(about, stg_about_info_copyright);
	gtk_about_dialog_set_website(about, stg_about_info_url);

	/* Destroy the dialog when the user responds to it (e.g. clicks a button) */
	g_signal_connect_swapped( about, "response",
			G_CALLBACK (gtk_widget_destroy),
			about );
#else
	GtkMessageDialog* about;
	size_t textlen;
	char* text;

	textlen = 
		strlen("\nVersion \n\n\n\n\n\n\n") +
		strlen(stg_about_info_appname) +
		strlen(stg_about_info_description) +
		strlen(stg_about_info_url) +
		strlen(stg_about_info_copyright) +
		strlen(stg_about_info_stageversion);
	if(stg_about_info_appversion != NULL)
		textlen += comments_len;
	text = (char*) malloc(textlen + 1);
	///@bug Memory Leak!
	
	assert(text);
	if(stg_about_info_appversion != NULL)
	{
//                       appname     appversion                     stageversion                                                                      copyright
//                       |           |   description                |                                                                                 |
//                       |           |   |   url                    |                                                                                 |
//                       V           V   V   V                      V                                                                                 V
snprintf(text, textlen, "%s\nVersion %s\n%s\n%s\n\nBuilt with Stage %s and GTK %d.%d.%d (running GTK %d.%d.%d)\nhttp://playerstage.sourceforge.net\n\n%s", 
				stg_about_info_appname, stg_about_info_appversion, stg_about_info_description, stg_about_info_url,
				stg_about_info_stageversion, GTK_MAJOR_VERSION, GTK_MINOR_VERSION, GTK_MICRO_VERSION, gtk_major_version, gtk_minor_version, gtk_micro_version, 
				stg_about_info_copyright);
	}
	else
	{
		snprintf(text, textlen, "Version %s\n%s\n\nhttp://playerstage.sourceforge.net\n\n%s", stg_about_info_stageversion, stg_about_info_description, stg_about_info_copyright);
	}
	about = (GtkMessageDialog*)
		gtk_message_dialog_new( NULL, 0, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, text);

	/* Destroy the dialog when the user responds to it (e.g. clicks a button) */
	g_signal_connect_swapped( about, "response",
			G_CALLBACK (gtk_widget_destroy),
			about );
	/* Don't do this, it causes either a deadlock or infinite loop in GTK 2.0:
	   g_signal_connect (about, "destroy",
	   G_CALLBACK(gtk_widget_destroyed), &about);
	 */
#endif

	gtk_widget_show(GTK_WIDGET(about));
}



void gui_action_export_interval( _MENU_RADIO_ACTION_FUNCTION_ARGS )
{
	// set the frame interval in milliseconds
	((stg_world_t*)userdata)->win->frame_interval = _MENU_RADIOITEM_VALUE;

	printf("frame export interval now %d ms\n", 
			((stg_world_t*)userdata)->win->frame_interval );
}

// set the graphics file format for window dumps
void gui_action_export_format( _MENU_RADIO_ACTION_FUNCTION_ARGS )
{
	((stg_world_t*)userdata)->win->frame_format = _MENU_RADIOITEM_VALUE;
}


void gui_action_save( _MENU_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
	printf( "Saving world \"%s\"\n",  world->token );
	stg_world_save( world );
}

void gui_action_reload_world( _MENU_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
	printf( "Reloading world \"%s\"\n",  world->token );
	stg_world_set_cursor_busy(world);
	stg_world_reload( world );
	stg_world_set_cursor_normal(world);
}

void gui_action_reset_poses( _MENU_ACTION_FUNCTION_ARGS )
{
    stg_world_t *world = (stg_world_t*)userdata;
	stg_world_set_cursor_busy(world);
    //stg_world_reset_all_model_poses(world);
    stg_world_reset_position_model_poses(world);
	stg_world_set_cursor_normal(world);
}

void gui_action_load_file(_MENU_ACTION_FUNCTION_ARGS)
{
	stg_world_t* world = (stg_world_t*)userdata;
	stg_world_load_file_act(world);
}

void gui_action_pause( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	PRINT_DEBUG( "Pause menu item" );
	((stg_world_t*)userdata)->paused = _MENU_CHECKITEM_IS_ACTIVE;
}

void gui_action_raytrace( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	PRINT_DEBUG( "Raytrace menu item" );  

	if( _MENU_CHECKITEM_IS_ACTIVE )
	{
		fig_debug_rays = stg_rtk_fig_create( world->win->canvas, NULL, STG_LAYER_DEBUG, "<debug.rays>" );
		stg_rtk_fig_color_rgb32( fig_debug_rays, stg_lookup_color(STG_DEBUG_COLOR) );
	}
	else if( fig_debug_rays )
	{ 
		stg_rtk_fig_destroy( fig_debug_rays );
		fig_debug_rays = NULL;
	}
}

void gui_action_geom( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	PRINT_DEBUG( "Geom menu item" );


	if( _MENU_CHECKITEM_IS_ACTIVE )
	{
		fig_debug_geom = stg_rtk_fig_create( world->win->canvas, NULL, STG_LAYER_GEOM, "<debug.geom>");
		stg_rtk_fig_color_rgb32( fig_debug_geom, 0xFF0000 );
	}
	else if( fig_debug_geom )
	{ 
		stg_rtk_fig_destroy( fig_debug_geom );
		fig_debug_geom = NULL;
	}
}

void gui_action_matrixtree( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;  
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	PRINT_DEBUG( "Matrix tree menu item" );

	if( world->win->matrix_tree ) 
	{
		stg_rtk_fig_destroy( world->win->matrix_tree );     
		world->win->matrix_tree = NULL;     
	}
	else
	{
		world->win->matrix_tree = 
			stg_rtk_fig_create(world->win->canvas,world->win->bg,STG_LAYER_MATRIX_TREE, "<debug.tree>");  
		stg_rtk_fig_color_rgb32( world->win->matrix_tree, 0x00CC00 );      
	} 
}

void gui_action_matrixdelta( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;  
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	PRINT_DEBUG( "Matrix delta menu item" );

	if( _MENU_CHECKITEM_IS_ACTIVE )
	{
		if( ! fig_debug_matrix )
			fig_debug_matrix = stg_rtk_fig_create(world->win->canvas,world->win->bg,STG_LAYER_MATRIX_TREE, "<debug.matrixd>");  

		_render_matrix_deltas = TRUE;

		// doesn't work: too many figs?
		//stg_cell_render_tree( world->matrix->root );
	}
	else
	{
		_render_matrix_deltas = FALSE;
		stg_cell_unrender_tree( world->matrix->root );
	} 
}

void gui_action_matrixocc( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	PRINT_DEBUG( "Matrix occupancy menu item" );

	if( world->win->matrix ) 
	{
		stg_rtk_fig_destroy( world->win->matrix );     
		world->win->matrix = NULL;
	}
	else
	{
		world->win->matrix = 
			stg_rtk_fig_create(world->win->canvas,world->win->bg,STG_LAYER_MATRIX, "<debug.matrixocc>");	  

		stg_rtk_fig_color_rgb32( world->win->matrix, 0x008800 );
	}
}


void test( GtkMenuItem* item, void* userdata )
{
	stg_model_t* mod = (stg_model_t*)userdata;
	printf( "TEST model %s\n", mod->token );

	// TODO - tree text view of models
	/*   GtkDialog* win = gtk_dialog_new_with_buttons ("Model inspector", */
	/* 						NULL, */
	/* 						0, */
	/* 						GTK_STOCK_CLOSE, */
	/* 						GTK_RESPONSE_CLOSE, */
	/* 						NULL); */

	/*   gtk_widget_show( win ); */
}

void gui_add_tree_item( stg_model_t* mod )
{
  STG_F()
#if GTK_CHECK_VERSION(2, 4, 0)

  if(!ui_manager) return;
	GtkWidget* m = 
		gtk_ui_manager_get_widget( ui_manager, "/Main/View/Debug/Model" );
  if(!m) return;
	assert(m);

	GtkWidget* menu = GTK_WIDGET(gtk_menu_item_get_submenu(GTK_MENU_ITEM(m)));
  if(!menu) return;

  GString *label = g_string_new(mod->token);   // Should be freed in gui_remove_tree_item
  g_string_append_printf(label, " (wid=%d; type=%s; parent=%s)", 
    mod->entity_id, mod->base_type_name, mod->parent?mod->parent->token:"null");
	GtkWidget* item = gtk_menu_item_new_with_label(label->str);
  g_object_set_data((GObject*)item, "stg_menu_label_gstring", label);  // save the gstring to be deallocated later
  g_object_set_data((GObject*)item, "stg_model_token", mod->token); // to help find it later
	assert(item);

	gtk_widget_set_sensitive( item, FALSE );

	//GtkWidget* submenu = gtk_menu_new();
	//gtk_menu_item_set_submenu( item, submenu );

	//g_signal_connect( item, "activate", test, (void*)mod );

	gtk_menu_shell_append( (GtkMenuShell*)menu, item );

	gtk_widget_show(item);

#else   /* For GTK 2.0: */
	printf("internal warning: gui_add_tree_item is not implemented for GTK 2.0 yet!\n");
#endif

}

void _gui_destroy_tree_item_if_token_match_cb(GtkWidget *item, gpointer cb_data)
{
  STG_F()
  char *findtoken = (char*)cb_data;
  char *thistoken = (char*)g_object_get_data((GObject*)item, "stg_model_token");
  if(thistoken == NULL) return;
  if(strcmp(findtoken, thistoken) == 0)
  {
    //GString *gstr = g_object_get_data((GObject*)item, "stg_menu_label_gstring");
    //if(gstr) free(gstr); XXX XXX LEAK why does this crash??
    gtk_widget_destroy(item);
  }
}

void gui_remove_tree_item(stg_model_t *mod)
{
  STG_F()
#if GTK_CHECK_VERSION(2, 4, 0)

  if(!ui_manager) return;
	GtkWidget* m = 
		gtk_ui_manager_get_widget( ui_manager, "/Main/View/Debug/Model" );
  if(!m) return;

	GtkWidget* menu = GTK_WIDGET(gtk_menu_item_get_submenu(GTK_MENU_ITEM(m)));
  if(!menu) return;

  // Find menu item for this model and destroy it
  gtk_container_foreach(GTK_CONTAINER(menu), _gui_destroy_tree_item_if_token_match_cb, mod->token);

#else   /* For GTK 2.0: */
	printf("internal warning: gui_add_tree_item is not implemented for GTK 2.0 yet!\n");
#endif
}

void gui_action_exit( _MENU_ACTION_FUNCTION_ARGS )
{
	PRINT_DEBUG( "Exit menu item" );
	_stg_quit = TRUE;
}

/// save a frame as a jpeg, with incremental index numbers. if series
/// is greater than 0, its value is incorporated into the filename
void export_window( gui_window_t* win  ) //stg_rtk_canvas_t* canvas, int series )
{
	char filename[128];
	char* suffix;

	win->frame_index++;

	switch( win->frame_format )
	{
		case STK_IMAGE_FORMAT_JPEG: suffix = "jpg"; break;
		case STK_IMAGE_FORMAT_PNG: suffix = "png"; break;
					   //case STK_IMAGE_FORMAT_PNM: suffix = "pnm"; break;
					   //case STK_IMAGE_FORMAT_PPM: suffix = "ppm"; break;
		default:
					   suffix = ".png";
	}

	if( win->frame_series > 0 )
		snprintf(filename, sizeof(filename), "stage-%03d-%04d.%s",
				win->frame_series, win->frame_index, suffix);
	else
		snprintf(filename, sizeof(filename), "stage-%03d.%s", win->frame_index, suffix);

	printf("Stage: saving [%s]\n", filename);

	stg_rtk_canvas_export_image( win->canvas, filename, win->frame_format );
}


gboolean frame_callback( gpointer data )
{
	export_window( (gui_window_t*)data );
	return TRUE;
}

void gui_action_exportsequence( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	gui_window_t* win = ((stg_world_t*)userdata)->win;

	if( _MENU_CHECKITEM_IS_ACTIVE )
	{
		PRINT_DEBUG1( "Saving frame sequence with interval %d", 
				win->frame_interval );

		// advance the sequence counter - the first sequence is 1.
		win->frame_series++;
		win->frame_index = 0;

		win->frame_callback_tag =
			g_timeout_add( win->frame_interval,
					frame_callback,
					(gpointer)win );
	}
	else
	{
		PRINT_DEBUG( "sequence stop" );
		// stop the frame callback
		g_source_remove(win->frame_callback_tag); 
	}
}



void model_render_polygons_cb( gpointer key, gpointer data, gpointer user )
{
  STG_F()
	stg_model_property_refresh( (stg_model_t*)data, "polygons" );
}


void gui_action_exportframe( _MENU_ACTION_FUNCTION_ARGS )
{ 
	stg_world_t* world = (stg_world_t*)userdata;
	export_window( world->win );
}


void gui_action_trails( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
	PRINT_DEBUG( "Trails menu item" );

	if( _MENU_CHECKITEM_IS_ACTIVE )
	{

		fig_trails = stg_rtk_fig_create( world->win->canvas,
				world->win->bg, STG_LAYER_BODY-1, "<trails>" );      
	}
	else
	{
		stg_rtk_fig_and_descendents_destroy( fig_trails );
		fig_trails = NULL;
	}
}


void gui_action_polygons( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
  stg_world_t* world = (stg_world_t*)userdata;
	PRINT_DEBUG( "Fill Polygons menu item" );
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
	stg_world_set_fill_polygons(world, _MENU_CHECKITEM_IS_ACTIVE );
}

void gui_action_disable_polygons( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
  stg_world_t* world = (stg_world_t*)userdata;
	PRINT_DEBUG( "Disable polygons menu item" );

  // Just a blanket "draw nothing" flag
  if(!world || !world->win || !world->win->canvas) return; // action can be called during gui creation
  world->win->show_polygons = _MENU_CHECKITEM_IS_ACTIVE;
  stg_rtk_canvas_enable(world->win->canvas, _MENU_CHECKITEM_IS_ACTIVE);

  // Another thing we could do is for the "property toggles" system to have flags
  // about what property callbacks are graphics-related, and skip them if
  // show_polygons is false, which might be a performance improvement.
  
}

void gui_action_grid( _MENU_TOGGLE_ACTION_FUNCTION_ARGS )
{
	stg_world_t* world = (stg_world_t*)userdata;
	PRINT_DEBUG( "Grid menu item" );

	// show or hide the layer depending on the state of the menu item
	if(!world->win) return;	// can happen if called before world fully created.
	stg_rtk_canvas_layer_show( world->win->canvas, 
			STG_LAYER_GRID,
			_MENU_CHECKITEM_IS_ACTIVE);
}

void gui_window_menus_create( gui_window_t* win )
{
	GtkAccelGroup* accel_group;
	GtkWidget* menubar;

#if GTK_CHECK_VERSION(2, 4, 0)
	GtkActionGroup* group;


	ui_manager = gtk_ui_manager_new ();

	// actions
	group = gtk_action_group_new ("MenuActions");
  win->menu_action_group = group;

	gtk_action_group_add_actions( group, entries, 
			G_N_ELEMENTS (entries),
			win->world );

	gtk_action_group_add_toggle_actions( group,toggle_entries, 
			G_N_ELEMENTS(toggle_entries), 
			win->world );

  // Make menus reflect startup state
  gui_sync_menu_toggle_items(win);

	gtk_action_group_add_radio_actions( group, export_format_entries, 
			G_N_ELEMENTS(export_format_entries), 
			win->frame_format, 
			G_CALLBACK(gui_action_export_format),
			win->world );

	gtk_action_group_add_radio_actions( group, export_freq_entries, 
			G_N_ELEMENTS(export_freq_entries),
			win->frame_interval, 
			G_CALLBACK(gui_action_export_interval), 
			win->world );

	gtk_ui_manager_insert_action_group (ui_manager, group, 0);

	// grey-out the radio list labels

#if GTK_CHECK_VERSION(2, 6, 0)
	// gtk+-2.6 code
	gtk_action_set_sensitive(gtk_action_group_get_action( group, "ExportFormat" ), FALSE );
	gtk_action_set_sensitive(gtk_action_group_get_action( group, "ExportInterval" ), FALSE );
#else
	{
		// gtk+-2.4 code
		int sensi = FALSE;
		g_object_set( gtk_action_group_get_action( group, "ExportFormat" ), "sensitive", sensi, NULL );
		g_object_set( gtk_action_group_get_action( group, "ExportInterval" ), "sensitive", sensi, NULL);
	}
#endif

	// accels
	accel_group = gtk_ui_manager_get_accel_group (ui_manager);

	// menus
	gtk_ui_manager_set_add_tearoffs( ui_manager, TRUE );

	GError* error = NULL;
	if (!gtk_ui_manager_add_ui_from_string (ui_manager, ui_description, -1, &error))
	{
		g_message ("building menus failed: %s", error->message);
		g_error_free (error);
		exit (EXIT_FAILURE);
	}


	// install the widget
	menubar = gtk_ui_manager_get_widget (ui_manager, "/Main");


#else   /* GTK 2.0: */

	{
		int menu_entries_count;
		GtkItemFactory *fac = NULL;
		GtkItemFactoryEntry *ep;
		GtkWidget* widget;

		menu_entries_count = 0;
		accel_group = gtk_accel_group_new();
		fac  = gtk_item_factory_new(GTK_TYPE_MENU_BAR, "<main>", accel_group);
    win->menu_item_factory = fac;
		for(ep = menu_entries; ep->path != NULL; ep++, menu_entries_count++);
		gtk_item_factory_create_items(fac, menu_entries_count, menu_entries, (gpointer)(win->world));
		menubar = gtk_item_factory_get_widget(fac, "<main>");

    gui_sync_menu_toggle_items(win);


	}


#endif

  // hide the file loader to start; it will be re-enabled if loader callbacks are added by the user.
  gui_enable_load_file_menu_item(win, FALSE);

  win->canvas->menu_bar = menubar;
  gtk_window_add_accel_group(GTK_WINDOW (win->canvas->frame), accel_group);
  gtk_box_pack_start (GTK_BOX(win->canvas->layout), menubar, FALSE, FALSE, 0);
  
}


void toggle_property_callback( _MENU_TOGGLE_PROPERTY_ACTION_FUNCTION_ARGS )
{
  stg_property_toggle_args_t* args = (stg_property_toggle_args_t*)userdata;  
  model_do_property_toggle(args, _MENU_CHECKITEM_IS_ACTIVE);
}

#if GTK_CHECK_VERSION(2, 4, 0)
static GtkActionGroup* _stg_dynamic_data_action_group = NULL;  
static GtkActionGroup* _stg_dynamic_data_types_action_group = NULL;  
#endif

void stg_model_add_property_toggles( stg_model_t* mod, 
				     const char* propname, 
				     stg_property_callback_t callback_on,
				     void* arg_on,
				     stg_property_callback_t callback_off,
				     void* arg_off,
				     const char* label,
				     int enabled )
{
  STG_F()

  // XXX TODO we need to remove model properties when the model is removed from
  // the world!
  
  stg_property_toggle_args_t* args = 
    calloc(sizeof(stg_property_toggle_args_t),1);
  
  args->mod = mod;
  args->propname = propname;
  args->callback_on = callback_on;
  args->callback_off = callback_off;
  args->arg_on = arg_on;
  args->arg_off = arg_off;


  // add to model's and world's list for access via the library API
  mod->world->model_property_toggles = g_list_append(mod->world->model_property_toggles, (gpointer)args);
  assert(mod->world->model_property_toggles != NULL);
  mod->property_toggles = g_list_append(mod->property_toggles, (gpointer)args);
  assert(mod->property_toggles != NULL);

  
  // optionally add ourselves to the GUI
  if( label && stg_show_menubar )
  {
    gui_lock();
    {

#if GTK_CHECK_VERSION(2, 4, 0)

    if(!ui_manager) return;

    GtkActionGroup* grp = NULL; // Action group for property toggles  (for individual models)
    GtkAction* act = NULL;      // Action for this property (for this model)


    /* Toggle items for each property of each model */

    if( ! _stg_dynamic_data_action_group )
    {
      // create action group the first time if neccesary. we will also create
      // the action below.
      _stg_dynamic_data_action_group = gtk_action_group_new( "DynamicDataActions" );
      grp = _stg_dynamic_data_action_group;
      gtk_ui_manager_insert_action_group(ui_manager, grp, 0);
    }      
    else
    {
      // find the action associated with this propname if it exists
      grp = _stg_dynamic_data_action_group;
      act = gtk_action_group_get_action( grp, propname );
    }

    assert(grp);
      
    // If we newly created the group above, or it exists but the action did not
    // yet exist, create it:
    if( act == NULL )
    {
      //fprintf(stderr, "stage gui_menus: creating new action/item for prop %s on model with token %s\n", propname, mod->token );
      GtkToggleActionEntry entry;
      memset( &entry, 0, sizeof(entry));

      gchar* action_name = g_strconcat(mod->token, "|", propname, NULL); // freed below after adding the action
      gchar* action_label = g_strconcat(label, " for ", mod->token, NULL); // freed below after adding the action
     
      g_strdelimit(action_label, "_", '-'); // GTK uses _ to indicate menu accelerator key. TODO make a new string that "escapes" _.
      entry.name = action_name;
      entry.label = action_label;
      entry.tooltip = "Enable/disable visualization of current data captured by this sensor";
        
      entry.callback = G_CALLBACK(toggle_property_callback);
      entry.is_active = !enabled; // invert the starting setting - see below
      
      gtk_action_group_add_toggle_actions( grp, &entry, 1, args  );   // XXX maybe entry has to be allocated and kept? or does GTK make a copy? (entry is given as a pointer because it's considerred an array (of size 1)
          

    
      guint merge = gtk_ui_manager_new_merge_id( ui_manager );
      gtk_ui_manager_add_ui( ui_manager, 
           merge,
           "/Main/View/ModelPropTogglesPlace", 
           action_name, 
           action_name, 
           GTK_UI_MANAGER_AUTO, 
           FALSE );
      
      act = gtk_action_group_get_action( grp, action_name );
      assert(act);

      // associate UI merge id with action so we can unmerge it later in
      // remove_property_toggles
      guint *copy = malloc(sizeof(guint)); // freed by stg_model_remove_property_toggles().
      *copy = merge;
      g_object_set_data((GObject*)act, "stg_ui_merge_id", copy);

      /* TEST
      memset(entry.name, 'x', strlen(entry.name));
      memset(entry.label, 'x', strlen(entry.label));
      */
      free(action_name);
      free(action_label);  // XXX Are these frees OK? -rh (seem to be)
    }
    else
    {
      //fprintf(stderr, "connecting to signal for existing action for model(token=%s, propname=%s)\n", mod->token, propname );
      g_signal_connect( act, "activate",  G_CALLBACK(toggle_property_callback), args );
    }
        
    // causes the callbacks to be called - un-inverts the starting setting!
    //fprintf(stderr, "activating action.\n");
    gtk_action_activate( act ); 
    //fprintf(stderr, "done activating action\n");



    /* Toggle item for all properties of this type for all models. Add this toggle to the "for all models" action group (creating it if neccesary)  */

    GtkActionGroup* grpall = NULL; // Action group for property type toggles ("for all models")
    GtkAction* actall = NULL;   // Action for this property type ("for all models")
    if( ! _stg_dynamic_data_types_action_group )
    {
      _stg_dynamic_data_types_action_group = gtk_action_group_new("DynamicDataTypesActions");
      grpall = _stg_dynamic_data_types_action_group;
      gtk_ui_manager_insert_action_group(ui_manager, grpall, 0);
    }
    else
    {
      grpall = _stg_dynamic_data_types_action_group;
      actall = gtk_action_group_get_action(grpall, propname);
    }

    assert(grpall);
    // If we newly created the group above, or it exists but the action did not
    // yet exist, create it:
    if( actall == NULL )
    {
      GtkToggleActionEntry entry;
      memset( &entry, 0, sizeof(entry));

      gchar* action_name = strdup(propname); // freed in remove_property_toggles
      gchar* action_label = g_strconcat(label, " for all models", NULL); // freed in  remove_property_toggles
     
      g_strdelimit(action_label, "_", '-'); // GTK uses _ to indicate menu accelerator key
      entry.name = action_name;
      entry.label = action_label;
      entry.tooltip = "Enable/disable visualization of current data captured by all instances of this sensor";
        
      entry.callback = G_CALLBACK(toggle_property_callback);
      entry.is_active = enabled; // invert the starting setting - see below
      
      gtk_action_group_add_toggle_actions( grpall, &entry, 1, args  );   // XXX maybe entry has to be allocated and kept? or does GTK just use its data, not the struct itself?
          
      guint merge = gtk_ui_manager_new_merge_id( ui_manager );
      gtk_ui_manager_add_ui( ui_manager, 
           merge,
           "/Main/View/AllPropTypeTogglesPlace", 
           action_name, 
           action_name, 
           GTK_UI_MANAGER_AUTO, 
           FALSE );  // FALSE makes it insert the item at the end 
      
      actall = gtk_action_group_get_action( grpall, action_name );
      assert(act);

      // associate UI merge id with action so we can unmerge it later in
      // remove_property_toggles
      guint *copy = malloc(sizeof(guint)); // freed by stg_model_remove_property_toggles().
      *copy = merge;
      g_object_set_data((GObject*)actall, "stg_ui_merge_id", copy);

      free(action_label);  // XXX Are these frees OK? -rh (seem to be)

    }
    else
    {
      g_signal_connect( actall, "activate",  G_CALLBACK(toggle_property_callback), args );
    }
  
#else     /* GTK pre-2.4: */

   
    /* Find the View menu and add the new entry */
    GList* menu_list_node;
    GtkMenuBar* menubar = GTK_MENU_BAR(mod->world->win->canvas->menu_bar);
    if(!menubar) return;
    GtkMenuShell* menubarshell = GTK_MENU_SHELL(menubar);
    if(!menubarshell) return;

    for(menu_list_node = menubarshell->children;   
        menu_list_node != NULL;
        menu_list_node = menu_list_node->next)
    {
      //GtkMenu* menu = GTK_MENU(menu_list_node->data);
      GtkMenuItem* menu_item = GTK_MENU_ITEM(menu_list_node->data);
      GtkWidget* menu;

      if(menu_item == NULL) 
      {
        printf("stage: gui_menus: Warning: menu list item is NULL or not a menu item! Skipping.\n");
        continue;
      }

      menu = gtk_menu_item_get_submenu(menu_item);
      if(GTK_MENU(menu) != NULL && strcmp(gtk_widget_get_name(menu), "<main>/View") == 0)
      {
        GtkCheckMenuItem* newitem;
        //printf("XXX Found the view menu! appending a new item\n");
        newitem = GTK_CHECK_MENU_ITEM(gtk_check_menu_item_new_with_label(propname));
        assert(newitem);
        gtk_check_menu_item_set_active(newitem, enabled);
        g_signal_connect((gpointer)newitem, "toggled", G_CALLBACK(toggle_property_callback), (gpointer)args);
        assert(GTK_MENU_SHELL(menu));
        gtk_menu_append(GTK_MENU_SHELL(menu), newitem);
        gtk_widget_show(GTK_WIDGET(newitem));

        // Call the callback manually to communicate new 'checked' state of th echeck item
        toggle_property_callback(GTK_WIDGET(newitem), args);
      }
    }
  
#endif

    }
    gui_unlock();
  }
}

void stg_model_remove_property_toggles( stg_model_t* mod, const char* propname)
{
  STG_F()
  stg_property_toggle_args_t *arg = NULL;
  GList *i = NULL;

  // Find the toggle arg for this model:
  for(i = g_list_first(mod->property_toggles); i != NULL; i = g_list_next(i))
  {
    arg = (stg_property_toggle_args_t*)(i->data);
    if(arg && arg->mod == mod && strcmp(arg->propname, propname) == 0)
      break;
  }

  if(!arg) return;  // none found


  // remove from model's and world's lists
  mod->world->model_property_toggles = g_list_remove(mod->world->model_property_toggles, arg);
  mod->property_toggles = g_list_remove(mod->property_toggles, arg);

  free(arg);

  gui_lock();

#if GTK_CHECK_VERSION(2, 4, 0)

  if(!_stg_dynamic_data_action_group) return;

  // find the action associated with this propname if it exists
  GtkAction *act = gtk_action_group_get_action( _stg_dynamic_data_action_group, propname );
  if(!act) 
  {
    gchar *action_name = g_strconcat(mod->token, "|", propname, NULL);
    act = gtk_action_group_get_action(_stg_dynamic_data_action_group, action_name);
    free(action_name);
  }
  if(!act) return;


  // remove menu items merged in when this property toggle was added:
  if(ui_manager)
  {
    guint *merge_id_p = g_object_get_data((GObject*)act, "stg_ui_merge_id");
    gtk_ui_manager_remove_ui(ui_manager, *merge_id_p);
    free(merge_id_p);
  }

  // remove the action:
  gtk_action_group_remove_action(_stg_dynamic_data_action_group, act);

  // TODO free these??
  //    GtkToggleActionEntry entry = entry for action 'act';
  //    free entry.name;
  //    free entry.label;
        

#else     /* GTK pre-2.4: */


//  XXX XXX TODO XXX XXX 

#if 0
    /* Find the View menu and add the new entry */
    GList* menu_list_node;
    GtkMenuBar* menubar = GTK_MENU_BAR(mod->world->win->canvas->menu_bar);
    GtkMenuShell* menubarshell = GTK_MENU_SHELL(menubar);
    assert(menubarshell);
    for(menu_list_node = menubarshell->children;   
        menu_list_node != NULL;
        menu_list_node = menu_list_node->next)
    {
      //GtkMenu* menu = GTK_MENU(menu_list_node->data);
      GtkMenuItem* menu_item = GTK_MENU_ITEM(menu_list_node->data);
      GtkWidget* menu;

      if(menu_item == NULL) 
      {
        printf("stage: gui_menus: Warning: menu list item is NULL or not a menu item! Skipping.\n");
        continue;
      }

      menu = gtk_menu_item_get_submenu(menu_item);
      if(GTK_MENU(menu) != NULL && strcmp(gtk_widget_get_name(menu), "<main>/View") == 0)
      {
        GtkCheckMenuItem* newitem;
        //printf("XXX Found the view menu! appending a new item\n");
        newitem = GTK_CHECK_MENU_ITEM(gtk_check_menu_item_new_with_label(propname));
        assert(newitem);
        gtk_check_menu_item_set_active(newitem, enabled);
        g_signal_connect((gpointer)newitem, "toggled", G_CALLBACK(toggle_property_callback), (gpointer)args);
        assert(GTK_MENU_SHELL(menu));
        gtk_menu_append(GTK_MENU_SHELL(menu), newitem);
        gtk_widget_show(GTK_WIDGET(newitem));

        // Call the callback manually to communicate new 'checked' state of th echeck item
        toggle_property_callback(GTK_WIDGET(newitem), args);
      }
    }
#endif //TODO
  
#endif

  gui_unlock();
}


void gui_enable_load_file_menu_item(gui_window_t* win, gboolean enable)
{
#if GTK_CHECK_VERSION(2, 4, 0)
  if(!ui_manager) return;
  GtkWidget* menu = gtk_ui_manager_get_widget(ui_manager, "/Main/File/Load");
  if(!menu) return;
  if(enable)
    gtk_widget_show(menu);
  else
    gtk_widget_hide(menu);
#endif

  // Under GTK 2.0 I don't know how to turn it off yet. It's ok, only MobileSim
  // uses GTK 2.0.
}


void gui_sync_menu_toggle_items(gui_window_t* win)
{
  STG_F()
  if(!win) return;

  // Make toggle items reflect startup state:
#if GTK_CHECK_VERSION(2, 4, 0)
  if(!ui_manager) return;
  GtkActionGroup* group = win->menu_action_group;
  if(!group)
    return;
  GtkAction* a = gtk_action_group_get_action(group, "ShowPolygons");
  if(a) gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(a), win->show_polygons);
  a = gtk_action_group_get_action(group, "FillPolygons");
  if(a) gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(a), win->fill_polygons);
  a = gtk_action_group_get_action(group, "Grid");
  if(a) gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(a), win->show_grid);
  a = gtk_action_group_get_action(group, "DebugMatrix");
  if(a) gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(a), win->show_matrix);
  a = gtk_action_group_get_action(group, "DebugGeom");
  if(a) gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(a), win->show_geom);
#else
  GtkItemFactory* fac = win->menu_item_factory;
  GtkWidget* widget;
  if(!fac) return;
  widget = gtk_item_factory_get_widget(fac, "/View/Show polygons");
  assert(widget);
  assert(win);
  gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(widget), win->show_polygons);
  widget = gtk_item_factory_get_widget(fac, "/View/Fill polygons");
  assert(widget);
  gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(widget), win->fill_polygons);
  widget = gtk_item_factory_get_widget(fac, "/View/Grid");
  assert(widget);
  gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(widget), stg_rtk_canvas_is_layer_shown(win->canvas, STG_LAYER_GRID));
#endif
}

