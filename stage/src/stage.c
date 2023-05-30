
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1 // maybe get extensions like sincos()
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>  // declares strcasecmp()
#include <stdarg.h>
#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>
#include <glib.h>
#include <locale.h>
#include <fcntl.h>
#include <time.h>

#include <gdk-pixbuf/gdk-pixbuf.h>

#ifdef WIN32
// for timeGetTime():
#include <windef.h>  
#include <mmsystem.h>
#endif


//#define DEBUG

#include "config.h"
#include "replace.h"
#include "stage_internal.h"

// Old autoconf defines VERSION instead of PACKAGE_VERSION:
#ifndef PACKAGE_VERSION
#ifdef VERSION
#define PACKAGE_VERSION VERSION
#endif
#endif

int _stg_quit = 0;
int _stg_disable_gui = FALSE;

char stg_last_error_message[STG_LOG_MSG_MAX];

int stg_show_messages_view = TRUE;

char stg_last_function[128]; ///< For debugging, since some bugs result in corrupted/meaningless stack trace

/** @mainpage Stage

Stage provides a virtual world populated by mobile robots and sensors,
along with various objects for the robots to sense and manipulate. It
is usually accessed via the Player robot server, but it can also be used
as a stand-alone robot simulation library. Stage is part of the
Player/Stage Project [http://playerstage.sf.net].

To get started using Stage with Player, read the following sections:
 - @ref stage
 - @ref player

Documentation about using libstage to create a custom simulator, or to
modify or add new models, is
available in the following section:
 - @ref libstage

<h2>Design</h2>

<p>Stage was designed with multi-agent systems in mind, so it provides
fairly simple, computationally cheap models of lots of devices rather
than attempting to emulate any device with great fidelity. This design
is intended to be useful compromise between conventional high-fidelity
robot simulations, the minimal simulations described by Jakobi [4], and
the grid-world simulations common in artificial life research [5]. We
intend Stage to be just realistic enough to enable users to move
controllers between Stage robots and real robots, while still being
fast enough to simulate large populations. We also intend Stage to be
comprehensible to undergraduate students, yet sophisticated enough for
professional reseachers.
  
<p>Player also contains several useful 'virtual devices'; including
some sensor pre-processing or sensor-integration algorithms that help
you to rapidly build powerful robot controllers. These are easy to use
with Stage.

[@ref refs]

<h2>Models</h2>

<p>Stage provides several sensor and actuator models, including sonar
or infrared rangers, scanning laser rangefinder, color-blob tracking,
fiducial tracking and mobile robot bases with odometric or global
localization.

<i>Note:
Some models from previous versions may not yet be available in this
release (e.g. gripper & puck), but we're working on them. Let us know which ones you need.
</i>

<h2>Credits</h2>

Stage was written by:

<ul>
<li>Richard Vaughan <tt>(vaughan@sfu.ca)</tt>
<li>Andrew Howard  <tt>(ahoward@robotics.usc.edu)</tt>
<li>Brian Gerkey <tt>(gerkey@robotics.stanford.edu)</tt>
<li>Reed Hedges <tt>(reed@activmedia.com)</tt>
</ul>

Many patches and bug reports have been contributed by users around the
world.

Stage is part of the <a
href="http://playerstage.sourceforge.net">Player/Stage Project</a>, a
community effort to develop Free Software tools for robotics
research. Stage is free to use, modify and redistribute under the
terms of the <a href=http://www.gnu.org/licenses/licenses.html> GNU
General Public License</a>.


If you use Stage in your work, we'd appreciate a citation. At the time of writing, the most suitable reference is:

<ul>
<li>
Brian Gerkey, Richard T. Vaughan and Andrew Howard. "The
Player/Stage Project: Tools for Multi-Robot and Distributed Sensor
Systems"
Proceedings of the 11th International Conference on Advanced Robotics,
pages 317-323,
Coimbra, Portugal, June 2003 <A HREF="http://www.isr.uc.pt/icar03/">(ICAR'03)</A>.
<br>
<A HREF="http://robotics.stanford.edu/~gerkey/research/final_papers/icar03-player.ps.gz">[gzipped
postscript]</A> <A HREF="http://robotics.stanford.edu/~gerkey/research/final_papers/icar03-player.pdf">[pdf]</A>  
</ul>

<p>Please help us keep track of what's being used out there by
correctly naming the Player/Stage components you use. Player and Stage
used together are referred to as "the Player/Stage system" or just
"Player/Stage". When libstage is used without Player, it's just called
"Stage". When Player is used with its 3D ODE-based simulation backend,
Gazebo, it's called Player/Gazebo. Gazebo without Player is just
"Gazebo". All this software is part of the "Player/Stage Project".

<p> Funding for Stage has been provided in part by:

<ul>
 <li>NSERC (Canada)
 <li>Simon Fraser University (Canada)
 <li>DARPA (USA)
 <li>NASA (USA)
 <li>NSF (USA)
</ul>

<p>The names "Player" and "Stage" were inspired by the lines:

@verbatim
 "All the world's a stage"
 "And all the men and women merely players"
@endverbatim

<p>from <i>"As You Like It"</i> by William Shakespeare.

*/


/** @defgroup stage Stage User Guide
 *
 *  The most common way to set up a Stage simulation is to 
 *  write a "world file".  This file contains specifications
 *  of models (e.g. robots and sensors) to create, world properties
 *  such as size and resolution, and GUI preferences.  The world
 *  file can also define specialized model types by giving a name
 *  to a set of property values and child models to use in instances
 *  of the specialized model type.
 *
 *  - @ref stg_world
 *  - @ref stg_model
 *  - @ref window
 *
 *  Stage provides a plugin driver to Player, which provides a means for
 *  client programs to control a robot over a network.  
 *
 *  - @ref player
 */




int stg_init( int argc, char** argv )
{
  STG_F_INIT();

  g_type_init(); // glib GObject initialization

  if( ! _stg_disable_gui )
    {
      // TODO - don't start the GUI if it was disabled
      //puts( "GUI_STARTUP" );x
      gui_startup( &argc, &argv );
    }

  // this forces use of decimal points in the config file rather than
  // euro-style commas. Do this after gui_startup() as GTK messes with
  // locale.
  if(!setlocale(LC_ALL,"POSIX"))
    if(!setlocale(LC_ALL, "C"))
      stg_print_warning("stg_init failed to set locale to \"POSIX\" or \"C\"; config file may not be parse correctly");

  srand(time(NULL));
#ifdef HAVE_SRAND48
  srand48( time(NULL) );
#endif

  memset(stg_last_error_message, 0, sizeof(stg_last_error_message));

  return 0; // ok
}

void stg_uninit()
{
  gui_shutdown();
  if(stg_user_typetable)
  {
    free(stg_user_typetable);
    // TODO free init_properties hash maps in the typetable entries
  }
}
 
void stg_model_all_property_toggles(stg_model_t* mod, gboolean on)
{
  assert(mod);
  if(!mod->property_toggles) return;
  g_list_foreach(mod->property_toggles, 
      on ? model_property_toggles_on_cb : model_property_toggles_off_cb, 
      (gpointer)NULL);
}
 
void model_do_property_toggle(stg_property_toggle_args_t* args, gboolean on)
{
  /// @todo force menu item state to synchronize with what we're doing here:
  assert(args);
  if(!args->mod) return;
  if(!args->propname) return;
  stg_model_lock(args->mod);
  if(on)
  {
    if( args->callback_off )
      stg_model_remove_property_callback( args->mod, 
					    args->propname, 
					    args->callback_off);
    if( args->callback_on )
      stg_model_add_property_callback( args->mod, 
					 args->propname, 
					 args->callback_on, 
					 args->arg_on );
  }
  else
  {
    if( args->callback_on )
      stg_model_remove_property_callback( args->mod, 
					    args->propname, 
					    args->callback_on );
    if( args->callback_off )
      stg_model_add_property_callback( args->mod, 
					   args->propname, 
					   args->callback_off, 
					   args->arg_off );
	}
  stg_model_unlock(args->mod);
}
const char* stg_version_string( void )
{
  return stg_about_info_stageversion;
}

/* const char* stg_model_type_string( stg_model_type_t type ) */
/* { */
/*   switch( type ) */
/*     { */
/*     case STG_MODEL_BASIC: return "model"; */
/*     case STG_MODEL_LASER: return "laser"; */
/*     case STG_MODEL_POSITION: return "position"; */
/*     case STG_MODEL_BLOB: return "blobfinder"; */
/*     case STG_MODEL_FIDUCIAL: return "fiducial"; */
/*     case STG_MODEL_RANGER: return "ranger"; */
/*       //case STG_MODEL_TEST: return "test"; */
/*     case STG_MODEL_GRIPPER: return "gripper"; */
/*     default: */
/*       break; */
/*     }   */
/*   return "<unknown type>"; */
/* } */

void stg_err( const char* err )
{
  stg_print_error("%s", err);
  _stg_quit = 1;
}


void stg_print_geom( stg_geom_t* geom )
{
  printf( "geom pose: (%.2f,%.2f,%.2f) size: [%.2f,%.2f]\n",
	  geom->pose.x,
	  geom->pose.y,
	  geom->pose.a,
	  geom->size.x,
	  geom->size.y );
}

void stg_print_laser_config( stg_laser_config_t* slc )
{
  printf( "slc fov: %.2f  range_min: %.2f range_max: %.2f samples: %d\n",
	  slc->fov,
	  slc->range_min,
	  slc->range_max,
	  slc->samples );
}


stg_msec_t stg_timenow( void )
{
  static stg_msec_t starttime = 0;
  static stg_bool_t no_monotonic = 0;
  stg_msec_t timenow;
  
#ifdef WIN32
  timenow = timeGetTime();
#else
#if defined(HAVE_CLOCK_GETTIME) && defined(_POSIX_TIMERS) && defined(_POSIX_MONOTONIC_CLOCK)
#warning Using monotonic clock via clock_gettime (this is good)
  struct timespec ts;
  if(!no_monotonic && clock_gettime(CLOCK_MONOTONIC, &ts) == 0)
  {
    timenow = (stg_msec_t)(ts.tv_sec*1000.0 + ts.tv_nsec/1e6);
  }
  else
#else // HAVE_CLOCK_GETTIME etc.
#warning no clock_gettime function with posix timers and monotonic clock features, using gettimeofday instead.
#endif // HAVE_CLOCK_GETTIME etc.
  {
    struct timeval tv;
    if(!no_monotonic)
    {
      no_monotonic = 1;
      stg_print_warning("system doesn't have a monotonic clock. falling back on gettimeofday.");
    }
    gettimeofday( &tv, NULL );
    timenow = (stg_msec_t)(tv.tv_sec*1000.0 + tv.tv_usec/1000.0);
  }
#endif // WIN32
  
  //printf("stg_timenow: starttime=%lu, timenow=%lu, returning difference=%lu\n", (unsigned long)starttime, (unsigned long)timenow, (timenow-starttime));

  if( starttime == 0 )
    starttime = timenow;
  
  return( timenow - starttime );
}


// if stage wants to quit, this will return non-zero
int stg_quit_test( void )
{
  return _stg_quit;
}

void stg_quit_request( void )
{
  _stg_quit = 1;
}

void stg_quit_request_code( int code )
{
  _stg_quit = code;
}

void stg_quit_cancel( void )
{
  _stg_quit = 0;
}


// Look up a color in a hard-coded list of common color names.
// This is used on Windows where there is no X11 color database file. 
stg_color_t stg_lookup_color_default(const char* name)
{
   if(strcasecmp(name, "white") == 0) return 0xFFFFFF;
   if(strcasecmp(name, "black") == 0) return 0x000000;
   if(strcasecmp(name, "gray75") == 0) return 0xBFBFBF;
   if(strcasecmp(name, "powder blue") == 0) return 0xB0E0E6;
   if(strcasecmp(name, "steel blue") == 0) return 0x4682B4;
   if(strcasecmp(name, "light steel blue") == 0) return 0xB0C4DE;
   if(strcasecmp(name, "gray85") == 0) return 0xD9D9D9;
   if(strcasecmp(name, "gray90") == 0) return 0xE5E5E5;
   if(strcasecmp(name, "red") == 0) return 0xFF0000;
   if(strcasecmp(name, "green") == 0) return 0x00FF00;
   if(strcasecmp(name, "blue") == 0) return 0x0000FF;
   if(strcasecmp(name, "magenta") == 0) return 0xFF00FF;
   if(strcasecmp(name, "yellow") == 0) return 0xFFFF00;
   if(strcasecmp(name, "cyan") == 0) return 0x00FFFF;
   if(strcasecmp(name, "grey") == 0 || strcasecmp(name, "gray") == 0)
     return 0xBEBEBE;
   if(strcasecmp(name, "lightblue") == 0) return 0xADD8E6;
   if(strcasecmp(name, "light blue") == 0) return 0xADD8E6;
   if(strcasecmp(name, "lightgreen") == 0) return 0x90EE90;
   if(strcasecmp(name, "light green") == 0) return 0x90EE90;
   if(strcasecmp(name, "lightgrey") == 0 || strcasecmp(name, "lightgray") == 0 ||
      strcasecmp(name, "light grey") == 0 || strcasecmp(name, "light gray") == 0)
     return 0xD3D3D3;
   if(strcasecmp(name, "brown") == 0) return 0xA52A2A;
   if(strcasecmp(name, "darkgreen") == 0 || strcasecmp(name, "dark green") == 0)
     return 0x006400;
   if(strcasecmp(name, "orange") == 0) return 0xFFA500;
   if(strcasecmp(name, "purple") == 0) return 0x6F08BE;
   if(strcasecmp(name, "ivory") == 0) return 0xFFFFF0;
   if(strcasecmp(name, "cornflower blue") == 0) return 0x6495ED;
   if(strcasecmp(name, "cornflowerblue") == 0) return 0x6495ED;
   if(strcasecmp(name, "slate blue") == 0) return 0x6A5ACD;
   if(strcasecmp(name, "slateblue") == 0) return 0x6A5ACD;
   if(strcasecmp(name, "mediumblue") == 0 || strcasecmp(name, "medium blue") == 0)
     return 0x000069;
   if(strcasecmp(name, "skyblue") == 0 || strcasecmp(name, "sky blue") == 0)
     return 0x87CEEB;
   if(strcasecmp(name, "lightpink") == 0 || strcasecmp(name, "light pink") == 0)
     return 0xFFB6C1;
   if(strcasecmp(name, "aquamarine") == 0 || strcasecmp(name, "aqua") == 0)
     return 0x7FFFD4;

  
   /* Fall back on black: */
   PRINT_WARN1("Unrecognized color \"%s\". Using black.", name);
   return 0x000000;
}


#ifdef HAVE_COLOR_DATABASE
const char* stg_color_files[5] = {
	COLOR_DATABASE, 
#else
const char* stg_color_files[4] = {
#endif
	"/etc/X11/rgb.txt", 
	"/usr/X11R6/lib/X11/rgb.txt", 
	"/usr/openwin/lib/X11/rgb.txt", 
NULL };


static GHashTable* stg_color_cache = NULL;

// Look up the color in a database.  (i.e. transform color name to
// color value).  If the color is not found in the database, a bright
// red color will be returned instead. If there is no X11 color database
// file, then we will look in a list of known hard-coded names (see above)
// The color will also be cached for future use.
stg_color_t stg_lookup_color(const char *name)
{
  FILE *file = NULL;
  const char *filename = NULL;
  const char **filenamep = NULL;
  gpointer *found;

  
  if( name == NULL ) // no string?
    return 0; // black
  
  if( strlen(name) == 0 ) // empty string?
    return 0; // black

  // create cache table if it doesn't exist yet
  if(stg_color_cache == NULL)
  {
    stg_color_cache = g_hash_table_new(g_str_hash, g_str_equal);
      // note, never freed
  }
  
  // search the cache and return a color if found
  found = g_hash_table_lookup(stg_color_cache, name);
  if(found)
    return (stg_color_t) *found;

  // if not found, find an X11 color file to open
  for(filenamep = stg_color_files; filenamep && *filenamep; filenamep++)
  {
    filename = *filenamep;
    if(strlen(filename) == 0) continue; // skip empty strings
    file = fopen(filename, "r");
    if(file) break;
  }

  if (!file)
  {
    PRINT_DEBUG("unable to open color database file. Trying internal defaults...");
    return stg_lookup_color_default(name);
  }

  PRINT_DEBUG1("using color database file \"%s\".", filename);
  
  // search the file
  while (TRUE)
  {
    char line[1024];
    int r, g, b;
    char *nname;
    int chars_matched;

    if (!fgets(line, sizeof(line), file))
      break;

    // it's a macro or comment line - ignore the line
    if (line[0] == '!' || line[0] == '#' || line[0] == '%') 
      continue;

    // Trim the trailing space
    while (strchr(" \t\n", line[strlen(line)-1]))
      line[strlen(line)-1] = 0;

    // Read the color
    chars_matched = 0;
    sscanf( line, "%d %d %d %n", &r, &g, &b, &chars_matched );
      
    // Read the name
    nname = line + chars_matched;

    // If the name matches
    if (strcmp(nname, name) == 0)
    {
      stg_color_t *c;
      // Close the file, cache the color, and return it.
      fclose(file);
      c = (stg_color_t*)malloc(sizeof(stg_color_t));
      *c = ((r << 16) | (g << 8) | b);
      g_hash_table_insert(stg_color_cache, strdup(name), c);
        // note, memory leak, neither key nor value ever deallocated 
      return *c;
    }
  }
  PRINT_WARN1("unable to find color [%s]; using default (red)", name);
  fclose(file);
  return 0xFF0000;
}




//////////////////////////////////////////////////////////////////////////
// scale an array of rectangles so they fit in a unit square
void stg_lines_normalize( stg_line_t* lines, int num )
{
  double minx, miny, maxx, maxy;
  int l;
  double scalex, scaley;

  // assuming the rectangles fit in a square +/- one billion units
  minx = miny = BILLION;
  maxx = maxy = -BILLION;
  
  for( l=0; l<num; l++ )
    {
      // find the bounding rectangle
      if( lines[l].x1 < minx ) minx = lines[l].x1;
      if( lines[l].y1 < miny ) miny = lines[l].y1;      
      if( lines[l].x1 > maxx ) maxx = lines[l].x1;      
      if( lines[l].y1 > maxy ) maxy = lines[l].y1;
      if( lines[l].x2 < minx ) minx = lines[l].x2;
      if( lines[l].y2 < miny ) miny = lines[l].y2;      
      if( lines[l].x2 > maxx ) maxx = lines[l].x2;      
      if( lines[l].y2 > maxy ) maxy = lines[l].y2;
    }
  
  // now normalize all lengths so that the lines all fit inside
  // rectangle from 0,0 to 1,1
  scalex = maxx - minx;
  scaley = maxy - miny;

  for( l=0; l<num; l++ )
    { 
      lines[l].x1 = (lines[l].x1 - minx) / scalex;
      lines[l].y1 = (lines[l].y1 - miny) / scaley;
      lines[l].x2 = (lines[l].x2 - minx) / scalex;
      lines[l].y2 = (lines[l].y2 - miny) / scaley;
    }
}

void stg_lines_scale( stg_line_t* lines, int num, double xscale, double yscale )
{
  int l;
  for( l=0; l<num; l++ )
    {
      lines[l].x1 *= xscale;
      lines[l].y1 *= yscale;
      lines[l].x2 *= xscale;
      lines[l].y2 *= yscale;
    }
}

void stg_lines_translate( stg_line_t* lines, int num, double xtrans, double ytrans )
{
  int l;
  for( l=0; l<num; l++ )
    {
      lines[l].x1 += xtrans;
      lines[l].y1 += ytrans;
      lines[l].x2 += xtrans;
      lines[l].y2 += ytrans;
    }
}

//////////////////////////////////////////////////////////////////////////
// scale an array of rectangles so they fit in a unit square
void stg_rotrects_normalize( stg_rotrect_t* rects, int num )
{
  double minx, miny, maxx, maxy;
  int r;
  double scalex, scaley;

  // assuming the rectangles fit in a square +/- one billion units
  minx = miny = BILLION;
  maxx = maxy = -BILLION;
  
  for( r=0; r<num; r++ )
    {
      // test the origin of the rect
      if( rects[r].pose.x < minx ) minx = rects[r].pose.x;
      if( rects[r].pose.y < miny ) miny = rects[r].pose.y;      
      if( rects[r].pose.x > maxx ) maxx = rects[r].pose.x;      
      if( rects[r].pose.y > maxy ) maxy = rects[r].pose.y;

      // test the extremes of the rect
      if( (rects[r].pose.x+rects[r].size.x)  < minx ) 
	minx = (rects[r].pose.x+rects[r].size.x);
      
      if( (rects[r].pose.y+rects[r].size.y)  < miny ) 
	miny = (rects[r].pose.y+rects[r].size.y);
      
      if( (rects[r].pose.x+rects[r].size.x)  > maxx ) 
	maxx = (rects[r].pose.x+rects[r].size.x);
      
      if( (rects[r].pose.y+rects[r].size.y)  > maxy ) 
	maxy = (rects[r].pose.y+rects[r].size.y);
    }
  
  // now normalize all lengths so that the rects all fit inside
  // rectangle from 0,0 to 1,1
  scalex = maxx - minx;
  scaley = maxy - miny;

  for( r=0; r<num; r++ )
    { 
      rects[r].pose.x = (rects[r].pose.x - minx) / scalex;
      rects[r].pose.y = (rects[r].pose.y - miny) / scaley;
      rects[r].size.x = rects[r].size.x / scalex;
      rects[r].size.y = rects[r].size.y / scaley;
    }
}	

// returns an array of 4 * num_rects stg_line_t's
stg_line_t* stg_rotrects_to_lines( stg_rotrect_t* rects, int num_rects )
{
  int r;

  // convert rects to an array of lines
  int num_lines = 4 * num_rects;
  stg_line_t* lines = (stg_line_t*)calloc( sizeof(stg_line_t), num_lines );
  
  for( r=0; r<num_rects; r++ )
    {
      lines[4*r].x1 = rects[r].pose.x;
      lines[4*r].y1 = rects[r].pose.y;
      lines[4*r].x2 = rects[r].pose.x + rects[r].size.x;
      lines[4*r].y2 = rects[r].pose.y;
      
      lines[4*r+1].x1 = rects[r].pose.x + rects[r].size.x;;
      lines[4*r+1].y1 = rects[r].pose.y;
      lines[4*r+1].x2 = rects[r].pose.x + rects[r].size.x;
      lines[4*r+1].y2 = rects[r].pose.y + rects[r].size.y;
      
      lines[4*r+2].x1 = rects[r].pose.x + rects[r].size.x;;
      lines[4*r+2].y1 = rects[r].pose.y + rects[r].size.y;;
      lines[4*r+2].x2 = rects[r].pose.x;
      lines[4*r+2].y2 = rects[r].pose.y + rects[r].size.y;
      
      lines[4*r+3].x1 = rects[r].pose.x;
      lines[4*r+3].y1 = rects[r].pose.y + rects[r].size.y;
      lines[4*r+3].x2 = rects[r].pose.x;
      lines[4*r+3].y2 = rects[r].pose.y;
    }
  
  return lines;
}

/// converts an array of rectangles into an array of polygons
stg_polygon_t* stg_polygons_from_rotrects( stg_rotrect_t* rects, size_t count )
{
  stg_polygon_t* polys = stg_polygons_create( count );
  stg_point_t pts[4];
  
  size_t r;
  for( r=0; r<count; r++ )
    {  
      pts[0].x = rects[r].pose.x;
      pts[0].y = rects[r].pose.y;
      pts[1].x = rects[r].pose.x + rects[r].size.x;
      pts[1].y = rects[r].pose.y;
      pts[2].x = rects[r].pose.x + rects[r].size.x;
      pts[2].y = rects[r].pose.y + rects[r].size.y;
      pts[3].x = rects[r].pose.x;
      pts[3].y = rects[r].pose.y + rects[r].size.y;
      
      // copy these points in the polygon
      stg_polygon_set_points( &polys[r], pts, 4 );
    }
  
  return polys;
}


// sets [result] to the pose of [p2] in [p1]'s coordinate system
void stg_pose_sum( stg_pose_t* result, stg_pose_t* p1, stg_pose_t* p2 )
{
  double cosa, sina, tx, ty, ta;

  STG_SINCOS(p1->a, sina, cosa);
  
  tx = p1->x + p2->x * cosa - p2->y * sina;
  ty = p1->y + p2->x * sina + p2->y * cosa;
  ta = p1->a + p2->a;
  
  result->x = tx;
  result->y = ty;
  result->a = ta;
}


// pb_* functions are only used inside this file

guchar* pb_get_pixel( GdkPixbuf* pb, int x, int y )
{
  guchar* pixels = gdk_pixbuf_get_pixels(pb);
  int rs = gdk_pixbuf_get_rowstride(pb);
  int ch = gdk_pixbuf_get_n_channels(pb);
  return( pixels + y * rs + x * ch );
}

void pb_zero_pixel( GdkPixbuf* pb, int x, int y )
{
  // bounds checking
  int width = gdk_pixbuf_get_width(pb);
  int height = gdk_pixbuf_get_height(pb);
  if( x >=0 && x < width && y >= 0 && y < height )
    {
      // zeroing
      guchar* pix = pb_get_pixel( pb, x, y );
      int bytes_per_sample = gdk_pixbuf_get_bits_per_sample (pb) / 8;
      int num_samples = gdk_pixbuf_get_n_channels(pb);
      memset( pix, 0, num_samples * bytes_per_sample );
    }
  else
    PRINT_WARN4( "zero pixel %d,%d out of range (image dimensions %d by %d)", x, y, width, height );
}

// zero all the pixels in a rectangle 
void pb_zero_rect( GdkPixbuf* pb, int x, int y, int width, int height )
{
  //todo - this could be faster - improve it if it gets used a lot)
  int a, b;
  for( a = y; a < y+height; a++ )
    for( b = x; b < x+width; b++ )
      pb_zero_pixel( pb,b,a );
}  

// returns TRUE if any channel in the pixel is non-zero
gboolean pb_pixel_is_set( GdkPixbuf* pb, int x, int y )
{
  guchar* pixel = pb_get_pixel( pb,x,y );
  //int channels = gdk_pixbuf_get_n_channels(pb);

  //int i;
  //for( i=0; i<channels; i++ )
  //if( pixel[i] ) return TRUE;
  if( pixel[0] ) return TRUE; // just use the red channel for now

  return FALSE;
}


stg_polygon_t* stg_polygons_from_image_file(  const char* filename, 
					     size_t* count )
{
  stg_rotrect_t* rects = NULL;
  int rect_count = 0;
  stg_polygon_t* polys  = NULL;

  if( stg_rotrects_from_image_file( filename,  
				    &rects,
				    &rect_count,
				    NULL, NULL ) )
    {
      PRINT_ERR1( "failed to load rects from image file \"%s\"",
		  filename );      
      return NULL;
    }

  //printf( "found %d rects\n", rect_count );
  // else

  *count = (size_t)rect_count;
  polys = stg_polygons_from_rotrects( rects, rect_count );
  free(rects);
  return polys;
}


int stg_rotrects_from_image_file( const char* filename, 
				  stg_rotrect_t** rects, 
				  int* rect_count,
				  int* widthp, int* heightp )
{
  int s;
  GError* err = NULL;
  GdkPixbuf* pb = gdk_pixbuf_new_from_file( filename, &err );
  if( err )
  {
    stg_print_warning("Error loading bitmap file \"%s\": %s\n", filename, err->message );
    return 1; // error
  }
  s = stg_rotrects_from_image_pixbuf(pb, rects, rect_count, widthp, heightp);
  gdk_pixbuf_unref( pb ); // free the image data
  return s;
}

int stg_rotrects_from_image_pixbuf(GdkPixbuf* pb,
				  stg_rotrect_t** rects, 
				  int* rect_count,
				  int* widthp, int* heightp )
{
  int img_width, img_height;
  int y, x;

  // this should be ok as no error was reported
  assert( pb );
  
#ifdef DEBUG
  printf( "image \"%s\" channels:%d bits:%d alpha:%d "
	  "width:%d height:%d rowstride:%d pixels:%p\n",
	  
	  filename,
	  gdk_pixbuf_get_n_channels(pb),
	  gdk_pixbuf_get_bits_per_sample(pb),
	  gdk_pixbuf_get_has_alpha(pb),	      
	  gdk_pixbuf_get_width(pb),
	  gdk_pixbuf_get_height(pb),
	  gdk_pixbuf_get_rowstride(pb),
	  gdk_pixbuf_get_pixels(pb) );
#endif

  *rect_count = 0;
  *rects = NULL;
  
  img_width = gdk_pixbuf_get_width(pb);
  img_height = gdk_pixbuf_get_height(pb);
  
  // if the caller wanted to know the dimensions
  if( widthp ) *widthp = img_width;
  if( heightp ) *heightp = img_height;
  
  for(y = 0; y < img_height; y++)
    {
      for(x = 0; x < img_width; x++)
	{
    int startx, starty, height;

	  // skip blank pixels
	  if( ! pb_pixel_is_set( pb,x,y) )
	    continue;
	  
	  // a rectangle starts from this point
	  startx = x;
	  starty = y;
	  height = img_height; // assume full height for starters
	  
	  // grow the width - scan along the line until we hit an empty pixel
	  for( ; x < img_width &&  pb_pixel_is_set(pb,x,y); x++ )
	    {
        int yy;

	      // handle horizontal cropping
	      //double ppx = x * sx; 
	      //if (ppx < this->crop_ax || ppx > this->crop_bx)
	      //continue;
	      
	      // look down to see how large a rectangle below we can make
	      yy  = y;
	      while( pb_pixel_is_set(pb,x,yy) && (yy < img_height-1) )
		{ 
		  // handle vertical cropping
		  //double ppy = (this->image->height - yy) * sy;
		  //if (ppy < this->crop_ay || ppy > this->crop_by)
		  //continue;
		  
		  yy++; 
		} 	      

	      // now yy is the depth of a line of non-zero pixels
	      // downward we store the smallest depth - that'll be the
	      // height of the rectangle
	      if( yy-y < height ) height = yy-y; // shrink the height to fit
	    } 
	  
	  // delete the pixels we have used in this rect
	  pb_zero_rect( pb, startx, starty, x-startx, height );
	  
	  // add this rectangle to the array
	  (*rect_count)++;
	  *rects = (stg_rotrect_t*)
	    realloc( *rects, *rect_count * sizeof(stg_rotrect_t) );
	  
#if 0 /* Unused? */
	  stg_rotrect_t *latest = &(*rects)[(*rect_count)-1];
	  latest->pose.x = startx;
	  latest->pose.y = starty;
	  latest->pose.a = 0.0;
	  latest->size.x = x - startx;
	  latest->size.y = height;
	  
	  //printf( "rect %d (%.2f %.2f %.2f %.2f %.2f\n", 
	  //  *rect_count, 
	  //  latest->x, latest->y, latest->a, latest->w, latest->h ); 
#endif
	  
	}
    }
  

  // now y-invert all the rectangles because we're using conventional
  // rather than graphics coordinates. this is much faster than
  // inverting the original image.
  {
    int r;
    for( r=0; r< *rect_count; r++ )
      {
        stg_rotrect_t *rect = &(*rects)[r]; 
        rect->pose.y = img_height - rect->pose.y;
        rect->size.y = -rect->size.y;
      }
  }
    

  return 0; // ok
}

// POINTS -----------------------------------------------------------

stg_point_t* stg_points_create( size_t count )
{
  return( (stg_point_t*)calloc( count, sizeof(stg_point_t)));
}

void stg_points_destroy( stg_point_t* pts )
{
  free( pts );
}

// POLYGONS -----------------------------------------------------------

/// return an array of [count] polygons. Caller must free() the space.
stg_polygon_t* stg_polygons_create( int count )
{
  int p;
  stg_polygon_t* polys = (stg_polygon_t*)calloc( count, sizeof(stg_polygon_t));
  
  // each polygon contains an array of points
  for( p=0; p<count; p++ )
    polys[p].points = g_array_new( FALSE, TRUE, sizeof(stg_point_t));

  return polys;
}

/// destroy (free) the polygons in an array of polygons (but does not free the
/// array itself)
void stg_polygons_destroy( stg_polygon_t* p, size_t count )
{
  size_t c;
  PRINT_DEBUG2("stg_model_destroy_polygons: destroying %lu polygons from 0x%x", count, p);
  for( c=0; c<count; c++ )
    if( p[c].points )
      g_array_free( p[c].points, TRUE );
  
  //free( p );    // rh removed, results in double-free when trying to destroy a property containing polygons (because the stg_polygon_t is stored as the property data)
}

stg_polygon_t* stg_unit_polygon_create( void )
{
  stg_point_t pts[4];
  stg_polygon_t* poly;

  pts[0].x = 0;
  pts[0].y = 0;
  pts[1].x = 1;
  pts[1].y = 0;
  pts[2].x = 1;
  pts[2].y = 1;
  pts[3].x = 0;
  pts[3].y = 1;  
  
  poly = stg_polygons_create(1);
  stg_polygon_set_points( poly, pts, 4 );  
  return poly;
}

// scale an array of points
void stg_points_normalize(stg_point_t* points, int num, double width, double height)
{
  double minx, miny, maxx, maxy;
  int p;
  double scalex, scaley;

  // assuming the size is less than 2 BILLION big
  minx = miny = BILLION;
  maxx = maxy = -BILLION;
  
  for(p=0; p<num; ++p) 
  {
      if( points[p].x < minx ) minx = points[p].x;
      if( points[p].y < miny ) miny = points[p].y;
      if( points[p].x > maxx ) maxx = points[p].x;
      if( points[p].y > maxy ) maxy = points[p].y;	  
  }      

  // now normalize all positions so that the lines all fit inside
  // the specified rectangle
  scalex = (maxx - minx);
  scaley = (maxy - miny);

  for(p=0; p<num; ++p)
  { 
      points[p].x = ((points[p].x - minx) / scalex * width) - width/2.0;
      points[p].y = ((points[p].y - miny) / scaley * height) - height/2.0;
  }
}

//////////////////////////////////////////////////////////////////////////
// scale an array of polygons so they fit in a rectangle of size
// [width] by [height], with the origin in the center of the rectangle
void stg_polygons_normalize( stg_polygon_t* polys, int num, 
			     double width, double height )
{
  double minx, miny, maxx, maxy;
  int l;
  double scalex, scaley;

  // assuming the rectangles fit in a square +/- one billion units
  minx = miny = BILLION;
  maxx = maxy = -BILLION;
  
  for( l=0; l<num; l++ ) // examine all the polygons
    {
      // examine all the points in the polygon
      int p;
      for( p=0; p<polys[l].points->len; p++ )
	{
	  stg_point_t* pt = &g_array_index( polys[l].points, stg_point_t, p);
	  if( pt->x < minx ) minx = pt->x;
	  if( pt->y < miny ) miny = pt->y;
	  if( pt->x > maxx ) maxx = pt->x;
	  if( pt->y > maxy ) maxy = pt->y;	  
	}      
    }
  
  // now normalize all lengths so that the lines all fit inside
  // the specified rectangle
  scalex = (maxx - minx);
  scaley = (maxy - miny);
  
  for( l=0; l<num; l++ ) // scale each polygon
    { 
      // scale all the points in the polygon
      int p;
      for( p=0; p<polys[l].points->len; p++ )
	{
	  stg_point_t* pt = &g_array_index( polys[l].points, stg_point_t, p);
	  
	  pt->x = ((pt->x - minx) / scalex * width) - width/2.0;
	  pt->y = ((pt->y - miny) / scaley * height) - height/2.0;
	}
    }
}

void stg_polygon_print( stg_polygon_t* poly )
{
  int i;
  printf( "polygon: %d pts : ", poly->points->len );
  
  for(i=0;i<poly->points->len;i++)
    {
      stg_point_t* pt = &g_array_index( poly->points, stg_point_t, i );
      printf( "(%.2f,%.2f) ", pt->x, pt->y );
    }
  puts("");
}

void stg_polygons_print( stg_polygon_t* polys, unsigned int count )
{
  int i;
  printf( "polygon array (%d polys)\n", count );
  
  for( i=0; i<count; i++ )
    {
      printf( "[%d] ", i ); 
      stg_polygon_print( &polys[i] );
    }
}


/// Copies [count] points from [pts] into polygon [poly], allocating
/// memory if mecessary. Any previous points in [poly] are
/// overwritten.
void stg_polygon_set_points( stg_polygon_t* poly, stg_point_t* pts, size_t count )
{
  assert( poly );
  
  g_array_set_size( poly->points, 0 );
  g_array_append_vals( poly->points, pts, count );
}

/// Appends [count] points from [pts] to the point list of polygon
/// [poly], allocating memory if mecessary.
void stg_polygon_append_points( stg_polygon_t* poly, stg_point_t* pts, size_t count )
{
  assert( poly );
  g_array_append_vals( poly->points, pts, count );
}


stg_radians_t stg_intersection_angle(stg_radians_t a, stg_radians_t b)
{
  stg_radians_t i = (M_PI/2.0) + a - b;
  while(i > (M_PI/2.0)) i -= M_PI;
  while(i < -(M_PI/2.0)) i += M_PI;
  return i;
}

// LOGGING --------------------------------------------------------------------
  
static stg_print_format_t stg_print_format = STG_PRINT_COLOR_TEXT;

FILE* stg_output_file = NULL;
size_t stg_output_file_max = 0;
size_t stg_output_file_sz = 0;
stg_log_file_full_callback_t stg_output_file_full_cb = NULL;
gboolean stg_in_output_file_full_cb = FALSE;

/** Set an output file to write log message to instead of stderr */
void stg_set_log_file(FILE* file) { stg_output_file = file; }
void stg_flush_log_file() { if(stg_output_file) fflush(stg_output_file); }
void stg_close_log_file()
{
  if(!stg_output_file) return;
  fclose(stg_output_file);
  stg_output_file = NULL;
}

FILE* stg_open_log_file(const char *filename, const char *mode)
{
  stg_output_file = fopen(filename, mode);
  if(!stg_output_file) return NULL;
#ifndef WIN32
  int fd = fileno(stg_output_file);
  int flags = fcntl(fd, F_GETFD);
  if(flags >= 0)
  {
    fcntl(fd, F_SETFD, flags|FD_CLOEXEC);
  }
#endif
  return stg_output_file;
}

void stg_set_log_file_max_size(size_t max, stg_log_file_full_callback_t callback)
{
    stg_output_file_max = max;
    stg_output_file_sz = 0; // if max was not set, could have been growing to huge value over time
    stg_output_file_full_cb = callback;
}

/** Internal function to do the printing in either text or HTML format, with
 *  leading timestamp, optionally also to a file.
 * @param level_str String to display at start of message after timestamp as a
 * "level" indicator. Furthermore, if outputing text format, and this string is 
 * "Error", then "Error" will
 * be displayed with red background, and if the string is "Warning", 
 * then "Warning" will be displayed with blue background. If outputing HTML
 * format, then the level string is displayed within a <span> with class
 * of "stage-log-[level_str]" (e.g. "stage-log-Error"), or simply "stage-log"
 * if level_str is NULL.
 * @param msg Message (a printf format string)
 * @param args Arguments to format string.
 * If a log file has been set with stg_set_log_file(), then messages will be printed to the file, otherwise it will be printed to stderr.
 */
void stg_print_output_with_time(const char* level_str, const char* msg, int have_args, va_list args)
{
  time_t now = time(NULL);
  char* time_str = ctime(&now);
  int ansi_color = -1;
  int r = 0;
  time_str[strlen(time_str)-1] = '\0';  // clobber newline
#ifdef WIN32
  if(stg_output_file == NULL) stg_output_file = stdout; // only stdout is printed in windows command shell
#else
  if(stg_output_file == NULL) stg_output_file = stderr;
#endif
  switch(stg_print_format)
  {

#ifndef WIN32
    case STG_PRINT_COLOR_TEXT:
      if(level_str && getenv("TERM") != NULL)
      {
        if(strcmp(level_str, "Error") == 0)
          ansi_color = 41;  //red
        else if(strcmp(level_str, "Warning") == 0)
          ansi_color = 44;  // blue
      }
#endif

    case STG_PRINT_PLAIN_TEXT:
      if(level_str)
      {
        if(ansi_color == -1)
          r = fprintf(stg_output_file, "%s %s: ", time_str, level_str);
        else
          r = fprintf(stg_output_file, "%s \033[%dm%s\033[0m: ", time_str, ansi_color, level_str);
      }
      else
      {
        r = fprintf(stg_output_file, "%s ", time_str);
      }
      if(r < 0)
      {
        fprintf(stderr, "stage: Warning: Error writing log message!");
        if(stg_output_file != stderr && stg_output_file != stdout) 
          fclose(stg_output_file);
        r = 0;
        break;
      }
        
      if(have_args)
        r += vfprintf(stg_output_file, msg, args);
      else
        r += fputs(msg, stg_output_file);
      r += fputc('\n', stg_output_file);
      break;

    case STG_PRINT_HTML:
      if(level_str)
        r = fprintf(stg_output_file, "<span class=\"stage-log-%s\"><span class=\"stage-log-time\">%s</span> <span class=\"stage-log-%s-tag\">%s</span>: <span class=\"stage-log-msg\">", level_str, time_str, level_str, level_str);
      else
        r = fprintf(stg_output_file, "<span class=\"stage-log\"><span class=\"stage-log-time\">%s</span> <span class=\"stage-log-msg\">", time_str);

      if(r < 0)
      {
        fprintf(stderr, "stage: Warning: Error writing log message!");
        if(stg_output_file != stderr && stg_output_file != stdout) 
          fclose(stg_output_file);
        r = 0;
        break;
      }

      if(have_args)
        r += vfprintf(stg_output_file, msg, args);
      else
        r += fputs(msg, stg_output_file);

      r += fputs("</span></span><br/>\n", stg_output_file);

      break;
  }
  fflush(stg_output_file);
  stg_output_file_sz += r;
  //printf("stg message output: printed message. now sz=%lu, max=%lu, (sz>=max?->%d) fp=0x%x (stdout?->%d, stderr?->%d, stdout=0x%x, stderr=0x%x), cb=0x%x in_full_cb=%d\n", stg_output_file_sz, stg_output_file_max, (stg_output_file_sz >= stg_output_file_max), stg_output_file, (stg_output_file == stdout), (stg_output_file == stderr), stdout, stderr, stg_output_file_full_cb, stg_in_output_file_full_cb);
  if(stg_output_file_max != 0 && stg_output_file != stdout && stg_output_file != stderr
     && stg_output_file_sz >= stg_output_file_max && stg_output_file_full_cb != NULL && !stg_in_output_file_full_cb)
  {
     //puts("-> file is full, calling callback.");
     //fflush(stdout);
     stg_in_output_file_full_cb = TRUE;
     (*stg_output_file_full_cb)(stg_output_file, stg_output_file_sz, stg_output_file_max);
     stg_in_output_file_full_cb = FALSE;
     stg_output_file_sz = 0;
     //printf("done, sz=%lu, max=%lu\n", stg_output_file_sz, stg_output_file_max);
     //fflush(stdout);
  }
}

void stg_print_output_with_time_v(const char* level_str, const char* msg, va_list args)
{
  stg_print_output_with_time(level_str, msg, 1, args);
}

void stg_print_output_with_time_s(const char* level_str, const char* msg)
{
  va_list dummy;
  stg_print_output_with_time(level_str, msg, 0, dummy);
}

void stg_print_error(const char* m, ...)
{
  va_list args;
  va_start(args, m);
  char buf[STG_LOG_MSG_MAX];
  vsnprintf(buf, STG_LOG_MSG_MAX, m, args);
  va_end(args);
  stg_print_error_s(buf);
}

void stg_print_warning(const char* m, ...)
{
  va_list args;
  va_start(args, m);
  char buf[STG_LOG_MSG_MAX];
  vsnprintf(buf, STG_LOG_MSG_MAX, m, args);
  va_end(args);
  stg_print_warning_s(buf);
}

void stg_print_msg(const char* m, ...)
{
  va_list args;
  va_start(args, m);
  char buf[STG_LOG_MSG_MAX];
  vsnprintf(buf, STG_LOG_MSG_MAX, m, args);
  va_end(args);
  stg_print_msg_s(buf);
}

void stg_print_message(const char* m, ...)
{
  va_list args;
  va_start(args, m);
  char buf[STG_LOG_MSG_MAX];
  vsnprintf(buf, STG_LOG_MSG_MAX, m, args);
  va_end(args);
  stg_print_msg_s(buf);
}

void stg_print_error_v(const char* m, va_list args)
{
  vsnprintf(stg_last_error_message, sizeof(stg_last_error_message), m, args);
  stg_print_output_with_time_v("Error", m, args);
}

void stg_print_error_s(const char *m)
{
  strncpy(stg_last_error_message, m, sizeof(stg_last_error_message));
  stg_print_output_with_time_s("Error", m);
}

void stg_print_msg_s(const char *m)
{
  stg_print_output_with_time_s(NULL, m);
}

void stg_print_warning_v(const char* m, va_list args)
{
  stg_print_output_with_time_v("Warning", m, args);
}

void stg_print_warning_s(const char *m)
{
  stg_print_output_with_time_s("Warning", m);
}

void stg_print_msg_v(const char* m, va_list args)
{
  stg_print_output_with_time_v(NULL, m, args);
#ifdef WIN32
  vprintf(m, args);
  puts("");
#endif
}


void stg_set_print_format(stg_print_format_t fmt)
{
  stg_print_format = fmt;
}


// MISC API ------------------------------------------------------------------

void stg_register_custom_type(char* type_name, stg_model_initializer_t init_func, GData *init_properties)
{
  stg_type_record_t* r;
  int n = 0;

  // initial allocation?
  if(!stg_user_typetable)
  {
    stg_user_typetable = malloc(2 * sizeof(stg_type_record_t));
    stg_user_typetable[0].keyword = type_name;
    stg_user_typetable[0].initializer = init_func;
    stg_user_typetable[1].keyword = NULL;
    stg_user_typetable[1].initializer = NULL;
    return;
  }

  // find end of list for current size
  r = stg_user_typetable;
  while(r->keyword)
  {
    r++;
    n++;
  }

  // reallocate new space for n existing items, plus one new item, plus one terminator
  stg_user_typetable = (stg_type_record_t*) realloc( stg_user_typetable, n+2 * sizeof(stg_type_record_t) );

  // Add item and new terminator item:
  stg_user_typetable[n].keyword = type_name;
  stg_user_typetable[n].initializer = init_func;
  stg_user_typetable[n].init_properties = init_properties;
  stg_user_typetable[n+1].keyword = NULL;
  stg_user_typetable[n+1].initializer = NULL;
  stg_user_typetable[n+1].init_properties = NULL;
}
