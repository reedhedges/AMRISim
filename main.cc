/*  
 *  Copyright (C) 2005 ActivMedia Robotics
 *  Copyright (C) 2006-2010 MobileRobots Inc.
 *  Copyright (C) 2011-2015 Adept Technology
 *  Copyright (C) 2016-2017 Omron Adept Technologies
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

#define _BSD_SOURCE 1	// to get getcwd() from Linux's <unistd.h> 

#include <stage.h>

#ifdef VERSION
#define STAGE_VERSION VERSION
#undef VERSION
#endif

#include <assert.h>

#ifndef MOBILESIM_NOGUI
#include <gtk/gtk.h>
#include <glib.h>
#endif

#include <map>
#include <set>
#include <locale.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <limits.h>
#include <unistd.h>
#include <signal.h>


#include "ariaUtil.h"
#include "ArRobotParams.h"
#include "ArMap.h"
#include "ArSocket.h"

#include "MobileSim.hh"
#include "Config.hh"
#include "util.h"
#include "StageInterface.hh"
#include "EmulatePioneer.hh"
#include "RobotFactory.hh"
#include "StageRobotFactory.hh"
#include "MapLoader.hh"
#include "Socket.hh"
#include "NetworkDiscovery.hh"


using namespace MobileSim;

#include "ArSocket.h"

#include <stdio.h>
// if defined, measure time it takes to do world updates in main loop
//#define DEBUG_LOOP_TIME 1

// if defined, delete the objects on program
// exit. usually not really neccesary since the program is exiting anyway, and
// is not fully debugged (it can crash)
//#define DELETE_EMULATORS 1

// if defined, delete all robot interface objects on program exit.
// usually not really neccesary since the program is exiting anyway, 
// and is not fully debugged (it can crash)
//#define DELETE_ROBOT_INTERFACES 1

// if defined, destroy stage world and call uninit at program
// exit. unually not really neccesary since the program is exiting
// anyway, and causes crashes on windows.
//#define STAGE_UNINIT 1


#define SHOW_WORLD_LOADING_DIALOG 1


/* Arbitrary limit for number of robots allowed in the initial
 * configuration dialog.  Theoretically, we're only limited by
 * the number of TCP ports possible (1024 through 65545).
 * MobileSim will probably become more and more unusable, however, 
 * the closer you approach this limit.
 */
#define ROBOTS_MENU_LIMIT 200

/* If undefined, temp files won't be deleted. Useful 
   for debugging them. 
   */
#define CLEANUP_TEMP_FILES 1


#define COPYRIGHT_TEXT "Stage 2.0 is (C) Copyright 2007  Richard Vaughan, Brian Gerkey, Andrew Howard, and others.\n" \
"MobileSim is (C) Copyright 2005, ActivMedia Robotics LLC, (C) Copyright 2006-2010 MobileRobots Inc.,\n" \
"(C) Copyright 2011-2015, Adept Technology Inc., (C) Copyright 2016-2017 Omron Adept Technologies.\n" \
"This software is released under the GNU General Public License.\n" \
"This software comes with NO WARRANTY.  This is free software, and you are\n" \
"welcome to redistribute it under certain conditions; see the LICENSE.txt file\n" \
"for details.\n\n"

#ifndef MOBILESIM_DEFAULT_DIR

#ifdef WIN32
#define MOBILESIM_DEFAULT_DIR "\\Program Files\\MobileRobots\\MobileSim"
#define MOBILESIM_DEFAULT_HELP_URL "file:C:/Program%20Files/MobileRobots/MobileSim/README.html";
#elif defined(__APPLE__)
#define MOBILESIM_DEFAULT_DIR "/usr/local/MobileSim" // TODO this should be inside MobileSim.app bundle directory
#define MOBILESIM_DEFAULT_HELP_URL "file:///usr/local/MobileSim/README.html" // TODO this should be inside MobileSim.app bundle directory
#else
#define MOBILESIM_DEFAULT_DIR "/usr/local/MobileSim"
#define MOBILESIM_DEFAULT_HELP_URL "file:///usr/local/MobileSim/README.html" 
#endif

#endif

/* Remember the world file we created so we can delete it */
char TempWorldFile[MAX_PATH_LEN];


/* Some predefined ActivMedia robot models for use in the GUI */
const char* CommonRobotModels[] = {
  "p3dx-sh-lms1xx", 
  "p3dx-sh-lms200",
  "p3dx-sh", 
  "p3at-sh-lms1xx", 
  "p3at-sh-lms200", 
  "p3at-sh", 
  "amigo-sh",
  "amigo-sh-tim3xx",
  "powerbot-sh", 
  "peoplebot-sh", 
  "seekur", 
  "seekurjr", 
  "pioneer-lx",
  "research-patrolbot", 
  "p2de", 
  "p2at", 
  "p3atiw-sh", 
  "p3dx-no-error",
  "p3dx-big-rot-error",
  "p3dx-big-x-error",
  0
};



/* Type used to store what simulated robot instances to create. Map from unique robot
 * name to model or .p file.
 */
typedef std::map<std::string, std::string> RobotModels;

/* Type to store what factories to create.*/
typedef  std::list<std::string> RobotFactoryRequests;

/* Used to load maps */
MapLoader mapLoader;

ArMap *map = NULL;

unsigned long MobileSim::log_stats_freq = 0;

/* Create a temporary world file and load it. Return NULL on error */
stg_world_t* create_stage_world(const char* mapfile, 
  /*std::map<std::string, std::string>& robotInstanceRequests, std::list<std::string>& robotFactoryRequests,*/ 
    const char* libdir, 
    //mobilesim_start_place_t start, 
    //double start_override_x, double start_override_y, double start_override_th, 
    double world_res = 0.0,  
    void (*loop_callback)() = NULL);


/* Figure out what directory to use for our external resources */
const char* find_libdir();

/* Delete temporary files: */
void cleanup_temp_files();

/* Utility to get data out of a GHashTable */
/* unused
typedef std::map<std::string, std::pair<std::string, std::string> > RobotTypeMap;
void add_model_to_robot_type_map_if_position(stg_model_t* model, char* model_name, void* map_p);
*/



#ifndef MOBILESIM_NOGUI

#ifdef GTK_CHECK_VERSION

#if GTK_CHECK_VERSION(2, 4, 0)
# define GTKFILEDIALOG GtkFileChooserDialog
# define GTKFILECHOOSER GtkFileChooser
# define GTKCOMBOWIDGET GtkComboBox
# define USE_GTKFILECHOOSER 1
# define USE_GTKCOMBOBOX 1
# define USE_GTKEXPANDER 1
#else
# define GTKFILEDIALOG GtkFileSelection
# define GTKFILECHOOSER GtkFileSelection
# define GTKCOMBOWIDGET GtkCombo
#endif

#else

#warning GTK_CHECK_VERSION not defined

# define GTKFILEDIALOG GtkFileChooserDialog
# define GTKFILECHOOSER GtkFileChooser
# define GTKCOMBOWIDGET GtkComboBox
# define USE_GTKFILECHOOSER 1
# define USE_GTKCOMBOBOX 1
# define USE_GTKEXPANDER 1

#endif


/* Show the map/options dialog displayed at startup if no map is
 * given on the command line. Exit the program if the dialog is closed.
 * If robotInstanceRequests is NULL, do not show any robot model or num. robots selection
 * controls. If not NULL, then robot model selection and number controls are
 * shown, and entries are added to the map accordingly.
 * @return 0 if window is closed (cancelled), 1 if map was selected, 2 if "no map" was chosen
 */
int map_options_dialog(std::string& map, RobotModels* robotInstanceRequests, RobotFactoryRequests *robotFactoryRequests, MobileSim::Options *opts);

int map_options_dialog(std::string& map, MobileSim::Options *opts) {
  return map_options_dialog(map, NULL, NULL, opts);
}

// show "busy loading world" dialog and return.  hide and destroy the widget to hide the dialog.
GtkWidget *busy_loading_dialog();

#endif

/* Print help info */

void usage()
{
  puts(
/*                                                                   column 80--v */
"MobileSim " MOBILESIM_VERSION "\n"
"Usage: MobileSim [-m <map>] [-r <robot model> ...] [...options...]\n\n"
"  --map <map>      : Load map file (e.g. created with Mapper3)\n"
"  -m <map>         : Same as -map <map>.\n"
"  --nomap          : Create an \"empty\" map to start with.\n"
"                     Same as -map \"\". Ignored if -m is also given.\n"
"  --robot <model>[:<name>] :\n"
"                     Create a simulated robot of the given model.\n"
"                     If an ARIA robot parameter file is given for <model>,\n"
"                     then MobileSim attempts to load that file from the\n"
"                     current working directory, and create an equivalent\n"
"                     model definition. May be repeated with different names\n"
"                     and models to create multiple simulated robots. Name \n"
"                     portion is optional.\n"
"                     Example: -robot p3dx -robot amigo:bot1 -robot custom.p:bot2\n"
"                     See PioneerRobotModels.world.inc for model definitions.\n"
"  -r <model>[:name]: Same as --robot <model>[:<name>]\n"
"  --robot-factory <model> :\n"
"                     Instead of creating one robot of the given model, accept\n"
"                     any number of client programs on its port, creating a new\n"
"                     instance of the model for each client, and destroying it\n"
"                     when the client disconnects.\n"
"  -R <model>       : Same as --robot-factory <model>\n"
"  -p <port>        : Emulate Pioneer connections starting with TCP port <port>.\n"
"                     (Default: 8101)\n"
#ifndef MOBILESIM_NOGUI
"  --fullscreen-gui : Display window in fullscreen mode.\n"
"  --maximize-gui   : Display window maximized.\n"
"  --minimize-gui   : Display window minimized (iconified)\n"
#endif
#ifdef WIN32
"  --noninteractive : Don't display any interactive dialog boxes that might\n"
"                     block program execution, and enable log rotation after\n"
"                     default maximum log file size of 10MB\n"
#else
"  --noninteractive : Don't display any interactive dialog boxes that might\n"
"                     block program execution, enable automatic crash-restart,\n"
"                     and enable log rotation after default maximum log file\n"
"                     size of 10MB\n"
"  --daemonize      : Run as a daemon (background process) after initialization\n"
"                     (still creates GUI). Forces noninteractive mode.  \n"
"                     Not available on Windows.\n"
#endif
#ifndef MOBILESIM_NOGUI
"  --lite-graphics  : Disable some graphics for slightly better performance\n"
"  --no-graphics    : Disable all graphics drawing for slightly better\n"
"                     performance\n"\
"  --no-gui         : Disable GUI entirely. Automatically enables noninteractive\n"
"                     mode as well.\n"
#endif
"  --html           : Print log messages and other output in HTML rather than\n"
"                     plain text\n"
"  --cwd <dir>      : Change directory to <dir> at startup.  Client programs\n"
"                     can then load maps relative to this directory.\n"
"  --log-file <file>: Print log messages to <file> instead of standard error.\n"
"  -l <file>        : Same as --log-file <file>.\n"
"  --log-file-max-size <size> :\n"
"                     If the amount of data (bytes) written to the log file\n"
"                     exceeds <size>, then rotate the files (up to 5) and open\n"
"                     a new file. This keeps the total size of the log files\n"
"                     under <size>*5 bytes.  If --noninteractive is given,\n"
"                     default is 5 MB (5*1024^2 bytes).\n"
"                     If --noninteractive is not given, or <size> is 0, no\n"
"                     limit is used and logs won't automatically rotate.\n"
"  --update-interval <ms> : \n"
"                     Time between each simulation update step.  Default is 100\n"
"                     ms. Less may improve simulation accuracy but impact client\n"
"                     responsiveness and data update; more may reduce CPU load\n"
"                     but impact accuracy (especially movement resolution).\n"
"  --update-sim-time <ms> : \n"
"                     How much simulated time each simulation update takes.\n"
"                     Default is equal to --update-interval.\n"
"  --start <x>,<y>,<th> OR --start outside OR --start random :\n"
"                     Use <x>, <y>, <th> (mm and degrees) as robot starting\n"
"                     point (even if the map has Home objects). Or, use the\n"
"                     keyword \"outside\" to cause robots to start 2m outside\n"
"                     the map bounds -- it later must be moved within the map\n"
"                     bounds to be used -- or, use the keyword \"random\" to\n"
"                     randomly choose a starting place within the map bounds.\n"
"  --resolution <r> : Use resolution <r> (milimeters) for collisions and\n"
"                     sensors. Default is 20mm (2cm)\n"
"  --ignore-command <num> :\n"
"                     Ignore the command whose number is given. Refer to robot\n"
"                     manual and MobileSim documentation for command numbers.\n"
"                     May be repeated. Warning, MobileSim and/or your program\n"
"                     may not function correctly if some critical commands are\n"
"                     ignored.\n"
"  --verbose   :      Be a more verbose about logging, especially things \n"
"                     that might be logged frequently (e.g. ignored unsupported\n"
"                     commands, some internal/debugging information, etc.)\n"
"  --log-timing-stats [sec]:\n"
"                     Log timing stats every [sec] seconds (default 30 sec).\n"
"  --bind-to-address <address> :\n"
"                     Only listen on network interface with IP address \n"
"                     <address> for new client connections (default is to\n"
"                     listen on all addresses).\n"
#ifndef WIN32
"  --no-crash-debug :\n"
"                     Disable GDB crash handler (just abort program on fatal\n"
"                     signals). (Not available on Windows)\n"
"  --no-crash-restart :\n"
"                     Disable automatic restart normally done if in noninteractive\n"
"                     mode. (Not available on Windows)\n"
#endif
"  --srisim-compat :\n"
"                     Enable compatability with SRISim by accepting \n"
"                     OLD_SIM_EXIT (62), OLD_SET_TRUE_X (66), OLD_SET_TRUE_Y (67),\n"
"                     OLD_SET_TRUE_TH (68), OLD_RESET_TO_ORIGIN (69). See the\n"
"                     MobileSim docs for details on new replacement commands.\n"
"  --no-srisim-laser-compat :\n"
"                     Disable compatability with SRISim laser commands\n"
"                     OLD_LRF_ENABLE (35), OLD_LRF_CFG_START (36),\n"
"                     OLD_LRF_CFG_END (37), and OLD_LRF_CFG_INC (38).\n"
"                     See MobileSim docs for details on new replacement commands.\n"
"  --log-packets-received :\n"
"                     Log all packets received from client.\n"
"  --log-movement-sent :\n"
"                     Log position and velocity values sent to client in SIP \n"
"                     (including with protocol conversion factors applied)."
"  --echo-stage-worldfile :\n"
"                     Print contents of stage world file while loading (for MobileSim debugging)\n"
"  --warn-unsupported-commands : \n"
"                     Warn when unrecognized or unsupported commands are received and ignored.\n"
"  --lines-chunksize: \n"
"                     When a map is loaded or reloaded, the map lines are split into chunks for\n"
"                     processing, to avoid long communication blackouts and subsequent disconnects.\n"
"                     If disconnects are happening during map loads, try setting this below " DEFAULT_MLPC_STR "\n"
"  --points-chunksize: \n"
"                     When a map is loaded or reloaded, the map points are split into chunks for\n"
"                     processing, to avoid long communication blackouts and subsequent disconnects.\n"
"                     If disconnects are happening during map loads, try setting this below " DEFAULT_MPPC_STR "\n"
"  --no-network-discovery : \n"
"                     Don't respond to network broadcast discovery requests.\n"
"  --odom-error-mode <random_init|random_each_update|constant|none> :\n"
"                     Specify odometry error behavior (see documentation).\n"
"  --help :           Print this help text and exit\n"
"  --version or -v :  Print MobileSim version number and exit\n"
"\n"
"MobileSim is based on the Stage 2.0 simulator library (see <http://playerstage.sf.net>)\n\n"
COPYRIGHT_TEXT
);
}



/* Get temp. dir. from environment or use default path */
const char* temp_dir()
{
  const char* tempdir = getenv("TEMP");
  if(!tempdir)
    tempdir = getenv("TMP");
  if(!tempdir)
#ifdef WIN32
    tempdir = "\\TEMP";
#else
  tempdir = "/tmp";
#endif
  return tempdir;
}

/* Return path to home directory if Linux, or My Documents directory if Windows. */

const char* user_docs_dir()
{
	const char *d;
#ifdef WIN32
	d = (const char*)g_get_user_special_dir(G_USER_DIRECTORY_DOCUMENTS);
#else
	d = (const char*)g_get_home_dir();
#endif
	return d;
}


/* Remember some info about the map we loaded so that other objects can use it. */
static mobilesim_start_place_t mobilesim_startplace = mobilesim_start_home;
static double map_min_x = 0;
static double map_min_y = 0;
static double map_max_x = 0;
static double map_max_y = 0;
static double map_home_x = 0;
static double map_home_y = 0;
static double map_home_th = 0;

void load_map_done(MapLoadedInfo info)
//StageMapLoader::Request *loadReq, double min_x, double min_y, double max_x, double max_y, double home_x, double home_y, double home_th)
{
    // this callback is called asynchronously, theoretically could get called
    // twice at once.
  map_min_x = info.min_x;
  map_min_y = info.min_y;
  map_max_x = info.max_x;
  map_max_y = info.max_y;
  map_home_x = info.have_home ? info.home_x : 0;
  map_home_y = info.have_home ? info.home_y : 0;
  map_home_th = info.have_home ? info.home_th : 0;
}

ArGlobalFunctor1<MapLoadedInfo> load_map_done_cb(&load_map_done);

int stage_load_file_cb(stg_world_t* /*world*/, char* filename, void* userdata)
{
 MapLoader *loader = (MapLoader*)userdata;
 //print_debug("MobileSim load file callback for filename %s!\n", filename);
 stg_print_msg("MobileSim load file callback for filename %s!\n", filename);
 loader->newMap(filename, NULL, &load_map_done_cb);
 return 0;
}



std::set<RobotInterface*> robotInterfaces;
std::set<RobotFactory*> robotFactories;

void log_file_full_cb(FILE* /*fp*/, size_t /*sz*/, size_t max)
{
    stg_print_warning("MobileSim: This log file is full (maximum size %d bytes). Rotating logs and starting a new one...", max);
    mobilesim_rotate_log_files(NULL);
}


// Command line arguments
MobileSim::Options options;


void do_gtk_iteration()
{
  if(!options.NonInteractive) 
    gtk_main_iteration_do(FALSE);
}

int main(int argc, char** argv) 
{

  MobileSim::Options& opt = options; // shortcut

  // save command-line arguments 
  opt.argc = argc;
  opt.argv = argv;

  // Map of robot name -> model string.
  RobotModels robotInstanceRequests;

  // List or set of model type strings.
  RobotFactoryRequests robotFactoryRequests;
//
//  // Stored config
//  MobileSimConfig config;

  for(int i = 1; i < argc; ++i) {
    if(command_argument_match(argv[i], "h", "help")) {
      usage();
      exit(0);
    }
    else if(command_argument_match(argv[i], "v", "version")) {
      puts("MobileSim " MOBILESIM_VERSION "\nUse --help for more information.\n");
      exit(0);
    }
    else if(command_argument_match(argv[i], "p", "port")) {
      if(++i < argc) {
        opt.port = atoi(argv[i]);
      } else {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "m", "map")) {
      if(++i < argc) {
        opt.map = argv[i];
      } else {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "nomap")) {
      // if -m was already given, don't change it. otherwise set to empty string
      opt.map = "";
      opt.nomap = true;
    }
    else if(command_argument_match(argv[i], "r", "robot")) {
      if(++i < argc) {
        std::string arg(argv[i]);
        size_t sep = arg.find(":");
        std::string model;
        std::string name;
        if(sep == arg.npos) {
          // No name given, invent one
          model = arg.substr(0, sep);
          name = model;
          char buf[4];
          for(int i = 2; robotInstanceRequests.find(name) != robotInstanceRequests.end(); i++) {
            snprintf(buf, 4, "%03d", i);
            name = model + "_" + buf;
          }
        } else {
          model = arg.substr(0, sep);
          name = arg.substr(sep+1);
        }
        //stg_print_msg("MobileSim: Robot named \"%s\" will be a \"%s\" model.", name.c_str(), model.c_str());
        robotInstanceRequests[name] = model;
      } else {
        usage();
        exit(ERR_USAGE);
      }
    } 
    else if(command_argument_match(argv[i], "R", "robot-factory")) {
      if(++i < argc) {
        stg_print_msg("MobileSim: Will create robot factory for model \"%s\".", argv[i]);
        robotFactoryRequests.push_back(argv[i]);
      } else {
        usage();
        exit(ERR_USAGE);
      }
    }
#ifndef MOBILESIM_NOGUI
    else if(command_argument_match(argv[i], "", "maximize") || command_argument_match(argv[i], "", "maximize-gui"))
    {
      opt.windowmode = opt.MAXIMIZE_WINDOW;
    }
    else if(command_argument_match(argv[i], "", "minimize") || command_argument_match(argv[i], "", "minimize-gui"))
    {
      opt.windowmode = opt.MINIMIZE_WINDOW;
    }
    else if(command_argument_match(argv[i], "", "fullscreen") || command_argument_match(argv[i], "", "fullscreen-gui"))
    {
      opt.windowmode = opt.FULLSCREEN_WINDOW;
    }
    else if(command_argument_match(argv[i], "", "noninteractive") || command_argument_match(argv[i], "", "non-interactive"))
    {
      opt.NonInteractive = true;
    }
#endif
#ifndef WIN32
    else if(command_argument_match(argv[i], "", "daemonize"))
    {
      opt.Daemon = true;
    }
#endif
#ifndef MOBILESIM_NOGUI
    else if(command_argument_match(argv[i], "", "no-graphics") || 
            command_argument_match(argv[i], "", "nographics") ||
            command_argument_match(argv[i], "", "disable-graphics"))
    {
      opt.graphicsmode = opt.NO_GRAPHICS;
    }
    else if(command_argument_match(argv[i], "", "no-gui") || 
            command_argument_match(argv[i], "", "nogui") ||
            command_argument_match(argv[i], "", "disable-gui"))
    {
      opt.graphicsmode = opt.NO_GUI;
      opt.NonInteractive = true;
    }
    else if(command_argument_match(argv[i], "", "lite-graphics") || command_argument_match(argv[i], "", "litegraphics"))
    {
      opt.graphicsmode = opt.LITE_GRAPHICS;
    }
#endif
    else if(command_argument_match(argv[i], "", "html"))
    {
      opt.log_html = true;
      stg_set_print_format(STG_PRINT_HTML);
    }
    else if(!strcasecmp(argv[i], "--cwd") || !strcasecmp(argv[i], "-cwd") || !strcasecmp(argv[i], "--cd") || !strcasecmp(argv[i], "-cd") ) 
    {
      if(++i < argc)
      {
        opt.change_to_directory = argv[i];
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "l", "log-file"))
    {
      if(++i < argc)
      {

        opt.log_file = argv[i];
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "log-file-max-size"))
    {
        if(++i < argc)
            opt.log_file_max_size = atoi(argv[i]);
        else
        {
            usage();
            exit(ERR_USAGE);
        }
    }
    else if(command_argument_match(argv[i], "", "update-interval"))
    {
      if(++i < argc)
      {
        opt.interval_real = (stg_msec_t) atoi(argv[i]);
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "update-sim-time"))
    {
      if(++i < argc)
      {
        opt.interval_sim = (stg_msec_t) atoi(argv[i]);
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "resolution"))
    {
      if(++i < argc)
      {
        opt.world_res = atof(argv[i]);
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "start"))
    {
      if(++i < argc)
      {
        if(strcmp(argv[i], "outside") == 0)
        {
          mobilesim_startplace = mobilesim_start_outside;
        }
        else if(strcmp(argv[i], "random") == 0)
        {
          mobilesim_startplace = mobilesim_start_random;
        }
        else
        {
          if(sscanf(argv[i], "%ld,%ld,%d", &opt.start_pos_override_pose_x, &opt.start_pos_override_pose_y, &opt.start_pos_override_pose_th) != 3)
          {
            fputs("Error: --start must be used either with the form <x>,<y>,<th> (e.g. 200,4650,90) (mm and deg), or the keyword \"random\" or \"outside\".", stderr);
            exit(ERR_USAGE);
          }
          mobilesim_startplace = mobilesim_start_fixedpos;
        }
      }
      else
      {
       fputs("Error: --start must be used either with the form <x>,<y>,<th> (e.g. 200,4650,90) (mm and deg), or the keyword \"random\" or \"outside\".", stderr);
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "ignore-command"))
    {
      if(i >= argc || argv[i+1][0] == '-')
      {
        fputs("Error: --ignore-command must be followed by command number.", stderr);
        exit(ERR_USAGE);
      }
      int c = atoi(argv[++i]);
      stg_print_msg("MobileSim: Will ignore command #%d.", c);
      opt.ignore_commands.insert(c);
    }
    else if(command_argument_match(argv[i], "", "verbose"))
    {
      opt.verbose = true;
    }
    else if(command_argument_match(argv[i], "", "log-timing-stats"))
    {
      int sec = 30;
      if(i+1 < argc && argv[i+1][0] != '-')
        sec = atoi(argv[++i]);
      MobileSim::log_stats_freq = stg_log_stats_freq = sec*1000; // msec
      stg_print_msg("MobileSim: Will log timing information every %d seconds.", sec);
    }
    else if(command_argument_match(argv[i], "", "bind-to-address"))
    {
      opt.listen_address = argv[++i];
    }
    else if(command_argument_match(argv[i], "", "no-crash-handler") ||
            command_argument_match(argv[i], "", "disable-crash-handler"))
    {
      // old, deprecated command line argument
      opt.EnableCrashDebug = false;
      opt.EnableCrashRestart = false;
    }
    else if(command_argument_match(argv[i], "", "no-crash-debug") ||
            command_argument_match(argv[i], "", "disable-crash-debug"))
    {
      opt.EnableCrashDebug = false;
    }
    else if(command_argument_match(argv[i], "", "no-crash-restart") ||
            command_argument_match(argv[i], "", "disable-crash-restart"))
    {
      opt.EnableCrashRestart = false;
    }
    else if(command_argument_match(argv[i], "", "restarting-after-crash"))
    {
      opt.RestartedAfterCrash = true;
    }
    else if(command_argument_match(argv[i], "", "srisim-compat"))
    {
      opt.srisim_compat = true;
    }
    else if(command_argument_match(argv[i], "", "no-srisim-laser-compat") ||
            command_argument_match(argv[i], "", "disable-srisim-laser-compat"))
    {
      opt.srisim_laser_compat = false;
    }
    else if(command_argument_match(argv[i], "", "no-menu"))
    {
      stg_show_menubar = 0;
    }
    else if(command_argument_match(argv[i], "", "no-messages"))
    {
      stg_show_messages_view = 0;
    }
    else if(command_argument_match(argv[i], "", "log-packets-received"))
    {
      opt.log_packets_received = true;
    }
    else if(command_argument_match(argv[i], "", "echo-stage-worldfile"))
    {
      opt.echo_stage_worldfile = true;
    }
    else if(command_argument_match(argv[i], "", "warn-unsupported-commands"))
    {
      opt.warn_unsupported_commands = true;
    }
    else if(command_argument_match(argv[i], "", "log-movement-sent"))
    {
      opt.log_sips_sent = true;
    }
    else if(command_argument_match(argv[i], "", "no-network-discovery"))
    {
      opt.run_network_discovery = false;
    }
    else if(command_argument_match(argv[i], "", "lines-chunksize"))
    {
      if(++i < argc)
      {
        opt.mapLoadLinesPerChunk = atol(argv[i]);
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "points-chunksize"))
    {
      if(++i < argc)
      {
        opt.mapLoadPointsPerChunk = atol(argv[i]);
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(command_argument_match(argv[i], "", "odom-error-mode"))
    {
      if(++i < argc)
      {
        if(strcasecmp(argv[i], "random_init") == 0)
        {
          opt.odom_error_mode = MobileSim::Options::RANDOM_INIT;
        }
        else if(strcasecmp(argv[i], "random_each_update") == 0)
        {
          opt.odom_error_mode = MobileSim::Options::RANDOM_EACH_UPDATE; 
        }
        else if(strcasecmp(argv[i], "constant") == 0)
        {
          opt.odom_error_mode = MobileSim::Options::CONSTANT;
        }
        else if(strcasecmp(argv[i], "none") == 0)
        { 
          opt.odom_error_mode = MobileSim::Options::NONE;
        }
        else
        {
          printf("Error: Unrecognized odom-error-mode \"%s\". Expected one of: random_init, random_each_update, constant, none.\n\n", argv[i]);
          usage();
          exit(ERR_USAGE);
        }
      }
      else
      {
        usage();
        exit(ERR_USAGE);
      }
    }
    else if(i == (argc-1))
    {
      // last argument on the end of the command line. if it's not a flag,
      // assume it's a map file.
      if(argv[i][0] == '-')
      {
        usage();
        exit(ERR_USAGE);
      }
      opt.map = argv[i];
    }
    else {
      stg_print_warning("MobileSim: Ignoring unrecognized command-line argument \"%s\".", argv[i]);
    }
  }


#ifdef MOBILESIM_NOGUI
  opt.NonInteractive = true;
  opt.graphicsmode = NO_GUI;
#endif

  if(opt.log_file)
  {
    // force noncolor text if not html
    if(!opt.log_html)
      stg_set_print_format(STG_PRINT_PLAIN_TEXT);

    stg_print_msg("MobileSim: Rotating log files (%s) if present...", opt.log_file);
    mobilesim_rotate_log_files(NULL);
    FILE* fp = ArUtil::fopen(opt.log_file, "w");
    if(!fp)
    {
      stg_print_error("MobileSim: Could not open log file \"%s\" for writing.", opt.log_file);
      exit(ERR_USAGE);
    }
    stg_set_log_file(fp);
    if(opt.log_file_max_size == 0 && opt.NonInteractive)
      opt.log_file_max_size = 5 * 1024 * 1024;  // default of 5 MB
    if(opt.log_file_max_size > 0)
        stg_set_log_file_max_size(opt.log_file_max_size, &log_file_full_cb);
  }


  print_msg("MobileSim version " MOBILESIM_VERSION " built " MOBILESIM_BUILDDATE);
  opt.log_argv();

  for(RobotModels::const_iterator i = robotInstanceRequests.begin(); i != robotInstanceRequests.end(); ++i)
  {
    stg_print_msg("MobileSim: Robot named \"%s\" will be a \"%s\"", i->first.c_str(), i->second.c_str());
  }

  /* Directory where supporting files (robot model defs, icons)
     can be found.
     */
  const char* libdir = find_libdir();


  /* Initialize Stage and GTK */
  stg_about_info_appname = (char*)"MobileSim";
  stg_about_info_description = (char*)"Simulator for MobileRobots/ActivMedia robots, based on the Stage robot simulator library (with modifications by MobileRobots Inc).";
  stg_about_info_url = (char*)"http://github.com/reedhedges/MobileSim";
  stg_about_info_copyright = (char*) COPYRIGHT_TEXT  ;
  stg_about_info_appversion = (char*)MOBILESIM_VERSION;
  stg_help_link = MOBILESIM_DEFAULT_HELP_URL;


#ifndef WIN32
  if(opt.Daemon)
  {
    opt.NonInteractive = true;
    print_msg("Forking into background (--daemonize option was given)...");
    pid_t pid = fork();
    if(pid == -1)
    {
      perror("Error forking new background process! ");
      exit(ERR_FORK);
    }
    if(pid != 0)
    {
      print_debug("Background process has PID %d.\n", pid);
      exit(0);
    }
  }
#endif

  if(opt.graphicsmode == opt.NO_GUI)
  {
    opt.NonInteractive = true;
  }

  if(opt.NonInteractive)
  {
    stg_use_error_dialog = FALSE;
  }

  switch(opt.odom_error_mode) 
  {
    case MobileSim::Options::RANDOM_INIT:
      stg_position_force_odom_error_mode_all_models(STG_POSITION_ODOM_ERROR_RANDOM_INIT);
      break;
    case MobileSim::Options::RANDOM_EACH_UPDATE:
      stg_position_force_odom_error_mode_all_models(STG_POSITION_ODOM_ERROR_RANDOM_EACH_UPDATE);
      break;
    case MobileSim::Options::CONSTANT:
      stg_position_force_odom_error_mode_all_models(STG_POSITION_ODOM_ERROR_CONSTANT);
      break;
    case MobileSim::Options::NONE:
      stg_position_force_odom_error_mode_all_models(STG_POSITION_ODOM_ERROR_NONE);
      break;
  }

  stg_init(argc, argv);

  if(opt.RestartedAfterCrash)
    stg_print_warning("MobileSim crashed and was automatically restarted.  See log files (-crash) for diagnostic information.");

#ifndef WIN32
  // ignore this signal, instead let write() return an error 
  signal(SIGPIPE, SIG_IGN);

  if(opt.EnableCrashDebug || opt.EnableCrashRestart)
  {
      // catch crashes, save debugging information, and restart if
      // noninteractive
      signal(SIGSEGV, mobilesim_crash_handler);
      signal(SIGILL, mobilesim_crash_handler);
      signal(SIGFPE, mobilesim_crash_handler);
      signal(SIGABRT, mobilesim_crash_handler);
      signal(SIGBUS, mobilesim_crash_handler);
  }
#endif

#ifndef MOBILESIM_NOGUI
#if GTK_CHECK_VERSION(2,2,0)
  /* Set GTK's default icon so Stage uses it */
  char iconfile[MAX_PATH_LEN];
  snprintf(iconfile, MAX_PATH_LEN-1, "%s%cicon.png", libdir, PATHSEPCH);
  GError* err = NULL;
  if(!gtk_window_set_default_icon_from_file(iconfile, &err))
  {
    stg_print_warning("MobileSim: Could not load program icon \"%s\": %s", iconfile, err->message);
    g_error_free(err);
  }
#endif
#endif // NOGUI

  /* Open config file */
  //XXX TODO


  /* Check for user's init.map */
  // XXX TODO use appropriate config directory on Windows
  char initmapfile[MAX_PATH_LEN];
  if(opt.map == "" && !opt.nomap) // neither -m or -nomap opt given
  {
    strncpy(initmapfile, g_get_home_dir(), MAX_PATH_LEN-1);
    strncat(initmapfile, "/.MobileSim/init.map", MAX_PATH_LEN-1);
    struct stat s;
    if( stat(initmapfile, &s) == 0 )
    {
      stg_print_msg("MobileSim: Found init. map file \"%s\".", initmapfile);
      opt.map = initmapfile;
    }
  }


  /* Change to startup directory before dialog box, so that it shows the startup
   * dir. 
   */
  if(opt.change_to_directory)
  {
    stg_print_msg("MobileSim: Changing to directory \"%s\"...", opt.change_to_directory);
    if(getcwd(opt.before_change_to_directory, MAX_PATH_LEN) == NULL)
        opt.before_change_to_directory[0] = '\0';
    if( chdir(opt.change_to_directory) != 0 )
    {
      print_error("MobileSim: Error changing to directory \"%s\"", opt.change_to_directory);
      exit(ERR_CHDIR);
    }
  }



  if(opt.nomap)
    stg_print_msg("MobileSim: Will start with no map.");
  else if (opt.map != "")
    stg_print_msg("MobileSim: Will use map \"%s\" for simulation environment.", opt.map.c_str());
  else
  {
    if(opt.NonInteractive)
    {
      stg_print_error("MobileSim: No map given in noninteractive mode. Use --map to specify a map file, or use --nomap for an empty map.");
      exit(ERR_USAGE);
    }

    /* Show the dialog box. Only show the robot models selection if instances or
     * factories are already requested on the command line.
     */
    int act = 0;
    if(robotInstanceRequests.empty() && robotFactoryRequests.empty())
    {
      act = map_options_dialog(opt.map, &robotInstanceRequests, &robotFactoryRequests, &opt );
    }
    else
    {
      act = map_options_dialog(opt.map, &opt);
    }
    if(act == 2)
      opt.nomap = true;

  }
  
  
 
  // Show an initializing/loading dialog
  GtkWidget *busydialog = NULL;
  if(!opt.NonInteractive)
  {
    busydialog = busy_loading_dialog(); 
    assert(busydialog);
  
    // main loop is not running yet, so do a few iterations to make sure the dialog contents are shown:
    for(int i = 0; i < 10; ++i)
      gtk_main_iteration_do(FALSE);
  }

  /* Setup our special pioneer-specific properties for the world file. These
   * will be read from the model definitions by
   * emulatePioneer/stageInterface.cc. 
   * TODO move to StageInterface.cc.
   */
  stg_model_user_property("position", "pioneer_robot_subtype", STG_STRING);
  stg_model_user_property("position", "pioneer_robot_type", STG_STRING);
  stg_model_user_property("position", "pioneer_diffconv", STG_FLOAT);
  stg_model_user_property("position", "pioneer_angleconv", STG_FLOAT);
  stg_model_user_property("position", "pioneer_distconv", STG_FLOAT);
  stg_model_user_property("position", "pioneer_vel2div", STG_FLOAT);
  stg_model_user_property("position", "pioneer_velconv", STG_FLOAT);
  stg_model_user_property("position", "pioneer_rangeconv", STG_FLOAT);
  stg_model_user_property("ranger",   "pioneer_hasfrontarray", STG_INT);
  stg_model_user_property("ranger",   "pioneer_hasreararray", STG_INT);
  stg_model_user_property("ranger",   "pioneer_max_readings_per_packet", STG_INT);
  //stg_model_user_property("position", "pioneer_sip_cycle", STG_INT);
  stg_model_user_property("position", "pioneer_watchdog", STG_INT);
  stg_model_user_property("position", "pioneer_batterytype", STG_INT);
  stg_model_user_property("position", "pioneer_gps_dop", STG_FLOAT);
  stg_model_user_property("position", "pioneer_gps_pos_x", STG_FLOAT);
  stg_model_user_property("position", "pioneer_gps_pos_y", STG_FLOAT);

  /* create Stage world with our map and robot(s) */
  stg_world_t* world;

  /* Create one p3dx model by default, if no instances or factories given: */
  if(robotInstanceRequests.size() == 0 && robotFactoryRequests.size() == 0)
    robotInstanceRequests["p3dx-sh"] = "p3dx-sh";

  /* You need to specify a map or nomap if you specify any arguments: */
  if(opt.map == "" && !opt.nomap)
  {
    stg_print_error("MobileSim: No mapfile given. Use --map to specify a map file, or --nomap.");
    exit(ERR_USAGE);
  }

  if(!opt.NonInteractive)
    gtk_main_iteration_do(FALSE);


  /* Convert Aria map to Stage world, and load it. TODO don't create robot
     * modelste there.  */
  world = create_stage_world(opt.map.c_str(), 
    //robotInstanceRequests, robotFactoryRequests,
    libdir, 
    //mobilesim_startplace, opt.start_pos_override_pose_x, opt.start_pos_override_pose_y, opt.start_pos_override_pose_th, 
    opt.world_res/1000.0, &do_gtk_iteration);


  if(!world) {
    exit(ERR_STAGELOAD);
  }

  mapLoader.setWorld(world);
  mapLoader.setMapLoadChunkSizes(opt.mapLoadLinesPerChunk, opt.mapLoadPointsPerChunk);


  // load map 
  if(opt.map != "")
  {
    mapLoader.newMap(opt.map, NULL, NULL);
    mapLoader.processUntilDone();
  }
  // save list of map files for use in startup GUI.
  //config.updateRecentMaps(map);

  

  // add callback for loading a new map file from GUI menus
  stg_world_add_file_loader(world, &stage_load_file_cb, "*.map", "MobileRobots/ActivMedia Map Files", &mapLoader);

  stg_world_set_quiet(world, !opt.verbose);

  /* Start the world now (needs to be running on order to both create robot
   * models and show the gui without deadlocks or race conditions) */
  stg_world_start(world);

  if(!opt.NonInteractive)
    stg_world_set_cursor_busy(world);

  /* Find place to start robot at, now that world has been created */
  stg_pose_t startPose;
  startPose.x = 0.0; //m
  startPose.y = 0.0; //m
  startPose.a = 0.0; //deg
  switch(mobilesim_startplace)
  {
    case mobilesim_start_fixedpos:
    {
        stg_print_msg("MobileSim: Starting robots at position given by --start option: %f, %f, %f.", opt.start_pos_override_pose_x/1000.0, opt.start_pos_override_pose_y/1000.0, opt.start_pos_override_pose_th);
      map_home_x = opt.start_pos_override_pose_x;
      map_home_y = opt.start_pos_override_pose_y;
      map_home_th = opt.start_pos_override_pose_th;
    }

    break;
    case mobilesim_start_outside:
    {
      map_home_x = map_min_x - 2000.0;
      map_home_y = map_max_y;
      map_home_th = 0;
      stg_print_msg("MobileSim: Starting robots 2m outside edge of map, at: %f, %f, %f.", map_home_x/1000.0, map_home_y/1000.0, map_home_th);
    }
    break;

    case mobilesim_start_random:
    {
      /* Nothing, will be randomly generated below */
      stg_print_msg("MobileSim: Starting robots at random positions in map.");
      map_home_x = map_home_y = map_home_th = 0.0;
    }
    break;


    default: /* home */
    {
      ArMapObject* home = map->findFirstMapObject(NULL, "RobotHome");
      if(!home)
        home = map->findFirstMapObject(NULL, "Dock");
      if(home)
      {
        map_home_x = home->getPose().getX();
        map_home_y = home->getPose().getY();
        map_home_th = home->getPose().getTh();
      }
      stg_print_msg("MobileSim: Starting robots at map's default starting place of: %f, %f, %f", map_home_x/1000.0, map_home_y/1000.0, map_home_th);
    }
  }

  startPose.x = map_home_x / 1000.0;
  startPose.y = map_home_y / 1000.0;
  startPose.a = map_home_th;

  if(!opt.NonInteractive)
    gtk_main_iteration_do(FALSE);


  /* Initialize Winsock */
  ArSocket::init();

  /* Create any robot factories specified */
  int facport = opt.port;
  for(std::list<std::string>::iterator i = robotFactoryRequests.begin(); i != robotFactoryRequests.end(); i++)
  {
    const char *modelname = (*i).c_str();
    stg_print_msg("MobileSim: Creating new robot factory for \"%s\"...", modelname);
    RobotFactory *stagefac;
    if(mobilesim_startplace == mobilesim_start_fixedpos) 
      stagefac = new StageRobotFactory(world, modelname, opt.start_pos_override_pose_x, opt.start_pos_override_pose_y, opt.start_pos_override_pose_th, &opt);
    else
      stagefac = new StageRobotFactory(world, modelname, &mobilesim_get_map_home, &mobilesim_get_map_bounds, (mobilesim_startplace==mobilesim_start_outside), &opt );
    //stagefac->setCommandsToIgnore(opt.ignore_commands);
    //stagefac->setVerbose(opt.verbose)
    //stagefac->setSRISimCompat(opt.srisim_compat, opt.srisim_laser_compat);
    //stagefac->setWarnUnsupportedCommands(opt.warn_unsupported_commands);
    stg_print_msg("MobileSim: opening factory port %d...", facport);
    ArSocket *facsock = stagefac->open(facport, (opt.listen_address == "" ? NULL : opt.listen_address.c_str()));
    if(!facsock) {
      stg_print_error("MobileSim: Could not open port %d for robot factory %s.  Try another with -p.\n", facport, modelname);
      exit(ERR_OPEN_PORT);
    }
    robotFactories.insert(stagefac);
    if(opt.listen_address != "")
      stg_print_msg("MobileSim: Robot factory for \"%s\" ready on %s port %d", modelname, opt.listen_address.c_str(), facport);
    else
      stg_print_msg("MobileSim: Robot factory for \"%s\" ready on port %d", modelname, facport);
    ++facport;

    if(!opt.NonInteractive) gtk_main_iteration_do(FALSE);
  }

  if(!opt.NonInteractive)
    gtk_main_iteration_do(FALSE);

  /* Create robot models  */
  // XXX TODO create robot factories for any robotFactoryRequests that are for .p files
  for(RobotModels::const_iterator i = robotInstanceRequests.begin(); i != robotInstanceRequests.end(); ++i)
  {
    std::string modelname = (*i).second.c_str();
    std::string robotname = (*i).first.c_str();
    
    if(mobilesim_startplace == mobilesim_start_random)
    {
      startPose.x = STG_RANDOM_RANGE(map_min_x, map_max_x) / 1000.0;
      startPose.y = STG_RANDOM_RANGE(map_min_y, map_max_y) / 1000.0;
      startPose.a = rand() % 360;
    }

    stg_model_t *model = stg_world_new_model(world, modelname.c_str(), NULL, (robotname=="")?NULL:robotname.c_str());
    if(!model) {
      stg_print_error("MobileSim: could not create a model of type \"%s\".", modelname.c_str());
      continue;
    }

    stg_model_set_pose(model, startPose);

    // XXX todo fprintf(world_fp, "\tmessages ( )\n");

    startPose.y -= 1.0;

    if(!opt.NonInteractive) gtk_main_iteration_do(FALSE);
  }
    

  /* Create Pioneer emulators for each position model.
   * TODO create robot models in stage here too instead of in the world file in create_stage_world. */
  for(std::map<std::string, std::string>::const_iterator i = robotInstanceRequests.begin();
      i != robotInstanceRequests.end(); i++)
  {
    const std::string& name = (*i).first;
    const std::string& model = (*i).second;
    //    stg_print_msg("MobileSim: Creating emulated Pioneer connection for robot named \"%s\" (\"%s\") on TCP port %d.", (*i).first.c_str(), (*i).second.c_str(), port);
    //    stg_world_display_message(world, 0, "MobileSim", STG_MSG_INFORMATION, "Creating emulated Pioneer connection for \"%s\" (\"%s\") on TCP port %d.", (*i).second.c_str(), (*i).first.c_str(), port);
    StageInterface* stageint = new StageInterface(world, model, name); 
    EmulatePioneer* emulator = new EmulatePioneer(stageint, model, opt.port++, false, true, &opt);
    if(map)
	    emulator->loadMapObjects(map);
    robotInterfaces.insert(stageint);
    //emulators.insert(emulator);  
    emulator->setSimulatorIdentification("MobileSim", MOBILESIM_VERSION);
    //emulator->setCommandsToIgnore(opt.ignore_commands);
    //emulator->setVerbose(opt.verbose);
    //emulator->setSRISimCompat(opt.srisim_compat, opt.srisim_laser_compat);
    //emulator->setLogPacketsReceived(opt.log_packets_received);
    //emulator->setWarnUnsupportedCommands(opt.warn_unsupported_commands);
    //emulator->setLogSIPsSent(opt.log_sips_sent);
    if(opt.listen_address != "")
      emulator->setListenAddress(opt.listen_address);
    // TODO just pass opt in to EmulatePioneer constructor, let it take options
    // from that rather than having to set them individually here.
    if(!emulator->openSocket())
      delete emulator;
  }

  /* Change Stage's window title. TODO: include map file name? */
  stg_world_set_window_title(world, "MobileSim");
  
  if(!opt.NonInteractive)
    gtk_main_iteration_do(FALSE);

#ifndef MOBILESIM_NOGUI
  /* Set window mode as requested */
  switch(opt.windowmode)
  {
    case Options::MAXIMIZE_WINDOW:
      stg_world_window_request_maximize(world);
      break;
    case Options::MINIMIZE_WINDOW:
      stg_world_window_request_minimize(world);
      break;
   case Options::FULLSCREEN_WINDOW:
      stg_world_window_request_fullscreen(world);
      break;
    default:
      /* do nothing. */
      ;
  }

  /* Set graphics mode as requested */
  switch(opt.graphicsmode)
  {
    case Options::NO_GUI:
      stg_world_disable_gui(world);
      break;
    case Options::NO_GRAPHICS:
      stg_world_hide_all_graphics(world);
      break;
    case Options::LITE_GRAPHICS:
      stg_world_set_fill_polygons(world, FALSE);
      break;
    default:
      /* do nothing */
      ;
  }
#endif

  /* run network autodiscovery responder thread */ 
  NetworkDiscoveryResponder discovery;
  if(opt.run_network_discovery)
  {
    stg_print_msg("MobileSim: Will respond to discovery requests on UDP port %d", discovery.getPort());
    discovery.runAsync();
  }

  /* Set time intervals as requested */
  if(opt.interval_sim == 0)
    opt.interval_sim = opt.interval_real;
  stg_world_set_interval_real(world, opt.interval_real);
  stg_world_set_interval_sim(world, opt.interval_sim);


  /* run until exit. */

  if(!opt.NonInteractive)
    stg_world_set_cursor_normal(world);
  
  if(busydialog)
  {
    gtk_widget_hide(busydialog);
    gtk_widget_destroy(busydialog);
  }
  
  int stageStat = 0;


  /* Main loop

     Stage updates and client output are time sensitive and must be done as
     close to the desired intervals as possible.  We decide which has to 
     be done first and sleep for that time, do it, then reset its 
     next scheduled time.  If during this loop we have some time available
     and it hasn't been too long, we also do non-time-critical tasks
     such as check for client and factory input, and work on loading
     a new map.
  
     see test_mainloop.cc for a test that looks at this loop strategy only,
     with dummy sleeps for the tasks.

     @todo Right now we assume that all clients want output at 100ms, but
     the real robot can be configured to use any SIP frequency; generalize 
     this runloop to a priority queue containing the stage update
     task plus each group of clients that has a distinct SIPFreq set.
  */
  const int clientOutputFreq = DEFAULT_SIP_FREQ; //100;
  const int clientOutputWarningTime = clientOutputFreq * 2;
  const int stageUpdateFreq = opt.interval_real;
  const int stageUpdateWarningTime = stageUpdateFreq * 2;
  ArTime stageUpdateDue;
  ArTime clientOutputDue;
  ArTime lastStageUpdate;
  ArTime lastClientOutput;
  ArTime lastClientInput;
  ArTime lastMapProcess;
  ArTime lastRobotFactory;
  const int clientInputWindow = 20; // Do client and factory input if we have this much time available or it's been too long (see below)
  const int clientInputMaxFreq = 200; // Do client input if it's been this long since the last time, or we have some free time available (see above)
  const int mapProcessWindow = 50; // See clientInputWindow
  const int mapProcessMaxFreq = 400; // See clientInputMaxFreq
  const int RobotFactoryWindow = 50; // See clientInputWindow
  const int RobotFactoryMaxFreq = 400; // See clientInputMaxFreq

  // If either the StageUpdate or ClientOutput has waited for longer than this time,
  //   restart the loop so that it can get serviced
  //const int emergencyRestartTime = 500;
  // If ClientInput, MapLoader, or RobotFactory has waited for longer than this time,
  //   complete the loop so that it can get serviced. This can potentially happen if
  //   many robots make the StageUpdate or ClientOutput consistently take a long time.
  //const int emergencyRestartByPass = 2000;


  while(stageStat == 0)
  {
    long untilStageUpdate = stageUpdateDue.mSecTo();
    if(untilStageUpdate < 0) untilStageUpdate = 0;
    long untilClientOutput = clientOutputDue.mSecTo();
    if(untilClientOutput < 0) untilClientOutput = 0;
    //print_debug("%ld ms until stage update, %ld ms to client output.", untilStageUpdate, untilClientOutput);
    if(untilStageUpdate < untilClientOutput)
    {
      /* Stage update */
      //ArTime t;
      MobileSim::sleep(untilStageUpdate);
      //print_debug("Waiting for stage update: sleep(%ld) took %ld ms", untilStageUpdate, t.mSecSince());
      if(lastStageUpdate.mSecSince() > stageUpdateWarningTime)
        print_warning("Took >%ld ms since last stage simulation update (%ld, interval is %ld)", stageUpdateWarningTime, lastStageUpdate.mSecSince(), stageUpdateFreq);
      lastStageUpdate.setToNow();
      stageUpdateDue.setToNow(); 
      stageUpdateDue.addMSec(stageUpdateFreq);
      stageStat = stg_world_update(world, FALSE /*sleepflag*/, FALSE /*skiptooson*/ );
    }
    else
    {
      /* Client output and other periodic EmulatePioneer work */
      //ArTime t;
      MobileSim::sleep(untilClientOutput);
      //print_debug("Waiting for client output: sleep(%ld) took %ld ms", untilClientOutput, t.mSecSince());
      if(lastClientOutput.mSecSince() > clientOutputWarningTime)
        print_warning("Warning: Took >%ld ms since last clients output update (%ld, interval is %ld)", clientOutputWarningTime, lastClientOutput.mSecSince(), clientOutputFreq);
      lastClientOutput.setToNow();
      clientOutputDue.setToNow();
      clientOutputDue.addMSec(clientOutputFreq);
      EmulatePioneer::processAll();
    }
/*
    // If ClientInput, MapLoader, or RobotFactory has waited for longer than this time,
    //   complete the loop so that it can get serviced. This can potentially happen if
    //   many robots make the StageUpdate or ClientOutput consistently take a long time.
    const int emergencyRestartByPass = 2000;
    if (lastClientInput.mSecSince() < emergencyRestartByPass &&
        lastMapProcess.mSecSince() < emergencyRestartByPass &&
        lastRobotFactory.mSecSince() < emergencyRestartByPass)
    {
      //ArLog::log(ArLog::Normal, "main: not bypassing the emergency loop restart check because lastClientInput.mSecSince=(%ld), lastMapProcess.mSecSince=(%ld), or lastRobotFactory.mSecSince=(%ld)",
      //    lastClientInput.mSecSince(), lastMapProcess.mSecSince(), lastRobotFactory.mSecSince());

      // If either the StageUpdate or ClientOutput has waited for longer than this time,
      //   restart the loop so that it can get serviced
      if (lastStageUpdate.mSecSince() > emergencyRestartTime ||
          lastClientOutput.mSecSince() > emergencyRestartTime)
      {
        //ArLog::log(ArLog::Normal, "main: executing emergency loop restart because of lastStageUpdate.mSecSince=(%ld) or lastClientOutput.mSecSince=(%ld)",
        //    lastStageUpdate.mSecSince(), lastClientOutput.mSecSince());
        continue;
      }
      //else
        //ArLog::log(ArLog::Normal, "main: not executing emergency loop restart because lastStageUpdate.mSecSince=(%ld) or lastClientOutput.mSecSince=(%ld)",
        //    lastStageUpdate.mSecSince(), lastClientOutput.mSecSince());

    }
    //else
      //ArLog::log(ArLog::Normal, "main: bypassing the emergency loop restart check because of lastClientInput.mSecSince=(%ld), lastMapProcess.mSecSince=(%ld), or lastRobotFactory.mSecSince=(%ld)",
      //    lastClientInput.mSecSince(), lastMapProcess.mSecSince(), lastRobotFactory.mSecSince());
*/

    /* Client and factory input */
    if( (stageUpdateDue.mSecTo() > clientInputWindow && clientOutputDue.mSecTo() > clientInputWindow) || lastClientInput.mSecSince() > clientInputMaxFreq )
    {
      lastClientInput.setToNow();
      Sockets::processInput();
    }


    /* Map loader */
    if( (stageUpdateDue.mSecTo() > mapProcessWindow && clientOutputDue.mSecTo() > mapProcessWindow) ||
        (lastMapProcess.mSecSince() > mapProcessMaxFreq && !mapLoader.shouldWaitForOthers()))
    {
      int processTimeWindow;
      if (stageUpdateDue.mSecTo() > mapProcessWindow && clientOutputDue.mSecTo() > mapProcessWindow)
        processTimeWindow = min(stageUpdateDue.mSecTo(), clientOutputDue.mSecTo());
      else
        processTimeWindow = mapProcessWindow;

      lastMapProcess.setToNow();
      mapLoader.process(processTimeWindow);
    }
   
    /* Robot Factory */
    if( (stageUpdateDue.mSecTo() > RobotFactoryWindow && clientOutputDue.mSecTo() > RobotFactoryWindow) ||
        lastRobotFactory.mSecSince() > RobotFactoryMaxFreq)
    {
     lastRobotFactory.setToNow();
      for (std::set<RobotFactory *>::iterator robotFac_it = robotFactories.begin(); robotFac_it != robotFactories.end(); robotFac_it++)
      {
        (*robotFac_it)->createNewRobotsFromClientsList();
      }
    }
  }












#if 0
  ArTime startTime;
  ArTime processTime;
  ArTime factoryProcessTime;
  int stageUpdateTime;
  int emuStat;
  int lastOvershoot = 0;
  int timeLeft;
  int t;
  const int warningtime = opt.interval_real+5;
  while(stageStat == 0)
  {
    //if(startTime.mSecSince() > interval_real)
    //  stg_print_warning("last loop took %d ms", startTime.mSecSince());

    // Update stage simulation and GUI:
    startTime.setToNow();
    const stg_bool_t sleepflag = FALSE;
    stageStat = stg_world_update(world, sleepflag);
    stageUpdateTime = startTime.mSecSince();

    // How much time is left?
    // maximum allowed loop time is until we need to do either a simulation update or send a SIP, minus a ms for overhead
    const int looptime = -1 + ((unsigned int) opt.interval_real > EmulatePioneer::smallestSIPFreq() ? opt.interval_real : EmulatePioneer::smallestSIPFreq());
    //
    // TODO ask stage for time to next update rather than use fixed interval_real; similar for EP's
    if(stageUpdateTime - lastOvershoot < looptime)
      timeLeft = looptime - stageUpdateTime - lastOvershoot;
    else
      timeLeft = 0;
    //printf("stage update took %d, lastOvershoot=%d, timeLeft=%d\n", stageUpdateTime, lastOvershoot, timeLeft);

    // Send data:
    processTime.setToNow();
    emuStat = EmulatePioneer::processAll(timeLeft);
    //printf("emuStat=%d (GOTDATA=%d)\n", emuStat, EmulatePioneer::GOTDATA);
    if(processTime.mSecSince() < timeLeft)
      timeLeft -= processTime.mSecSince();
    else
      timeLeft = 0;

    assert(timeLeft >= 0);
    // Continue loading any ongoing map loads:
    processTime.setToNow();
    mapLoader.process(timeLeft/3);
    if(processTime.mSecSince() < timeLeft)
      timeLeft -= processTime.mSecSince();
    else
      timeLeft = 0;

    // Wait for incoming data (listening sockets for EmulatePioneer and RobotFactory, as well as current clients):
    processTime.setToNow();
    assert(timeLeft >=0);
    Sockets::process(timeLeft);
    t = processTime.mSecSince();
    if(t <= timeLeft)
      timeLeft -= t;
    else
    {
      if(t - timeLeft > 5)
        stg_print_warning("MobileSim: Took too long to process client input. (took %d ms, window was %d ms)", t, timeLeft);
      timeLeft = 0;
    }

    // how long did the whole thing take?
    t = startTime.mSecSince();
    if(t > warningtime)
      stg_print_warning("MobileSim: Took >%d ms (%d ms) to update simulation and handle all client commands and packet sending.", warningtime, t);

    /*
    // set lastOvershoot to how far over looptime we went since the last stage update. next loop, we will run shorter accordingly.
    if(t > looptime)
      lastOvershoot = min(looptime/2, (t - looptime));
    else
      lastOvershoot = 0;

    printf("lastOvershoot = %d", lastOvershoot);
    assert(lastOvershoot < looptime);
    */
  }
#endif



  /* Cleanup and return */


#ifdef DELETE_EMULATORS
  puts("Stage exited. deleting emulators...");
  EmulatePioneer::deleteAllInstances();
#endif


#ifdef DELETE_ROBOT_INTERFACES
  puts("Stage exited. Deleting robot interfaces...");
  for(std::set<RobotInterface*>::const_iterator i = robotInterfaces.begin(); i != robotInterfaces.end(); i++)
  {
    delete(*i);
  }
#endif

  //stg_world_destroy( world );// crashes
  cleanup_temp_files();

#ifdef STAGE_UNINIT
  stg_world_destroy(world);
  stg_uninit();
#endif

  ArSocket::shutdown();

  if(opt.before_change_to_directory[0] != '\0')
  {
    if(chdir(opt.before_change_to_directory) != 0)	// this is needed so profiling info can be written to expected directory
    {
      print_warning("MobileSim: Error returning to previous current directory \"%s\"", opt.before_change_to_directory); 
    }
  }


  return stageStat - 1; // stageStat == 1 should cause normal exit (0)
}




void mobilesim_get_map_bounds(double *min_x, double *min_y, double *max_x, double *max_y)
{
  //printf(">>> Mobilesim_get_map_bounds: min_x = %f, min_y = %f, max_x = %f, max_y = %f.\n", map_min_x, map_min_y, map_max_x, map_max_y);
  if(min_x) *min_x = map_min_x;
  if(min_y) *min_y = map_min_y;
  if(max_x) *max_x = map_max_x;
  if(max_y) *max_y = map_max_y;
}

void mobilesim_get_map_home(double *home_x, double *home_y, double *home_th)
{
  if(mobilesim_startplace == mobilesim_start_random)
  {

    if(home_x) {
      if(map_min_x == 0 && map_max_x == 0) 
        *home_x = map_home_x = STG_RANDOM_RANGE(-25000, 25000);
      else
        *home_x = map_home_x = STG_RANDOM_RANGE(map_min_x, map_max_x); 
    }
    if(home_y) {
      if(map_min_y == 0 && map_max_y == 0)
        *home_y = map_home_y = STG_RANDOM_RANGE(-25000, 25000);
      else
        *home_y = map_home_y = STG_RANDOM_RANGE(map_min_y, map_max_y);
    }
    if(home_th) {
      *home_th = map_home_th = STG_RANDOM_RANGE(0, 359);
    }
  }
  else
  {
    if(home_x) *home_x = map_home_x;
    if(home_y) *home_y = map_home_y;
    if(home_th) *home_th = map_home_th;
  }
}

void mobilesim_set_map_bounds(double min_x, double min_y, double max_x, double max_y)
{
  map_min_x = min_x;
  map_min_y = min_y;
  map_max_x = max_x;
  map_max_y = max_y;
}

void mobilesim_set_map_home(double home_x, double home_y, double home_th)
{
  map_home_x = home_x;
  map_home_y = home_y;
  map_home_th = home_th;
}

/* Create a temporary world file and load it: 
   If map is null or empty, will make an empty world of some size with
   the robot in the middle, else will convert an aria map by that
   name and include it in the world.
   You may supply a pointer to a set of stg_model_t*. If non-null,
   each model created while loading the map file is added
   to this list.
   If not null, loop_callback is called several times during the course of creating the world (not at a regular pace, just a chance to do any random updates to UI etc.)
   */
stg_world_t* create_stage_world(const char* mapfile, 
  /*std::map<std::string, std::string>& robotInstanceRequests, * std::list<std::string>& robotFactoryRequests, */
  const char* libdir, 
  //mobilesim_start_place_t startplace, 
  //double start_override_x, double start_override_y, double start_override_th, 
  double world_res, 
  void (*loop_callback)())
{

  char worldfile[MAX_PATH_LEN];

  /* Filenames */

  const char* tempdir = temp_dir();

  // generate stage worldfile:
#ifdef WIN32
  // Don't include the PID on Windows. Temp files aren't properly deleted on
  // Windows (file locking issue?) so we just always replace the old one.
  snprintf(worldfile, MAX_PATH_LEN-1, "%s%cMobileSim-stage_world.world", tempdir, PATHSEPCH);
#else
  snprintf(worldfile, MAX_PATH_LEN-1, "%s%cMobileSim-stage_world-%d.world", tempdir, PATHSEPCH, getpid());
#endif

   ArTime t;

  /* Create stage world file. */

  // XXX TODO don't save it to a file 

  FILE* world_fp = ArUtil::fopen(worldfile, "w");
  if(!world_fp)
  {
    if(options.NonInteractive)
    {
      stg_print_error("MobileSim: could not create temporary file \"%s\" (%s). Is the system temporary directory \"%s\" accessible?", worldfile, strerror(errno), tempdir);
      exit(ERR_TEMPFILE);
    }
    else
    {
      char buf[256];
      snprintf(buf, 255, "Could not write Stage world file \"%s\" (%s).\nIs the system temporary directory \"%s\" accessible?", worldfile, strerror(errno), tempdir);
      stg_gui_fatal_error_dialog("MobileSim: Error creating temprorary file.", buf, ERR_TEMPFILE, FALSE); 
      exit(ERR_TEMPFILE);
    }
  }

  
  // Change locale to good old C to get decimal points rather than commas 
  // in floating point numbers (if locale wants them) (stage expects the C locale, and AM map files
  // use C locale when written).  stg_init is  supposed to do this but I 
  // guess something else changed it back.
  if(!setlocale(LC_ALL, "C"))
    stg_print_warning("MobileSim: failed to set locale to \"C\", Stage world files may not parse correctly!");

  if(loop_callback) (*loop_callback)();
	
  // Write comment, include model definitions, and GUI spec.
  fprintf(world_fp, "# World file for Stage\n# Automatically generated by MobileSim, the MobileRobots/ActivMedia mobile robot simulator.\n\n");
  fprintf(world_fp, "include \"%s%cPioneerRobotModels.world.inc\"\n", libdir, PATHSEPCH);
  fprintf(world_fp, "window\n(\n"\
      "\tscale 0.03\n"\
      "\tmouse_button_pan 3\n"\
      "\tmouse_button_zoom 2\n"\
      "\tlaser_data 1\n"\
      "\tranger_data 1\n"\
      "\tposition_data 0\n"\
      ")\n\n");
  if(world_res > 0) {
    //fprintf(world_fp, "world\n(\n\tresolution %f\n)\n\n", world_res);
    fprintf(world_fp, "resolution %f\n\n", world_res);
  }
  

  if(loop_callback) (*loop_callback)();

  /* Include user's model definition files if they have them. */
#ifdef WIN32
  char* homedir = getenv("USERPROFILE");
  ArUtil::fixSlashesForward(homedir, strlen(homedir));
#else
  char* homedir = getenv("HOME");
#endif
  //printf("%d ms to getenv()\n", t.mSecSince());
  t.setToNow();
  DIR* dir = NULL;
  char includedir[MAX_PATH_LEN];
  if(homedir) 
  {
	strncpy(includedir, homedir, MAX_PATH_LEN-1);
    strncat(includedir, "/.MobileSim/include", MAX_PATH_LEN-1);
    t.setToNow();
    dir = opendir(includedir);
    //printf("%ld ms to call opendir(%s) => 0x%x\n", t.mSecSince(), includedir, dir);
    t.setToNow();
  } 
  
  if(dir)
  {
    stg_print_msg("MobileSim: Checking %s for files to include...", includedir);
    struct dirent* e;
    while((e = readdir(dir)))
    {
#ifndef WIN32
      // Check that the entry is a regular file (or a symlink), except on Windows where we don't have d_type.
      if(e->d_type == DT_REG || e->d_type == DT_LNK)
#endif
      {
        // Don't include a name that starts with a '.' or '#' or ',' or ends with a '~'.
        if(e->d_name[0] == '.' || e->d_name[0] == '#' || e->d_name[0] == ',' || (strlen(e->d_name) > 0 && e->d_name[strlen(e->d_name)-1] == '~'))
        {
          stg_print_msg("MobileSim: skipping include file \"%s/%s\" because it looks like a system, temporary, RCS, or backup file.", includedir, e->d_name);
        }
        else
        {
          stg_print_msg("MobileSim: including \"%s/%s\" into world configuration file...", includedir, e->d_name);
          fprintf(world_fp, "include \"%s/%s\"\n", includedir, e->d_name);
        }

      }
	  if(loop_callback) (*loop_callback)();
    }
    closedir(dir);
  }
 #ifdef WIN32
  // create it for the user since Windows file explorer will prevent you from
  // making a new folder starting with .
  // this is only done on MinGW in Windows. The version of mkdir in MinGW doesn't take permissions argument.
  else if(homedir)
  {
    char msimdir[MAX_PATH_LEN];
    strncpy(msimdir, homedir, MAX_PATH_LEN-1);
    strncat(msimdir, "/.MobileSim", MAX_PATH_LEN-1);
	if(mkdir(msimdir) == 0) 
	{
		printf("Created %s directory.\n", msimdir);
		if(mkdir(includedir) == 0)
		  printf("Created %s directory.\n", includedir);
	}
  }
 #endif

  
  
  /* Load the ActivMedia map file to get the size, dock/home points, etc. */
  // TODO use MapLoader object... 
  // objects?
  map = new ArMap();
  if(mapfile && strlen(mapfile) > 0)
  {
    if(!map->readFile(mapfile))
    {
      if(options.NonInteractive) {
        stg_print_error("MobileSim: Could not load map \"%s\".", mapfile);
        exit(ERR_MAPCONV);
      }
      else {
        char buf[256];
        snprintf(buf, 256, "Error loading map \"%s\".\n.", mapfile);
        stg_gui_fatal_error_dialog("MobileSim: Error loading map", buf, ERR_MAPCONV, FALSE); 
      }
      return NULL;
    }
  }
  else
  {
    /* If mapfile is the empty string, make a 500x500m  empty space. */
    stg_print_warning("MobileSim: No map, using empty simulated space");
    fprintf(world_fp, "# Requested that no map be used:\nsize [500.0 500.0]\n\n");
  }

  if(loop_callback) (*loop_callback)();
  
  /* Remember map bounds for future use */
  map_min_x = map->getLineMinPose().getX();
  map_min_y = map->getLineMinPose().getY();
  map_max_x = map->getLineMaxPose().getX();
  map_max_y = map->getLineMaxPose().getY();

#if 0
  /* Find place to start robot at */
  double startPose.x = 0.0; //m
  double startPose.y = 0.0; //m
  double startPose.a = 0.0; //deg
  switch(startplace)
  {
    case mobilesim_start_fixedpos:
      {
        stg_print_msg("MobileSim: Starting robots at position given by --start option: %f, %f, %f.", start_override_x/1000.0, start_override_y/1000.0, start_override_th);
      map_home_x = start_override_x;
      map_home_y = start_override_y;
      map_home_th = start_override_th;
    }
    break;

    case mobilesim_start_outside:
    {
      map_home_x = map_min_x - 2000.0;
      map_home_y = map_max_y;
      map_home_th = 0;
      stg_print_msg("MobileSim: Starting robots 2m outside edge of map, at: %f, %f, %f.", map_home_x/1000.0, map_home_y/1000.0, map_home_th);
    }
    break;

    case mobilesim_start_random:
    {
      /* Nothing, will be randomly generated below */
      stg_print_msg("MobileSim: Starting robots at random positions in map.");
      map_home_x = map_home_y = map_home_th = 0.0;
    }
    break;


    default: /* home */
    {
      ArMapObject* home = map->findFirstMapObject(NULL, "RobotHome");
      if(!home)
        home = map->findFirstMapObject(NULL, "Dock");
      if(home)
      {
        map_home_x = home->getPose().getX();
        map_home_y = home->getPose().getY();
        map_home_th = home->getPose().getTh();
      }
      stg_print_msg("MobileSim: Starting robots at map's default starting place of: %f, %f, %f", map_home_x/1000.0, map_home_y/1000.0, map_home_th);
    }
  }

  startPose.x = map_home_x / 1000.0;
  startPose.y = map_home_y / 1000.0;
  startPose.a = map_home_th;
#endif

  
#if 0 // TODO move to outside the temp file
  /* Create robot models in the world */
  for(RobotModels::const_iterator i = robotInstanceRequests.begin();
        i != robotInstanceRequests.end(); i++) 
  {
  
    if(loop_callback) (*loop_callback)();
	
    // if the modelname has /, it will cause a syntax error, so find the last
    // one. if the modelname has no /, this procedure should result in an
    // identical string.
    std::string modelname = (*i).second.c_str();
    std::string::size_type lastslash = modelname.rfind('/');
    if(lastslash != 0) lastslash++; // don't include the / we found
    modelname = modelname.substr(lastslash);

    // same thing with robot name, it's annoying for it to be a file path
    std::string robotname = (*i).first.c_str();
    lastslash = modelname.rfind('/');
    if(lastslash != 0) lastslash++; // don't include the / we found
    robotname = robotname.substr(lastslash);
    // TODO: remove the old entry in the robotInstanceRequests map and add this new one.
    // Can't do it while we're iterating it though...
    
    if(startplace == mobilesim_start_random)
    {
      startPose.x = STG_RANDOM_RANGE(map_min_x, map_max_x) / 1000.0;
      startPose.y = STG_RANDOM_RANGE(map_min_y, map_max_y) / 1000.0;
      startPose.a = rand() % 360;
    }

    fprintf(world_fp, "%s\n(\n"\
                      "\tname \"%s\"\n"\
                      "\tpose [%g %g %g]\n",
                      modelname.c_str(), robotname.c_str(), 
                      startPose.x, startPose.y, startPose.a
    );
    fprintf(world_fp, "\tmessages ( )\n");
    fprintf(world_fp, ")\n\n");
    startPose.y -= 1.0;
  }
#endif
  
  /* Close the worldfile, then have stage load it */
  fclose(world_fp);

  // Store filename in global variables so we can delete it on exit:
  strncpy(TempWorldFile, worldfile, 256);

  stg_print_msg("MobileSim: Loading stage world file \"%s\"...", worldfile);
  
  if(loop_callback) (*loop_callback)();
  
  stg_world_t* world = stg_world_create_from_file(worldfile, options.echo_stage_worldfile, loop_callback); 
  if(!world)
  {
    if(options.NonInteractive)
    {
      stg_print_error("MobileSim: Error creating and initializing the world environment. %s", stg_last_error_message);
      cleanup_temp_files();
      exit(ERR_STAGEINIT);
    }
    else
    {
      const char *errormsg = "MobileSim: Error creating and initializing world environment";
      const char *prefixtext = " (worldfile saved as";
      const char *postfixtext = ")";
      const size_t maxlen = strlen(errormsg) + strlen(prefixtext) + MAX_PATH_LEN + strlen(postfixtext);
      char message[maxlen];
      char renamed_worldfile[MAX_PATH_LEN];
      snprintf(renamed_worldfile, MAX_PATH_LEN-1, "%s%cMobileSim-stage_world_ERROR.world", tempdir, PATHSEPCH);
      if(rename(worldfile, renamed_worldfile) != -1)
      {
        snprintf(message, maxlen-1, "%s%s%s%s", errormsg, prefixtext, renamed_worldfile, postfixtext);
      }
      else
      {
        strncpy(message, errormsg, maxlen-1);
      }
      cleanup_temp_files();
      stg_gui_fatal_error_dialog(message, stg_last_error_message, ERR_STAGEINIT, FALSE); 
          // this function exits the program when user clicks OK
    }
    return NULL;
  }



  /* Create models for the map and map objects */
  if(mapfile && (strlen(mapfile) > 0) && world)
  {
    stg_print_msg("MobileSim: Loading map data from \"%s\"...", mapfile);
    //StageMapLoader loader(&map, world, NULL);
    mapLoader.setWorld(world);
    if(!mapLoader.newMap(map) && !mapLoader.process(0))
    {
      stg_print_error("MobileSim: Error loading map \"%s\"!", mapfile);
      exit(ERR_MAPCONV);
    }
    stg_print_msg("MobileSim: Finished loading map data from \"%s\".", mapfile);
  }
  

  return world;
}

void cleanup_temp_files()
{

#ifdef CLEANUP_TEMP_FILES

  if(TempWorldFile == NULL || strlen(TempWorldFile) == 0)
    return;

#ifdef WIN32
  // On Windows we must change '/' to '\\':
  for(char* c = TempWorldFile; c && *c; c++)
    if(*c == '/') *c = '\\';
#endif


  if(unlink(TempWorldFile) != 0)
    stg_print_warning("MobileSim: Failed to delete temporary file \"%s\".", TempWorldFile);
#else
  stg_print_warning("MobileSim: Temporary world file has been left behind: \"%s\"; .", TempWorldFile);
#endif

}



const char* find_libdir()
{
  const char* libdir = getenv("MOBILESIM");
  //unused? char buf[256];
  if(libdir == NULL)
  {

#ifdef WIN32
    char buf[1024];
    if (ArUtil::getStringFromRegistry(ArUtil::REGKEY_LOCAL_MACHINE,
        "SOFTWARE\\MobileRobots\\MobileSim", "Install Directory", buf, 255))
    {
      libdir = strdup(buf);
      stg_print_msg("MobileSim: Expecting supporting resources to be installed in \"%s\" (according to registry key).", libdir);
    }
    else if (ArUtil::getStringFromRegistry(ArUtil::REGKEY_LOCAL_MACHINE,
        "SOFTWARE\\ActivMedia Robotics\\MobileSim", "Install Directory", buf, 255))
    {
      libdir = strdup(buf);
      stg_print_msg("MobileSim: Expecting supporting resources to be installed in \"%s\" (according to ActivMedia Robotics registry key).", libdir);
    }
    else  
    {
      libdir = MOBILESIM_DEFAULT_DIR; // "\\Program Files\\MobileRobots\\MobileSim";
      stg_print_msg("MobileSim: Expecting supporting resources to be installed in the default location: \"%s\".", libdir);
    }

#else 

    libdir = MOBILESIM_DEFAULT_DIR; // "/usr/local/MobileSim";
    stg_print_msg("MobileSim: Expecting supporting resources to be installed in the default location: \"%s\".", libdir);
#endif

  } 
  else 
  {
    stg_print_msg("MobileSim: Expecting supporting resources to be installed in \"%s\" (according to MOBILESIM environment variable).", libdir);
  }
  return libdir;
}


/*
unusued?
void add_model_to_robot_type_map_if_position(stg_model_t* model, char* name, void* map_p)
{
  RobotTypeMap *map = (RobotTypeMap*)map_p;
  if(strcmp(stg_model_get_base_type_name(model), "position") == 0)
  {
    std::string type;
    std::string subtype;
    // property is not null terminated:

    size_t l;
    char* t = (char*)stg_model_get_property(model, "pioneer_robot_subtype", &l);
    if(t == NULL)
    {
      subtype = stg_model_get_token(model);
      stg_print_msg("Warning: World position model \"%s\" does not have pioneer_robot_subtype property. Using \"%s\" for robot type, this may be invalid.", name, subtype.c_str());
    } else {
      subtype.assign(t, l);
    }

    t = (char*)stg_model_get_property(model, "pioneer_robot_type", &l);
    if(t == NULL)
      type = "Pioneer";
    else
      type.assign(t, l);

  
    map->insert( std::pair< std::string, std::pair<std::string, std::string> >(name, std::pair<std::string, std::string>(type, subtype)) );
  }
}
*/





GtkWidget *busy_loading_dialog()
{
  GtkMessageDialog *dialog;
  dialog = GTK_MESSAGE_DIALOG(gtk_message_dialog_new(NULL, (GtkDialogFlags)0, GTK_MESSAGE_INFO, GTK_BUTTONS_NONE, "Creating simulation world..."));
  gtk_window_set_title(GTK_WINDOW(dialog), "MobileSim");
#if GTK_CHECK_VERSION(2, 22, 0)
  // message_area added in 2.22, just skip spinner if older.
  GtkWidget *msgarea = gtk_message_dialog_get_message_area(dialog);
  GtkWidget *spinner = gtk_spinner_new();
  gtk_box_pack_end(GTK_BOX(msgarea), spinner, FALSE, FALSE, 0);
  gtk_widget_show(spinner);
  gtk_spinner_start(GTK_SPINNER(spinner));
#endif
  gtk_widget_show_now(GTK_WIDGET(dialog));
  //gtk_dialog_run(GTK_DIALOG(dialog));
  return(GTK_WIDGET(dialog));
}


int map_options_dialog(std::string& map, RobotModels *robotInstanceRequests, RobotFactoryRequests *robotFactoryRequests, MobileSim::Options *opt)
{
  int ret;
  GtkDialog* dialog;
  GTKFILECHOOSER* filechooser;
  GtkEntry* portentry = NULL;
  GtkEntry* numentry = NULL;
  GtkRadioButton *factory_radio = NULL;
  GtkRadioButton *multirobot_radio = NULL;
  GTKCOMBOWIDGET* modelmenu = NULL;
  gboolean include_modelmenu = (robotInstanceRequests != NULL && robotFactoryRequests != NULL);
  GtkWidget *verbosecheck = NULL;
  GtkWidget *warnunsuppcheck = NULL;


#ifdef USE_GTKFILECHOOSER

  dialog = GTK_DIALOG(
    gtk_file_chooser_dialog_new("MobileSim: Load Map File...", NULL, 
      GTK_FILE_CHOOSER_ACTION_OPEN, 
      "No Map", GTK_RESPONSE_CANCEL, 
      "Load Map", GTK_RESPONSE_ACCEPT, 
    NULL)
  );
  filechooser = GTK_FILE_CHOOSER(dialog);

  GtkFileFilter* filter_maps = gtk_file_filter_new();
  GtkFileFilter* filter_all = gtk_file_filter_new();
  // TODO: need to ref the filters and destroy in the destructor?
  gtk_file_filter_set_name(filter_maps, "MobileRobots/ActivMedia Map Files");
  gtk_file_filter_add_pattern(filter_maps, "*.map");
  gtk_file_filter_set_name(filter_all, "All Files");
  gtk_file_filter_add_pattern(filter_all, "*");
  gtk_file_chooser_add_filter(filechooser, filter_maps);
  gtk_file_chooser_add_filter(filechooser, filter_all);
  gtk_file_chooser_set_current_folder(filechooser, user_docs_dir());
#ifdef WIN32
  const char *msim_dir = "file:///Program Files/MobileRobots/MobileSim";
  const char *aria_map_dir = "file:///Program Files/MobileRobots/Aria/maps";
  const char *arnl_example_dir = "file:///Program Files/MobileRobots/Arnl/examples";
#else
  const char *msim_dir = "file:///usr/local/MobileSim";
  const char *aria_map_dir = "file:///usr/local/Aria/maps";
  const char *arnl_example_dir = "file:///usr/local/Arnl/examples";
#endif
  gtk_file_chooser_add_shortcut_folder_uri(filechooser, msim_dir, NULL);
  gtk_file_chooser_add_shortcut_folder_uri(filechooser, aria_map_dir, NULL);
  gtk_file_chooser_add_shortcut_folder_uri(filechooser, arnl_example_dir, NULL);

#else
  char prev_dir[MAX_PATH_LEN];
  getcwd(prev_dir, MAX_PATH_LEN);
  chdir(user_docs_dir());
  filechooser = GTK_FILE_SELECTION(gtk_file_selection_new("MobileSim: Load Map File..."));
  dialog = GTK_DIALOG(filechooser);
  gtk_file_selection_hide_fileop_buttons(filechooser);
  gtk_button_set_label(GTK_BUTTON(filechooser->ok_button), "Load Map");
  gtk_button_set_label(GTK_BUTTON(filechooser->cancel_button), "No Map");

#endif

  gtk_dialog_set_default_response(GTK_DIALOG(dialog), GTK_RESPONSE_ACCEPT);

  // opt: select model, advanced, etc.:
  {   
      /* opt Boxes
       *   0              1              2
       * 0 +--------------+--------------+ 0
       *   | Robot Model: | [Model Menu] |
       * 1 +--------------+--------------+ 1
       *   | > More options              |
       * 2 +-----------------------------+
       *
       * More options Expandable panel:
       *
       *   Multiple Robots: 
       *    (o) Create [Num Entry] robot(s), starting at port [Port Entry]
       *    ( ) Create a new robot for each connection to port [Port Entry]
       *
       *   [ ] Verbose Logging
       *   [ ] Warn about unsupported commands received
       */


    GtkFrame* optframe = GTK_FRAME(gtk_frame_new("Options"));
    gtk_widget_show(GTK_WIDGET(optframe));

    /*
    GtkTable* opttable = GTK_TABLE(gtk_table_new(2, 2, FALSE));
    gtk_container_add(GTK_CONTAINER(optframe), GTK_WIDGET(opttable));
    gtk_widget_show(GTK_WIDGET(opttable));
    */

    GtkBox* optbox = GTK_BOX(gtk_vbox_new(FALSE, 10));
    gtk_container_set_border_width(GTK_CONTAINER(optbox), 10);
    gtk_container_add(GTK_CONTAINER(optframe), GTK_WIDGET(optbox));
    gtk_widget_show(GTK_WIDGET(optbox));
    
    //const guint pad = 5;

    // Menu to select a robot model to create, unless -r was given:
    if(include_modelmenu)
    {
      GtkBox* modelbox = GTK_BOX(gtk_hbox_new(FALSE, 10));
      gtk_widget_show(GTK_WIDGET(modelbox));
      gtk_box_pack_start(optbox, GTK_WIDGET(modelbox), FALSE, FALSE, 0);

      GtkLabel* modellabel = GTK_LABEL(gtk_label_new("Robot Model:"));
      gtk_widget_show(GTK_WIDGET(modellabel));

      /*
      gtk_misc_set_alignment(GTK_MISC(modellabel), 1, 0.5);
      gtk_table_attach(opttable, GTK_WIDGET(modellabel), 0, 1, 0, 1, GTK_FILL, GTK_FILL, pad, pad);
        // L, R, T, B, xopt, yopt, xpad, ypad
      */

      gtk_box_pack_start(modelbox, GTK_WIDGET(modellabel), FALSE, FALSE, 0);

# ifdef USE_GTKCOMBOBOX
# if GTK_CHECK_VERSION(2, 4, 0) // check 2,6,0 instead?s
      modelmenu = GTK_COMBO_BOX(gtk_combo_box_entry_new_text());
# else
      modelmenu = GTK_COMBO_BOX(gtk_combo_box_new_text());
# endif

      for(const char** p = CommonRobotModels; *p; p++) {
        gtk_combo_box_append_text(modelmenu, *p);
      }
      gtk_combo_box_set_active(modelmenu, 0);

#else   /* (not using newer combobox) */ 

      modelmenu = GTK_COMBO(gtk_combo_new());
      GList* items = NULL;
      for(const char** m = CommonRobotModels; *m != 0; m++ ) 
      {
        items = g_list_append(items, (gchar*)*m);
      }
      gtk_combo_set_popdown_strings(modelmenu, items);
      gtk_entry_set_text(GTK_ENTRY(modelmenu->entry), CommonRobotModels[0]);

#endif

      gtk_widget_show(GTK_WIDGET(modelmenu));
      /*
      gtk_table_attach(opttable, GTK_WIDGET(modelmenu), 1, 2, 0, 1, GTK_FILL, GTK_FILL, pad, pad);
        // L, R, T, B, xopt, yopt, xpad, ypad
      */
      gtk_box_pack_start(modelbox, GTK_WIDGET(modelmenu), FALSE, FALSE, 0);
    }


    // - Advanced opt for multirobots:
    //
    
#ifdef USE_GTKEXPANDER
    GtkContainer *more = GTK_CONTAINER(gtk_expander_new("More options"));
#else
    GtkContainer *more = GTK_CONTAINER(gtk_frame_new(NULL));
#endif
    gtk_box_pack_start(optbox, GTK_WIDGET(more), FALSE, FALSE, 0);

    GtkBox *more_box = GTK_BOX(gtk_vbox_new(FALSE, 0));
    gtk_container_add(more, GTK_WIDGET(more_box));
    gtk_widget_show(GTK_WIDGET(more_box));
    

    // Only include num. robot opt if neither -r or -R given
    if(include_modelmenu)
    {

      /*    (o) Create [Num Entry] robot(s), starting at port [Port Entry] */
      GtkBox *multirobot_row = GTK_BOX(gtk_hbox_new(FALSE, 0));
      gtk_widget_show(GTK_WIDGET(multirobot_row));
      gtk_box_pack_start(more_box, GTK_WIDGET(multirobot_row), FALSE, FALSE, 3);
      multirobot_radio = GTK_RADIO_BUTTON(gtk_radio_button_new_with_label(NULL, "Create"));
      gtk_widget_show(GTK_WIDGET(multirobot_radio));
      gtk_box_pack_start(multirobot_row, GTK_WIDGET(multirobot_radio), FALSE, FALSE, 5);
      numentry = GTK_ENTRY(gtk_spin_button_new_with_range(1, ROBOTS_MENU_LIMIT, 1));
      gtk_widget_show(GTK_WIDGET(numentry));
      gtk_box_pack_start(multirobot_row, GTK_WIDGET(numentry), FALSE, FALSE, 5);
      GtkWidget *label = gtk_label_new(" robot(s) on seperate TCP ports.");
      gtk_widget_show(label);
      gtk_box_pack_start(multirobot_row, label, FALSE, FALSE, 0);

      /*    ( ) Create a new robot for each connection to port [Port Entry] */
      GtkBox *factory_row = GTK_BOX(gtk_hbox_new(FALSE, 0));
      gtk_widget_show(GTK_WIDGET(factory_row));
      gtk_box_pack_start(more_box, GTK_WIDGET(factory_row), FALSE, FALSE, 3);
      factory_radio = GTK_RADIO_BUTTON(gtk_radio_button_new_with_label(gtk_radio_button_get_group(GTK_RADIO_BUTTON(multirobot_radio)), "Create a new robot for each connection."));
      gtk_widget_show(GTK_WIDGET(factory_radio));
      gtk_box_pack_start(factory_row, GTK_WIDGET(factory_radio), FALSE, FALSE, 5);
    }

    /*    Port: [8101] */
    GtkBox *port_row = GTK_BOX(gtk_hbox_new(FALSE, 0));
    gtk_widget_show(GTK_WIDGET(port_row));
    gtk_box_pack_start(more_box, GTK_WIDGET(port_row), FALSE, FALSE, 15);
    GtkWidget *label = gtk_label_new("TCP Port: ");
    gtk_widget_show(label);
    gtk_box_pack_start(port_row, label, FALSE, FALSE, 10);
    portentry = GTK_ENTRY(gtk_spin_button_new_with_range(1024, 65535, 1)); // 1024..65535 is the allowed range for non-privilaged TCP ports
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(portentry), opt->port);
    gtk_widget_show(GTK_WIDGET(portentry));
    gtk_box_pack_start(port_row, GTK_WIDGET(portentry), FALSE, FALSE, 1);

    verbosecheck = gtk_check_button_new_with_label("Verbose Logging");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(verbosecheck), opt->verbose);
    gtk_widget_show(verbosecheck);
    gtk_box_pack_start(more_box, verbosecheck, FALSE, FALSE, 15);

    warnunsuppcheck = gtk_check_button_new_with_label("Warn when unsupported command received");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(warnunsuppcheck), opt->warn_unsupported_commands);
    gtk_widget_show(warnunsuppcheck);
    gtk_box_pack_start(more_box, warnunsuppcheck, FALSE, FALSE, 15);

    gtk_widget_show(GTK_WIDGET(more));

/****** Old table: 
 * 
 *   0              1              2   3              4              5
 * 0 +--------------+--------------+---+--------------+--------------+ 0
 *   | Num. Robots: | [Num Entry ] |   | Start Port:  | [Port Entry] |
 * 1 +--------------+--------------+---+--------------+--------------+ 1
 */
#if 0

    {
#ifdef USE_GTKEXPANDER
      GtkExpander* more = GTK_EXPANDER(gtk_expander_new("More opt"));
#else
      GtkFrame* more = GTK_FRAME(gtk_frame_new(NULL));   // It's obvious that they are "more" :)
#endif
      gtk_widget_show(GTK_WIDGET(more));
      /*
      gtk_table_attach(opttable, GTK_WIDGET(more), 0, 2, 1, 2, GTK_FILL, GTK_FILL, pad, pad);
        // L, R, T, B, xopt, yopt, xpad, ypad
      */
      gtk_box_pack_start(optbox, GTK_WIDGET(more), FALSE, FALSE, 0);

      GtkTable* moretable = GTK_TABLE(gtk_table_new(1, 5, FALSE));
      gtk_container_add(GTK_CONTAINER(more), GTK_WIDGET(moretable));
      gtk_widget_show(GTK_WIDGET(moretable));

      // Num. Robots, unless -r was given above:
      if(include_modelmenu)
      {
        GtkLabel* numlabel = GTK_LABEL(gtk_label_new("Num. Robots:"));
        gtk_widget_show(GTK_WIDGET(numlabel));
        gtk_misc_set_alignment(GTK_MISC(numlabel), 1, 0.5);
        gtk_table_attach(moretable, GTK_WIDGET(numlabel), 0, 1, 0, 1, GTK_FILL, GTK_FILL, pad, pad); 
          // L, R, T, B, xopt, yopt, xpad, ypad
        numentry = GTK_ENTRY(gtk_spin_button_new_with_range(1, ROBOTS_MENU_LIMIT, 1));
        gtk_widget_show(GTK_WIDGET(numentry));
        gtk_table_attach(moretable, GTK_WIDGET(numentry), 1, 2, 0, 1, GTK_FILL, GTK_FILL, pad, pad); 
          // L, R, T, B, xopt, yopt, xpad, ypad
      }

      // Spacer in column 2,3
      gtk_table_set_col_spacing(moretable, 2, 40);

      // Starting port, defaulting to 8101 or value given with -p:
      {
        GtkLabel* portlabel = GTK_LABEL(gtk_label_new("First TCP Port:"));
        gtk_widget_show(GTK_WIDGET(portlabel));
        gtk_misc_set_alignment(GTK_MISC(portlabel), 1, 0.5);
        gtk_table_attach(moretable, GTK_WIDGET(portlabel), 3, 4, 0, 1, GTK_FILL, GTK_FILL, pad, pad);
          // L, R, T, B, xopt, yopt, xpad, ypad
        portentry = GTK_ENTRY(gtk_spin_button_new_with_range(1024, 65535, 1)); // 1024..65535 is the allowed range for non-privilaged TCP ports
        gtk_widget_show(GTK_WIDGET(portentry));
        gtk_table_attach(moretable, GTK_WIDGET(portentry), 4, 5, 0, 1, GTK_FILL, GTK_FILL, pad, pad); 
          // L, R, T, B, xopt, yopt, xpad, ypad
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(portentry), *port);
      }
    }
#endif

#ifdef USE_GTKFILECHOOSER
    gtk_file_chooser_set_extra_widget(filechooser, GTK_WIDGET(optframe));

    // Make the dialog a bit taller, to help deal with long lists of maps,
    // and also when you expand the more options expander.
    GtkRequisition szreq;
    gtk_widget_size_request(GTK_WIDGET(dialog), &szreq);
    gtk_widget_set_size_request(GTK_WIDGET(dialog), szreq.width, szreq.height * 2);
#else
    gtk_box_pack_end(GTK_BOX(dialog->vbox), GTK_WIDGET(optframe), TRUE, TRUE, 0);
#endif
  }




  // Show dialog and check responses
  int r = gtk_dialog_run(dialog);
  if(r == GTK_RESPONSE_ACCEPT || r == GTK_RESPONSE_OK) {
    // selected a map.
#ifdef USE_GTKFILECHOOSER
    map = (char*)gtk_file_chooser_get_filename(filechooser);
#else
    map = (char*)gtk_file_selection_get_filename(filechooser);
#endif
    ret = 1;
  } else if( r == GTK_RESPONSE_DELETE_EVENT ) {
    // closed dialog box. exit program.
    ret = 0;
    exit(0);
  } else {
    // clicked "No Map"
    map = "";
    ret = 2;
  }

  // Check selected robot model
  if(modelmenu)
  {
    std::string selected_robot;
#ifdef USE_GTKCOMBOBOX
# if GTK_CHECK_VERSION(2,6,0)
    selected_robot = gtk_combo_box_get_active_text(modelmenu);
# else
    gint i = gtk_combo_box_get_active(modelmenu);
    if(i != -1)
      selected_robot = CommonRobotModels[i];
# endif
#else
    selected_robot = (char*)gtk_entry_get_text(GTK_ENTRY(modelmenu->entry));
#endif
    if(selected_robot == "")
      selected_robot = "p3dx-sh";

    if(multirobot_radio && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(multirobot_radio)))
    {
      int n = 1;
      if(numentry)
        n = (int) gtk_spin_button_get_value(GTK_SPIN_BUTTON(numentry));

      std::string name = selected_robot;
      for(int i = 0; i < n; i++)
      {
        (*robotInstanceRequests)[name] = selected_robot;
        char numstr[8];
        snprintf(numstr, 8, "_%03d", i+2);
        name = selected_robot + numstr;
      }
    }

    else if(factory_radio && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(factory_radio)))
    {
      stg_print_msg("MobileSim: Will create a robot factory for model \"%s\".", selected_robot.c_str());
      robotFactoryRequests->push_back(selected_robot);
    }

  }

  // other options
  opt->port = (int) gtk_spin_button_get_value(GTK_SPIN_BUTTON(portentry));
  opt->verbose = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(verbosecheck));
  opt->warn_unsupported_commands = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(warnunsuppcheck));

  // hide file dialog
  gtk_widget_hide(GTK_WIDGET(dialog));
  // TODO: destroy widgets
 
#ifndef USE_GTKFILECHOOSER
  // go back to previous working directory if we changed for the file dialog
  chdir(prev_dir);
#endif

  return ret;
}


void print_error(const char *m, ...)
{
  va_list args;
  va_start(args, m);
  stg_print_error_v(m, args);
  va_end(args);
  fflush(stg_output_file);
}

void print_msg(const char *m, ...)
{
  va_list args;
  va_start(args, m);
  stg_print_msg_v(m, args);
  va_end(args);
}

void print_debug(const char *m, ...)
{
  char *formatstr = (char*)malloc(strlen(m) + strlen("DEBUG :") + 1); // + 1 for terminating NULL
  strcpy(formatstr, "DEBUG: ");
  strncpy(formatstr + strlen("DEBUG: "), m, strlen(m) + 1); // + 1 includes terminating NULL
  va_list args;
  va_start(args, m);
  stg_print_msg_v(formatstr, args);
  va_end(args);
  free(formatstr);
  fflush(stg_output_file);
}

void print_warning(const char *m, ...) 
{
  va_list args;
  va_start(args, m);
  stg_print_warning_v(m, args);
  va_end(args);
}

std::string getMapName()
{
  return mapLoader.getMapName();
}

void MobileSim::Options::log_argv()
{
  std::string s;
  for(int i = 0; i < argc; ++i)
  {
    s += argv[i];
    s += " ";
  }
  print_msg("MobileSim command options were: %s", s.c_str());
}
