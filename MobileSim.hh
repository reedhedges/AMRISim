/*
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

#ifndef _MOBILESIM_HH_
#define _MOBILESIM_HH_

#include "ariaUtil.h"
#include <set>
#include <iostream>
#include <assert.h>
//#include <sys/select.h>
#include <string>
#include <stdexcept>

class ArFunctor;
class ArSocket;

#define PATHSEPCH '/'
#define OTHER_PATHSEPCH '\\'


/* Program version */

#ifndef MOBILESIM_VERSION
#error MOBILESIM_VERSION not defined!
#endif

/* Size of buffers used to store file path names */
#define MAX_PATH_LEN 512

/* Error codes */
#define ERR_USAGE -1 // becomes 255
#define ERR_TEMPFILE -2 //becomes 254
#define ERR_MAPCONV -3 // becomes 253
#define ERR_MAPLOAD -3 // becomes 253
#define ERR_STAGELOAD -4 // becomes 252
#define ERR_STAGEINIT -4 // becomes 252
#define ERR_FORK -5   // becomes 251
#define ERR_PFILE -6 // becomes 250
#define ERR_ROBOT_INIT -7 // becomes 249
#define ERR_OPEN_PORT -8 // becomes 248
#define ERR_CHDIR -9
// -10 through -19 are used by the EmulatePioneer class.
#define ERR_CRASH -20 // becomes 236


/* Default real update interval */
#define DEFAULT_UPDATE_INTERVAL 100

/* Default MapLoad model chunk sizes */
#define DEFAULT_MAP_LINES_PER_CHUNK   10000 // Please update the DEFAULT_MLPC_STR if you change this
#ifndef DEFAULT_MLPC_STR
#define DEFAULT_MLPC_STR "10,000"
#endif

#define DEFAULT_MAP_POINTS_PER_CHUNK 100000 // Please update the DEFAULT_MPPC_STR if you change this
#ifndef DEFAULT_MPPC_STR
#define DEFAULT_MPPC_STR "100,000"
#endif

namespace MobileSim
{

class Options
{
public:
  int argc;
  char **argv;
  void log_argv();
  std::string map;
  int port;
  enum {NORMAL_WINDOW, MAXIMIZE_WINDOW, MINIMIZE_WINDOW, FULLSCREEN_WINDOW} windowmode;
  enum {NORMAL_GRAPHICS, LITE_GRAPHICS, NO_GRAPHICS, NO_GUI} graphicsmode;
  bool NonInteractive;
  bool Daemon;
  bool log_html;
  char* change_to_directory;
  char before_change_to_directory[MAX_PATH_LEN];
  unsigned int interval_real;
  unsigned int interval_sim;
  double world_res;
  long int start_pos_override_pose_x;
  long int start_pos_override_pose_y;
  int start_pos_override_pose_th;
  std::set<int> ignore_commands;
  bool verbose;
  const char* log_file;
  std::string listen_address;
  size_t log_file_max_size;
  bool srisim_compat;
  bool srisim_laser_compat;
  bool nomap;
  bool RestartedAfterCrash;
  bool EnableCrashDebug;
  bool EnableCrashRestart;
  bool log_packets_received;
  bool echo_stage_worldfile;
  bool warn_unsupported_commands;
  bool log_sips_sent;
  bool onHostWithEM;
  size_t mapLoadLinesPerChunk;
  size_t mapLoadPointsPerChunk;
  bool run_network_discovery;
  enum {RANDOM_INIT, RANDOM_EACH_UPDATE, CONSTANT, NONE} odom_error_mode;

  Options()
  {
      port = 8101;
      windowmode = NORMAL_WINDOW;
      graphicsmode = NORMAL_GRAPHICS;
      NonInteractive = false;
      Daemon = false;
      log_html = false;
      change_to_directory = 0;
      interval_real = DEFAULT_UPDATE_INTERVAL;
      interval_sim = 0;
      world_res = 0;
      verbose = false;
      log_file = NULL;
      listen_address = "";
      log_file_max_size = 0;
      srisim_compat = false;
      srisim_laser_compat = true;
      nomap = false;
      RestartedAfterCrash = false;
      EnableCrashDebug = true;
      EnableCrashRestart = false;
      before_change_to_directory[0] = 0;
      log_packets_received = false;
      echo_stage_worldfile = false;
      warn_unsupported_commands = false;
      log_sips_sent = false;
      onHostWithEM = false;
      mapLoadLinesPerChunk = DEFAULT_MAP_LINES_PER_CHUNK;
      mapLoadPointsPerChunk = DEFAULT_MAP_POINTS_PER_CHUNK;
      run_network_discovery = true;
#ifdef MOBILESIM_DEFAULT_ODOM_ERROR_MODE
      odom_error_mode = MOBILESIM_DEFAULT_ODOM_ERROR_MODE;
#else
      odom_error_mode = RANDOM_EACH_UPDATE;
#endif
   }
};


}// namespace MobileSim

extern MobileSim::Options options;

/* Figure out what directory to use for our external resources */
extern const char* find_libdir();

/* Functions used as callbacks */
typedef void(mobilesim_get_bounds_cb_t)(double*, double*, double*, double*);
typedef void(mobilesim_get_pose_cb_t)(double*, double*, double*);

extern mobilesim_get_bounds_cb_t mobilesim_get_map_bounds;
extern void mobilesim_get_map_home(double *home_x, double *home_y, double *home_th);
extern mobilesim_get_pose_cb_t mobilesim_set_map_home;
extern void mobilesim_set_map_bounds(double min_x, double min_y, double max_x, double max_y);

/* How to decide where robots start */
typedef enum {
  mobilesim_start_fixedpos,
  mobilesim_start_home,
  mobilesim_start_outside,
  mobilesim_start_random
} mobilesim_start_place_t;

extern void mobilesim_crash_handler(int signal);
extern int mobilesim_rotate_log_files(const char *tag);
extern FILE* mobilesim_reopen_log_file();

class RobotFactory;
class RobotInterface;
class EmulatePioneer;

extern std::set<RobotInterface*> robotInterfaces;
extern std::set<EmulatePioneer*> emulators;
extern std::set<RobotFactory*> factories;

//void storeEmulator(EmulatePioneer *em);
//void removeStoredEmulator(EmulatePioneer *em);
//void storeRobotInterface(RobotInterface *em);
//void removeStoredRobotInterface(RobotInterface *em);

void print_error(const char *m, ...);
void print_msg(const char* m, ...);
void print_warning(const char* m, ...);

/// Same as print_msg but prefixes output with DEBUG and function calls are easily searched for removal
void print_debug(const char *m, ...);

class MapLoader;

extern MapLoader mapLoader;

std::string getMapName();

class LogInterface {
  std::string name;
public:

  LogInterface(std::string _name) : name(_name) {}

  virtual ~LogInterface() {}

  /** display fatal error message. this default implementation
      prints it to cerr with some pretty colors followed by a newline.
  */
  virtual void error_s(const char* message) {
    std::cerr << name << ": \033[41mError:\033[0m " << message << std::endl << std::flush;
  }

  /** display warning message. this default implementation
      prints it to cerr with some pretty colors followed by a newline.
  */
  virtual void warn_s(const char* message) {
    std::cerr << name << ": \033[41mWarning:\033[0m " << message << std::endl << std::flush;
  }

  /** display informational message. this default implementation
      prints it to clog with some pretty colors followed by a newline.
  */
  virtual void inform_s(const char* message) {
    std::clog << name << ": " << message << std::endl << std::flush;
  }

  /** Log informational message (with newline), to console or log file but don't display in a GUI. */
  virtual void log_s(const char* message) {
    assert(message);
    std::clog << name << ": " << message << std::endl << std::flush;
  }

  virtual void log_error_s(const char *message) {
      error_s(message);
  }


  void error_v(const char *format, va_list args)
  {
    char buf[1024];
    vsnprintf(buf, 1024, format, args);
    error_s(buf);
  }

  void inform_v(const char *format, va_list args)
  {
    char buf[1024];
    vsnprintf(buf, 1024, format, args);
    inform_s(buf);
  }

  void warn_v(const char *format, va_list args)
  {
    char buf[1024];
    vsnprintf(buf, 1024, format, args);
    warn_s(buf);
  }

  void error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    error_v(format, args);
    va_end(args);
  }
  void warn(const char* format, ...)  {
    va_list args;
    va_start(args, format);
    warn_v(format, args);
    va_end(args);
  }
  void inform(const char* format, ...)  {
    va_list args;
    va_start(args, format);
    inform_v(format, args);
    va_end(args);
  }
  void log(const char* format, ...)  {
    va_list args;
    va_start(args, format);
    char buf[1024];
    vsnprintf(buf, 1024, format, args);
    va_end(args);
    log_s(buf);
  }

  /** Like error() but happens immediately, and is written to log file or
      other output rather than the GUI. Use for fatal errors where program exit
      is imminent.
  */
  virtual void log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buf[1024];
    vsnprintf(buf, 1024, format, args);
    va_end(args);
    log_error_s(buf);
  }

};

namespace MobileSim
{
  inline double min(double x, double y) { return ((x <= y) ? x : y); }
  inline double max(double x, double y) { return ((x >= y) ? x : y); }
  inline int min(int x, int y) { return ((x <= y) ? x : y); }
  inline int max(int x, int y) { return ((x >= y) ? x : y); }
  inline long int min(long int x, long int y) { return ((x <= y) ? x : y); }
  inline long int max(long int x, long int y) { return ((x >= y) ? x : y); }

  /** Wrapper around usleep. (Don't use ArUtil::sleep(), it adds error) */
  inline void sleep(unsigned long msec) 
  { 
    if(msec > 0) 
#ifdef WIN32
      Sleep(msec);
      //_sleep(msec);
#else
      usleep(msec * 1000); 
#endif
  }


  extern unsigned long log_stats_freq;



/* function to display a byte as a string of 8 '1' and '0' characters. */
inline std::string byte_as_bitstring(char byte) 
{
  char tmp[9];
  int bit; 
  int ch;
  for(bit = 7, ch = 0; bit >= 0; bit--,ch++)
    tmp[ch] = ((byte>>bit)&1) ? '1' : '0';
  tmp[8] = 0;
  return std::string(tmp);
}

/* function to display a 2-byte int as a string of 16 '1' and '0' characters. */
inline std::string int16_as_bitstring(int16_t n) 
{
  char tmp[17];
  int bit;
  int ch;
  for(bit = 15, ch = 0; bit >= 0; bit--, ch++)
    tmp[ch] = ((n>>bit)&1) ? '1' : '0';
  tmp[16] = 0;
  return std::string(tmp);
}


// Interface for exceptions used by callbacks to delete instances outside the callback method within that instance.
class DeletionRequest : public std::runtime_error
{
public:
  DeletionRequest() : std::runtime_error("Object requested deletion after callback") {}
  virtual void doDelete() = 0;
};



}; // namespace MobileSim
#endif
