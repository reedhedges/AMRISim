
/*  
 *  AMRISim is based on MobileSim (Copyright 2005 ActivMedia Robotics, 2006-2010 
 *  MobileRobots Inc, 2011-2015 Adept Technology, 2016-2017 Omron Adept Technologies)
 *  and Stage version 2 (Copyright Richard Vaughan, Brian Gerkey, Andrew Howard, and 
 *  others), published under the terms of the GNU General Public License version 2.
 *
 *  Copyright 2018 Reed Hedges and others
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

#ifndef _EP_STAGE_MAP_LOADER_HH_
#define _EP_STAGE_MAP_LOADER_HH_

#include <set>
#include <string>
#include <vector>
#include <memory>

#include "stage.h"

#include "Aria/ariaUtil.h"
#include "Aria/ArGPSCoords.h"


class ArMap;
class ArMapObject;
class RobotInterface;
class ArFunctor;

struct MapLoadedInfo {
  double min_x = 0;
  double min_y = 0;
  double max_x = 0;
  double max_y = 0;
  double home_x = 0;
  double home_y = 0;
  double home_th = 0;
  bool have_home = false;
  std::string filename;
  int status = 1; ///< 0=not reloaded, already loaded; 1=loaded; 2=error, file not found
  std::shared_ptr<ArMap> map;
};

typedef ArFunctor1<MapLoadedInfo>* MapLoadedCallback;
  ///< minx, miny, maxx, maxy, homex, homey, hometh

/** Load data from an ActivMedia/MobileRobots map file into
 * a Stage world, and keep track of models created. This class contains
 * the methods to do so, as well as keeps track of in-progress loading (loading
 * can be performed incrementally to avoid blocking the program for too long while doing so).
 * Generally this class is meant to be a singleton (AMRISim should just keep one instance to track loading).
 */
class MapLoader
{
private:
  stg_world_t *world = NULL;
  //StageInterface *interface;
  std::string mapfile;
  //MapLoadedCallback callback;
  std::set<MapLoadedCallback> callbacks;
  static const double ReflectorThickness; ///< meters
  std::set<stg_model_t*> mapModels; ///< all models created by loading maps
  std::shared_ptr<ArMap> map; ///< points to an ArMap object while we are using it to load a map
  //bool created_map = false; ///< we created map, and can delete it.
  bool loading = false; ///< true while we are in the process of loading a map, false when done.
  bool hostHasEM = false; // If this AMRISim is on the same VM as the EM, then shouldReloadMap can simply check the file mod timestamp directly from the source file
  time_t lastMapReloadTime  = 0;
public:
  bool haveMapOriginLLA = false;

  ArLLACoords mapOriginLLA;

public:
  MapLoader(stg_world_t *_world = NULL); 

  void setWorld(stg_world_t *_world) {
    cancelLoad();
    world = _world;
  }

  void setHostHasEM(bool hasEM_arg)
  {
    lastMapReloadTime = 0;
    hostHasEM = hasEM_arg;
  }

  void setMapLoadChunkSizes(size_t numLinesPerChunk, size_t numPointsPerChunk)
  {
    myPolysPerChunk = numLinesPerChunk;
    myPointsPerChunk = numPointsPerChunk;
  }

  //virtual ~MapLoader();
    
  std::string getMapName() { return mapfile; }

  ArLLACoords getMapOriginLLA() { return mapOriginLLA; }

  /** Prepare to load a new map from the given file.
      Clears out any models created from a previous map from
      stage.  Call process() to make progress on actually loading the data and creating models in the stage world.
      If called while still loading another map, that map is closed.  If the given map file is already loaded (i.e. is not
      a new name, or file has not changed since last load), then loading is considered finished (and @a callback is invoked to
      indicate this).
      @param callback If not NULL, call this callback when map is done loading.
      @param errormsg if not NULL, place any error messages in this string
      @return true if ready to load, false on error (error opening file or preparing to read)
    */
  bool newMap(const std::string& newmapfile, MapLoadedCallback callback = NULL, std::string *errormsg = NULL);

  bool newMap(std::shared_ptr<ArMap> newmap);

  //std::shared_ptr<ArMap>& getMap() { return map; }
private:
  bool newMap();


public:
  /// Proceed to load data from an open map file, or do nothing if map loading is not in progress.
  /// @param maxTime if not 0, stop loading when this amount of time has passed. Loading will resume on next call to process(). If 0, no time limit.
  bool process(unsigned int maxTime = 0);

  void cancelLoad();

  void addCallback(MapLoadedCallback cb);
  void removeCallback(MapLoadedCallback cb);


  /// keep calling process() until idle
  bool processUntilDone()
  {
    do
    {
      if(!process(20000)) return false;
    } while(myProcessState != NEWMAP_INACTIVE);
    return true;
  }
    

private:
  /// Check whether the given map file name is new or the map file has changed since the last time it was loaded.
  /// @return true if new map file or file changed, otherwise return false.
  bool shouldReloadMap(const std::string& newmapfile);

  struct ObjectClass {
    std::string name;
    bool obstacle = true;
    stg_laser_return_t laser_return = 1;
    bool sonar_return = true;
    ObjectClass() {}
    ObjectClass(const std::string& _name) : name(_name), obstacle(true), laser_return(1), sonar_return(1) {}
  };
  stg_model_t* loadReflector(ArMapObject* cairnObj, stg_model_t* map_model, stg_laser_return_t laser_return);
  stg_model_t* loadBoxObstacle(ArMapObject* cairnObj, stg_model_t* map_model, ObjectClass objclass);

  void invokeMapLoadedCallback(MapLoadedCallback cb, int status, const std::string& filename, const std::shared_ptr<ArMap>& map) const;

  void invokeMapLoadedCallback(MapLoadedCallback cb, int status, const std::string& filename) const;
  //void invokeMapLoadedCallbacks(int status, const std::string& filename, ArMap* map) const; // JPL - experimental
  void reset();

  typedef enum
  {
    NEWMAP_INACTIVE,
    NEWMAP_STARTPROCESS,

    NEWMAP_LOADLINESTART,
    NEWMAP_LOADLINECONT,
    NEWMAP_LOADPOINTSTART,
    NEWMAP_LOADPOINTCONT,
    NEWMAP_LOADCHECK,

    NEWMAP_STAGECREATE,
    NEWMAP_STAGEINITPOLY,
    NEWMAP_STAGEINITPOINT,
    NEWMAP_STAGECLEAR,
    NEWMAP_STAGEINSERT,

    NEWMAP_CUSTTYPESTART,
    NEWMAP_CUSTTYPECONT,

    NEWMAP_CAIRNSTART,
    NEWMAP_CAIRNCONT,

    NEWMAP_RESIZE,
    NEWMAP_CALLBACK,
    NEWMAP_RESET,


  }NewMapProcessState;

  NewMapProcessState myProcessState = NEWMAP_INACTIVE;

  const char *stateName(NewMapProcessState stat);

  // Data needs to be persistent for using non-blocking loading processes in process()
  stg_model_t* myMapModel = NULL;  // Holds only cairn objects (used to hold all map models)
  std::list<stg_model_t*> myModelsToInit;

  // myMapPolysModels is a list of arbitrary-sized groups of lines, so they can be loaded without long blocking executions
  size_t myNumLines = 0;
  size_t myNumPolys = 0;
  std::vector<ArLineSegment>::const_iterator myLine_it;
  size_t myPolysPerChunk = 0;
  std::vector<stg_polygon_t*> myMapPolysChunks;
  size_t myNumMapPolysChunks = 0;
  size_t myCurPolysChunkIdx = 0;
  std::vector<stg_model_t*> myMapPolysModels;

  // myMapPointsModels is a list of arbitrary-sized groups of points, so they can be loaded without long blocking executions
  size_t myNumPoints = 0;
  size_t myPointCount = 0;
  std::vector<ArPose>::const_iterator myPoint_it;
  size_t myPointsPerChunk = 0;
  std::vector<stg_point_t*> myMapPointsChunks;
  size_t myNumMapPointsChunks = 0;
  size_t myCurPointsChunkIdx = 0;
  std::vector<stg_model_t*> myMapPointsModels;

  std::map<std::string, ObjectClass> myObjectClasses;
  std::list<ArArgumentBuilder*>::const_iterator myCusType_it;
  std::list<ArMapObject*>::const_iterator myCairnObj_it;
  bool myLoadedData = false;

  bool processTimeCheck(unsigned int maxTime, ArTime mapProcessStart, NewMapProcessState checkProcessState, bool logPrint = false);

  // FOR DEBUGGING
  //ArTime NewMapLoadTime, NewMapStageTime;

public:
  bool shouldWaitForOthers();

};

#endif
