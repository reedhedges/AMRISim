
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

/* TODO add map loading into a new "World" interface ? */

// enable debug log messages:
//#define DEBUG 1

// to maybe get extensions like sincos():
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#ifndef __USE_GNU
#define __USE_GNU 1
#endif

#include <math.h>
#include <assert.h>
#include "stage.h"
#include "MapLoader.hh"
#include "RobotInterface.hh"
#include "ArMap.h"
#include "ariaUtil.h"


const double MapLoader::ReflectorThickness = 0.0200; // meters (=2cm).



#define PRINT_NUM_POINTS_CREATED 1
#define PRINT_NUM_LINES_CREATED  1

MapLoader::MapLoader(stg_world_t *_world) :
    world(_world), map(NULL), created_map(false), loading(false), hostHasEM(false), haveMapOriginLLA(false),
    myProcessState(MapLoader::NEWMAP_INACTIVE),
    myMapModel(NULL), myNumLines(0), myNumPolys(0), 
    myPolysPerChunk(0), myNumMapPolysChunks(0), myCurPolysChunkIdx(0),
    myNumPoints(0), myPointCount(0), myPointsPerChunk(0), myNumMapPointsChunks(0), myCurPointsChunkIdx(0),
    lastMapReloadTime(0),
    myLoadedData(false)
  { }


MapLoader::~MapLoader()
{
  if(map && created_map)
    delete map;
}


void MapLoader::cancelLoad() {
  reset();
}

void MapLoader::addCallback(MapLoadedCallback cb)
{
  //ArLog::log(ArLog::Normal, "MapLoader::addCallback(): adding callback: %p", (void*)cb);

  callbacks.insert(cb);

  //ArLog::log(ArLog::Normal, "MapLoader::addCallback(): current callbacks:");
  //for (std::set<MapLoadedCallback>::iterator cb_it = callbacks.begin(); cb_it != callbacks.end(); ++cb_it)
  //  ArLog::log(ArLog::Normal, "\t\t%p", *cb_it);

}

void MapLoader::removeCallback(MapLoadedCallback cb)
{
  //ArLog::log(ArLog::Normal, "MapLoader::removeCallback(): removing callback: %p", (void*)cb);

  callbacks.erase(cb);

  //ArLog::log(ArLog::Normal, "MapLoader::addCallback(): current callbacks:");
  //for (std::set<MapLoadedCallback>::iterator cb_it = callbacks.begin(); cb_it != callbacks.end(); ++cb_it)
  //  ArLog::log(ArLog::Normal, "\t\t%p", *cb_it);

}

bool MapLoader::newMap(const std::string& newmapfile, [[maybe_unused]] RobotInterface *requestor, MapLoadedCallback cb, std::string *errorMsg)
{
#ifdef DEBUG
  ArLog::log(ArLog::Normal, "MapLoader::newMap: newmapfile=%s", newmapfile.c_str());
#endif
  if(!shouldReloadMap(newmapfile)) // If the robot has requested to change to the current map...
  {
    //if(requestor) requestor->warn("Not reloading map file \"%s\', it has not changed since last load.", newmapfile.c_str());
    //else print_warning("Not reloading map file \"%s\', it has not changed since last load.", newmapfile.c_str());
    if(myProcessState == NEWMAP_INACTIVE) // If the current map has already been processed...
    {
      //ArLog::log(ArLog::Normal, "MapLoader::newMap(): !shouldReloadMap: myProcessState == NEWMAP_INACTIVE: invoking the requesting robot's callback: %p", (void*)cb);
      invokeMapLoadedCallback(cb, false, newmapfile, NULL); // ... tell the robot the map is loaded, and let its EmulatePioneer load the new map data
    }
    //else
    //{
      //ArLog::log(ArLog::Normal, "MapLoader::newMap(): !shouldReloadMap: myProcessState != NEWMAP_INACTIVE: not invoking the requesting robot's callback");
    //}
    return true;
  }

  /// ??? @todo ? Get points and lines change timestamps from ArMap and check against saved times.
  ///  (Though note that this will create a bug, that modifying reflectors or ?
  //obstacle cairns will be ignored)

  //if (requestor) ArLog::log(ArLog::Normal, "MapLoader::newMap(str, RI, MLCb, str) %s actually loading new map: %s", requestor->getRobotName().c_str(), newmapfile.c_str());
  //else ArLog::log(ArLog::Normal, "MapLoader::newMap(str, RI, MLCb, str) actually loading new map: %s", newmapfile.c_str());

  reset();

  ArTime timer;

  if(!map) {
    map = new ArMap();
    created_map = true;
  }

  char errbuf[128];

  struct stat path_st;
  struct stat file_st;
  char fileName[256];
  char curPath[256];
  char copyPath[256];

  // XXX TODO remove map copying, check for EM, etc. from here 

  strncpy(curPath, newmapfile.c_str(), 255);
  curPath[255] = '\0';
  ArUtil::getDirectory(newmapfile.c_str(), curPath, 256);
  strncpy(copyPath, curPath, 255);
  copyPath[255] = '\0';
  const size_t len = strlen(copyPath);
  if (hostHasEM)
    strncat(copyPath, "./sim/", 255-len);
  else
    strncat(copyPath, "/copyMap/", 255-len);
  copyPath[255 - len] = '\0';

  if(stat(newmapfile.c_str(), &file_st) == -1 || stat(copyPath, &path_st) == -1)
  {
    stg_print_msg("Could either not find the map or not find the copyDir. Processing in-place.");

    if(!map->readFile(newmapfile.c_str(), errbuf, 127))
    {
      if(errorMsg) *errorMsg = errbuf;
      return false;
    }
    //print_debug("Took %d msec to read map file", timer.mSecSince());
    //ArLog::log(ArLog::Normal, "MapLoader::newMap(str, RI, MLCb, str): using requestor (%p) and sending cb (%p) to mapLoader.newMap(MLCb)", (void*)requestor, (void*)cb);
    return newMap(cb);
  }
  else
  {
    stg_print_msg("Found the map and the copyDir. Copying to new location: %s", copyPath);
    char cpCommand[264];
    const auto r = snprintf(cpCommand, 263, "cp \"%s\" \"%s\"", newmapfile.c_str(), copyPath);
    assert(r < 263);
    stg_print_msg("Copy command: %s", cpCommand);
    const int sr = system(cpCommand);
    assert(sr == 0);

    ArUtil::getFileName(newmapfile.c_str(), fileName, 256);

    std::string newname;
    newname.clear();
    newname += copyPath;
    newname += fileName;

    stg_print_msg("Resetting the newname: %s", newname.c_str());
    if(!map->readFile(newname.c_str(), errbuf, 127))
    {
      stg_print_msg("Failed to read map file: %s", newname.c_str());
      if(errorMsg) *errorMsg = errbuf;
      return false;
    }
    //print_debug("Took %d msec to read map file", timer.mSecSince());
    //ArLog::log(ArLog::Normal, "MapLoader::newMap(str, RI, MLCb, str): using requestor (%p) and sending cb (%p) to mapLoader.newMap(MLCb)", (void*)requestor, (void*)cb);
    return newMap(cb);
  }
}

bool MapLoader::newMap(ArMap *newmap, MapLoadedCallback cb)
{
  //ArLog::log(ArLog::Normal, "MapLoader::newMap(ArMap, MLCb) enter");

  reset();
  map = newmap;
  return newMap(cb);
}

bool MapLoader::newMap(MapLoadedCallback cb)
{
#ifdef DEBUG
  ArLog::log(ArLog::Normal, "MapLoader::newMap(MLCb): cb: %p", (void*)cb);
  //ArLog::log(ArLog::Normal, "MapLoader::newMap(MLCb): map: %p", (void*)map);
#endif


  // Prepare class member variables for use from MapLoader::process()
  //callback = cb;  // obsolete // TODO: remove cb from all function argument lists
  mapfile = map->getFileName();
  myLoadedData = false;
  myProcessState = MapLoader::NEWMAP_STARTPROCESS;
  loading = true;

  // XXX should we call callback?

    // TODO build a new matrix at the right resolution and size and use it to
    // filter out redundant points for that resolution (avoid too much data in
    // stg_point_t array for instance) as we get
    // data from ArMap, then add robots from old matrix, then swap in new
    // matrix. This should be doable over several process() calls, and avoid
    // having to resize the world.


  return true;
}

#ifdef DEBUG
#define DEBUG_LOG_NEW_STATE() print_debug("MapLoader::process(): Set new state, is now %s (%d)", stateName(myProcessState), myProcessState);
#define DEBUG_LOG_STATE_ACTION() print_debug("MapLoader::process(): Doing %s (%d)...", stateName(myProcessState), myProcessState);
#else
#define DEBUG_LOG_NEW_STATE() {}
#define DEBUG_LOG_STATE_ACTION() {}
#endif

bool MapLoader::process(unsigned int maxTime)
{

#define SINGLE_PROCESS_LINE     1 // if this is '1', the loop will check remeaining time before processing each entry in map->getLines().      else, it will process them all at once
#define SINGLE_PROCESS_POINT    1 // if this is '1', the loop will check remeaining time before processing each entry in map->getPoints().     else, it will process them all at once
#define SINGLE_PROCESS_CUSTTYPE 1 // if this is '1', the loop will check remeaining time before processing each entry in map->getMapInfo().    else, it will process them all at once
#define SINGLE_PROCESS_CAIRNOBJ 1 // if this is '1', the loop will check remeaining time before processing each entry in map->getMapObjects(). else, it will process them all at once
  
#ifdef DEBUG
  print_debug("MapLoader::process() called (maxTime=%u).  State is %s (%d).  SINGLE_PROCESS_LINE? %d SINGLE_PROCESS_POINT? %d SINGLE_PROCESS_CUSTTYPE? %d SINGLE_PROCESS_CAIRNOBJ?  %d", maxTime, stateName(myProcessState), myProcessState, SINGLE_PROCESS_LINE, SINGLE_PROCESS_POINT, SINGLE_PROCESS_CUSTTYPE, SINGLE_PROCESS_CAIRNOBJ);
#endif

  // There is no pending newMap processing to service
  if (myProcessState == MapLoader::NEWMAP_INACTIVE)
  {
    return true;
  }

  // There's no time to process
  if (maxTime == 0)
    return true;

  // There is no map object
#ifdef DEBUG
  ArLog::log(ArLog::Normal, "MapLoader::process(): map: %p", (void*)map);
#endif
  if (map == NULL)
    return false;


  // This is a dummy state that performs no action, and returns to the main loop.
  // This is necessary because the map change procedure is called asynchronously
  //   and calls ArMap::readFile(), blocking other things and taking up lots of time.
  // After this happens, we want to return to the main loop processes before beginning
  //   work here.
  if (myProcessState == MapLoader::NEWMAP_STARTPROCESS)
  {
    myProcessState = MapLoader::NEWMAP_LOADLINESTART;
    DEBUG_LOG_NEW_STATE();
    return true;
  }

  ArTime mapProcessStart;

  // Load the lines from the new map file
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADLINESTART))
    return true;
  if (myProcessState == MapLoader::NEWMAP_LOADLINESTART)
  {
    DEBUG_LOG_STATE_ACTION();

    //NewMapLoadTime.setToNow();
    myNumLines = map->getLines()->size();

    if (myNumLines > 0)
    {
      //myPolysPerChunk      =  10000;   // TODO: Hardcoded for now. Create AMRISim commandline parameter to overwrite this value
      myNumMapPolysChunks  = myNumLines / myPolysPerChunk;
      if (myNumLines % myPolysPerChunk != 0) ++myNumMapPolysChunks;
      myMapPolysChunks.clear();
      myMapPolysChunks.resize(myNumMapPolysChunks);

      for (size_t i = 0; i < myNumMapPolysChunks; ++i)
      {
        if(i < myNumMapPolysChunks-1 || myNumLines % myPolysPerChunk == 0)
          myMapPolysChunks[i] = stg_polygons_create((int) myPolysPerChunk);
        else
          myMapPolysChunks[i] = stg_polygons_create((int) (myNumLines % myPolysPerChunk));
      }
      myLine_it = map->getLines()->begin();
      myNumPolys = 0;

      myProcessState = MapLoader::NEWMAP_LOADLINECONT;
      DEBUG_LOG_NEW_STATE()
    }
    else
    {
      myNumMapPolysChunks = 0;

      // If there are no lines to process, just skip to the next state
      myProcessState = MapLoader::NEWMAP_LOADPOINTSTART;
      DEBUG_LOG_NEW_STATE()
    }
    ArLog::log(ArLog::Normal, "MapLoader::process(): myNumLines: %lu, myPolysPerChunk: %lu, myNumMapPolysChunks: %lu", myNumLines, myPolysPerChunk,  myNumMapPolysChunks);

    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADLINESTART section finished at %d msec.", NewMapLoadTime.mSecSince());
  }

  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADLINECONT))
    return true;
  if (myProcessState == MapLoader::NEWMAP_LOADLINECONT)
  {
    DEBUG_LOG_STATE_ACTION();
    ArTime timer;
    if(myNumLines == 0)
    {
      myProcessState = MapLoader::NEWMAP_LOADPOINTSTART;
      DEBUG_LOG_NEW_STATE()
      myLoadedData = true;
      return true;
    }

    for(; myLine_it != map->getLines()->end(); ++myLine_it)
    {
#if SINGLE_PROCESS_LINE
      // Make sure we have time left for one more iteration
      if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADLINECONT))
      {
        //ArLog::log(ArLog::Normal, "AMRISim map loader: exiting this process loop at %lu lines...", myNumPolys);
        //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADLINECONT section took %d msec.", timer.mSecSince());
        return true;
      }
#endif

      size_t curPolysChunkIdx = (myNumPolys / myPolysPerChunk);
      stg_polygon_t* curPolysChunk = myMapPolysChunks[curPolysChunkIdx];
      size_t curPolysSubIdx = (myNumPolys % myPolysPerChunk);

      // record this line. convert mm to m
      stg_point_t v1 = {(*myLine_it).getX1() / 1000.0, (*myLine_it).getY1() / 1000.0};
      stg_point_t v2 = {(*myLine_it).getX2() / 1000.0, (*myLine_it).getY2() / 1000.0};
      stg_polygon_append_points(&curPolysChunk[curPolysSubIdx], &v1, 1);
      stg_polygon_append_points(&curPolysChunk[curPolysSubIdx], &v2, 1);


#if PRINT_NUM_LINES_CREATED
      if(myNumPolys % myPolysPerChunk == 0)
        ArLog::log(ArLog::Normal, "AMRISim map loader: At %lu lines...", myNumPolys);
#endif
      ++myNumPolys;
    }

    // If the process has reached the end of the list, move on to the next state
    myLoadedData = true;
    myProcessState = MapLoader::NEWMAP_LOADPOINTSTART;
    DEBUG_LOG_NEW_STATE()
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADLINECONT section finished at %d msec.", NewMapLoadTime.mSecSince());
  }


  // Load the points from the new map file
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADPOINTSTART))
    return true;
  if (myProcessState == MapLoader::NEWMAP_LOADPOINTSTART)
  {
    DEBUG_LOG_STATE_ACTION();
    myNumPoints = map->getPoints()->size();

    if(myNumPoints > 0)
    {
      //myPointsPerChunk     = 100000;  // TODO: Hardcoded for now. Create AMRISim commandline parameter to overwrite this value
      myNumMapPointsChunks  = myNumPoints / myPointsPerChunk;
      if (myNumPoints % myPointsPerChunk != 0) ++myNumMapPointsChunks;
      myMapPointsChunks.clear();
      myMapPointsChunks.resize(myNumMapPointsChunks);
      for (size_t i = 0; i < myNumMapPointsChunks; ++i)
      {
#ifdef DEBUG
        print_debug("MapLoader::process(): Allocating chunk %d of %d...", i, myNumMapPointsChunks);
#endif
        if(i < myNumMapPointsChunks-1 || myNumPoints % myPointsPerChunk == 0)
        {
#ifdef DEBUG
        print_debug("MapLoader::process(): stg_points_create(myPolysPerChunk %d)...", myPolysPerChunk);
#endif
          myMapPointsChunks[i] = stg_points_create(myPolysPerChunk);
        }
        else
        {
#ifdef DEBUG
        print_debug("MapLoader::process(): stg_points_create(myNumPoints %d %% myPointsPerChunk %d == %d)", myNumPoints, myPointsPerChunk, myNumPoints % myPointsPerChunk);
#endif
          //myMapPointsChunks[i] = stg_points_create(myNumLines % myPointsPerChunk);
          myMapPointsChunks[i] = stg_points_create(myNumPoints % myPointsPerChunk);
        }
      }
      myPoint_it = map->getPoints()->begin();
      myPointCount = 0;

      myProcessState = MapLoader::NEWMAP_LOADPOINTCONT;
      DEBUG_LOG_NEW_STATE()
    }
    else
    {
      myNumMapPointsChunks = 0;

      // If there are no points to process, just skip to the next state
      myProcessState = MapLoader::NEWMAP_LOADCHECK;
      DEBUG_LOG_NEW_STATE()
    }
    ArLog::log(ArLog::Normal, "MapLoader::process(): myNumPoints: %lu, myPointsPerChunk: %lu, myNumMapPointsChunks: %lu", myNumPoints, myPointsPerChunk,  myNumMapPointsChunks);

    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADPOINTSTART section finished at %d msec.", NewMapLoadTime.mSecSince());
  }

  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADPOINTCONT))
    return true;
  if (myProcessState == MapLoader::NEWMAP_LOADPOINTCONT)
  {
    DEBUG_LOG_STATE_ACTION();
    if(myNumLines == 0)
    {
      myProcessState = MapLoader::NEWMAP_LOADCHECK;
      DEBUG_LOG_NEW_STATE()
      myLoadedData = true;
      return true;
    }

    for(; myPoint_it != map->getPoints()->end(); ++myPoint_it)
    {
#if SINGLE_PROCESS_POINT
      // Make sure we have time left for one more iteration
      if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADPOINTCONT))
        return true;
#endif
      size_t curPointsChunkIdx = (myPointCount / myPointsPerChunk);
      stg_point_t* curPointsChunk = myMapPointsChunks[curPointsChunkIdx];
      size_t curPointsSubIdx = (myPointCount % myPointsPerChunk);
      curPointsChunk[curPointsSubIdx].x = (stg_meters_t) ( myPoint_it->getX() / 1000.0 );     // convert mm to m
      curPointsChunk[curPointsSubIdx].y = (stg_meters_t) ( myPoint_it->getY() / 1000.0 );

#if PRINT_NUM_POINTS_CREATED
      if(myPointCount % myPointsPerChunk == 0)
        ArLog::log(ArLog::Normal, "AMRISim map loader: At %lu points...", myPointCount);
#endif
      ++myPointCount;
    }

    // If the process has reached the end of the list, move on to the next state
    myLoadedData = true;
    myProcessState = MapLoader::NEWMAP_LOADCHECK;
    DEBUG_LOG_NEW_STATE()
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADPOINTCONT section finished at %d msec.", NewMapLoadTime.mSecSince());
  }


  // Check to make sure something has been loaded from the new map file
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_LOADCHECK))
    return true;
  if (myProcessState == MapLoader::NEWMAP_LOADCHECK)
  {
    DEBUG_LOG_STATE_ACTION();
    if(!myLoadedData)
    {
      stg_print_warning("AMRISim: No obstacle data loaded from map file \"%s\"!", mapfile.c_str());
      //if(!loadPoints) stg_print_warning("AMRISim: Requested not to load point data, try enabling.");
      //if(!loadLines) stg_print_warning("AMRISim: Requested not to load line data, try enabling.");
    }

    // Load origin georeference
    haveMapOriginLLA = map->hasOriginLatLongAlt();
    if(haveMapOriginLLA)
    {
      mapOriginLLA.setX(map->getOriginLatLong().getX());
      mapOriginLLA.setY(map->getOriginLatLong().getY());
      mapOriginLLA.setZ(map->getOriginAltitude());
      stg_print_msg("AMRISim: Map has OriginLatLon point, will be able to send simulated GPS coordinates if requested.");
    }

    myProcessState = MapLoader::NEWMAP_STAGECREATE;
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOADCHECK section finished at %d msec.", NewMapLoadTime.mSecSince());
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_LOAD* sections took %d msec.", NewMapLoadTime.mSecSince());
  }

  // Create the Stage model from themap file (TODO: probably safe to move NEWMAP_STAGECLEAR below this, in pursuit of making NEWMAP_STAGECLEAR/NEWMAP_STAGEINSERT atomic)
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_STAGECREATE))
    return true;
  if (myProcessState == MapLoader::NEWMAP_STAGECREATE)
  {
    DEBUG_LOG_STATE_ACTION();
    //NewMapStageTime.setToNow();
    // find a unique id number
    stg_id_t id = 0;
    for(id = 0; stg_world_get_model(world, id) != NULL && id <= STG_ID_T_MAX; ++id)
      ;
    if(id == STG_ID_T_MAX)
    {
      stg_print_error("AMRISim: !!! too many models in the world, can't create a new one.");
      return false;
    }

    // create model TODO use stg_world_new_model instead?
    // set as a background figure, so it isn't redrawn every time the robot moves
    // or whatever.
    myMapModel = stg_model_create(world, NULL, id, mapfile.c_str(), "model", "model", 0, NULL, TRUE);
    myModelsToInit.push_back(myMapModel);

    myMapPolysModels.clear();
    myMapPolysModels.resize(myNumMapPolysChunks);
    for(size_t i = 0; i < myNumMapPolysChunks; ++i)
    {
      myMapPolysModels[i] = stg_model_create(world, NULL, id, mapfile.c_str(), "model", "model", 0, NULL, TRUE);
      myModelsToInit.push_back(myMapPolysModels[i]);
    }
    myCurPolysChunkIdx = 0; // This prepares NEWMAP_STAGEINITPOLY to start at the first polys chunk

    myMapPointsModels.clear();
    myMapPointsModels.resize(myNumMapPointsChunks);
    for(size_t i = 0; i < myNumMapPointsChunks; ++i)
    {
      myMapPointsModels[i] = stg_model_create(world, NULL, id, mapfile.c_str(), "model", "model", 0, NULL, TRUE);
      myModelsToInit.push_back(myMapPointsModels[i]);
    }
    myCurPointsChunkIdx = 0; // This prepares NEWMAP_STAGEINITPOLY to start at the first points chunk

    // get the color
    stg_color_t mapcolor = stg_lookup_color("dark gray");
    if(mapcolor == 0xFF0000) mapcolor = 0; // black if not found

    // get size
    double maxX_mm = 0;
    double minX_mm = 0;
    double maxY_mm = 0;
    double minY_mm = 0;
    if(myNumPoints > 0 && myNumLines > 0)
    {
      maxX_mm = fmax(map->getLineMaxPose().getX(), map->getMaxPose().getX());
      minX_mm = fmin(map->getLineMinPose().getX(), map->getMinPose().getX());
      maxY_mm = fmax(map->getLineMaxPose().getY(), map->getMaxPose().getY());
      minY_mm = fmin(map->getLineMinPose().getY(), map->getMinPose().getY());
    }
    else if(myNumPoints > 0)
    {
      maxX_mm = map->getMaxPose().getX();
      minX_mm = map->getMinPose().getX();
      maxY_mm = map->getMaxPose().getY();
      minY_mm = map->getMinPose().getY();
    }
    else if(myNumLines > 0)
    {
      maxX_mm = map->getLineMaxPose().getX();
      minX_mm = map->getLineMinPose().getX();
      maxY_mm = map->getLineMaxPose().getY();
      minY_mm = map->getLineMinPose().getY();
    }
    stg_size_t size;
    size.x = maxX_mm/1000.0 - minX_mm/1000.0; //mm to m
    size.y = maxY_mm/1000.0 - minY_mm/1000.0; //mm to m
    stg_print_msg("New world from loading map \"%s\" will be %f x %f meters in size.", mapfile.c_str(), size.x, size.y);

    // get origin offset (obsolete?)
    //stg_pose_t offset;
    //offset.x = (size.x / 2.0) + (minX_mm / 1000.0);
    //offset.y = (size.y / 2.0) + (minY_mm / 1000.0);
    //offset.a = 0;

    int grid = 0; // Turn off grid
    int movemask = 0; // Make it unmovable
    time_t tm = time(NULL);// store creation time and file source


    // Give the basic map initialization to each model
    int model_num = 0;
    for (std::list<stg_model_t*>::iterator mod_it = myModelsToInit.begin(); mod_it != myModelsToInit.end(); ++mod_it)
    {
#ifdef DEBUG
      ArLog::log(ArLog::Normal, "Initializing model_num: %d", model_num);
#endif
      ++model_num;

      stg_model_t *cur_mod = (*mod_it);

#ifdef DEBUG
      ArLog::log(ArLog::Normal, "THIS IS THE POINTER: %u", cur_mod);
#endif

      // store the map file name
      stg_model_set_property(cur_mod, "source", (void*)mapfile.c_str(), mapfile.size()+1);

      // set color
      stg_model_set_property(cur_mod, "color", &mapcolor, sizeof(mapcolor));

      // set size
      stg_model_set_size(cur_mod, size);

      // but don't scale it to that size, lines and points are already at the right
      // places for correct scale
      stg_model_set_scaling(cur_mod, FALSE);

      // set origin offset (obsolete?)
      //stg_model_set_origin(cur_mod, offset);

      stg_model_set_property(cur_mod, "grid", &grid, sizeof(int));
      stg_model_set_property(cur_mod, "mask", &movemask, sizeof(int));
      //stg_model_set_property(cur_mod, "source", (void*)mapfile.c_str(), mapfile.size()+1); // redundant
      stg_model_set_property(cur_mod, "creation_time", &tm, sizeof(tm));
    }

    myProcessState = MapLoader::NEWMAP_STAGEINITPOLY;
    DEBUG_LOG_NEW_STATE()
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGECREATE section finished at %d msec.", NewMapLoadTime.mSecSince());
  }

  // Insert polys into the Stage world (TODO: probably safe to move NEWMAP_STAGECLEAR below this, in pursuit of making NEWMAP_STAGECLEAR/NEWMAP_STAGEINSERT atomic)
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_STAGEINITPOLY))
    return true;
  if (myProcessState == MapLoader::NEWMAP_STAGEINITPOLY)
  {
    DEBUG_LOG_STATE_ACTION();
#ifdef DEBUG
    ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGEINITPOLY: myNumMapPolysChunks: %lu", myNumMapPolysChunks);
#endif
    ArTime timer;
    if(myNumMapPolysChunks > 0)
    {
      timer.setToNow();

      //ArLog::log(ArLog::Normal, "MapLoader::process(): calling stg_model_set_polygons(): myNumPolys: %lu", myNumPolys);
      stg_model_t *curPolysModel = myMapPolysModels[myCurPolysChunkIdx];
      stg_polygon_t *curPolysChunk = myMapPolysChunks[myCurPolysChunkIdx];
      if(myCurPolysChunkIdx < myNumMapPolysChunks-1 || myNumLines % myPolysPerChunk == 0)
      {
#ifdef DEBUG
        ArLog::log(ArLog::Normal, "MapLoader::process(): setting polygons for chunk %d, containing %lu polys", myCurPolysChunkIdx, myPolysPerChunk);
#endif
        stg_model_set_polygons(curPolysModel, curPolysChunk, myPolysPerChunk); 
      }
      else
      {
#ifdef DEBUG
        ArLog::log(ArLog::Normal, "MappLoader::process(): setting polygons for chunk %d, containing %lu polys", myCurPolysChunkIdx, myNumLines % myPolysPerChunk);
#endif
        stg_model_set_polygons(curPolysModel, curPolysChunk, myNumLines % myPolysPerChunk);
      }
#ifdef DEBUG
      print_debug("Took %d msec to store model polygons in myMapPolysModel.", timer.mSecSince());
#endif

      // TODO: Why is this call free(myMapPolys), while the call below is stg_points_destroy(myMapPoints) ?
      free(curPolysChunk);  // it was copied by stg_model_init_polygons

      ++myCurPolysChunkIdx;
    }


    //ArLog::log(ArLog::Normal, "MapLoader::process(): myCurPolysChunkIdx: %d, myNumMapPolysChunks: %lu", myCurPolysChunkIdx, myNumMapPolysChunks);
    if(myCurPolysChunkIdx >= myNumMapPolysChunks)
    {
      //ArLog::log(ArLog::Normal, "Continuing to NEWMAP_STAGEINITPOINT");
      myProcessState = MapLoader::NEWMAP_STAGEINITPOINT;
      DEBUG_LOG_NEW_STATE()
    }
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGEINITPOLY section finished at %d msec.", NewMapStageTime.mSecSince());
  }

  // Insert polys into the Stage world
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_STAGEINITPOINT))
    return true;
  if (myProcessState == MapLoader::NEWMAP_STAGEINITPOINT)
  {
    DEBUG_LOG_STATE_ACTION();
#ifdef DEBUG
    print_debug("MapLoader::process(): myNumMapPointsChunks: %lu", myNumMapPointsChunks);
#endif
    ArTime timer;
    if(myNumMapPointsChunks > 0)
    {
      timer.setToNow();

#ifdef DEBUG
      print_debug("MapLoader::process(): calling stg_model_init_points(): myNumPolys: %lu", myNumPoints);
#endif
      stg_model_t *curPointsModel = myMapPointsModels[myCurPointsChunkIdx];
      stg_point_t *curPointsChunk = myMapPointsChunks[myCurPointsChunkIdx];

#ifdef DEBUG
      print_debug("MapLoader::process(): calling stg_model_init_points(): myNumPolys: %lu", myNumPoints);
#endif
      if(myCurPointsChunkIdx < myNumMapPointsChunks-1 || myNumPoints % myPointsPerChunk == 0)
      {
#ifdef DEBUG
        print_debug("MapLoader::process(): setting points for chunk %d, containing %lu points (from myPointsPerChunk)", myCurPointsChunkIdx, myPointsPerChunk);
#endif
        stg_model_init_points(curPointsModel, curPointsChunk, myPointsPerChunk);
      }
      else
      {
#ifdef DEBUG
        print_debug("MapLoader::process(): setting points for chunk %d, containing %lu points (myNumPoints %d %% myPointsPerChunk %d)", myCurPointsChunkIdx, myNumPoints % myPointsPerChunk, myNumPoints, myPointsPerChunk);
#endif
        stg_model_init_points(curPointsModel, curPointsChunk, myNumPoints % myPointsPerChunk);
      }
      //print_debug("Took %d msec to store model points in myMapPointsModel.", timer.mSecSince());

      // TODO: Why is this call stg_points_destroy(myMapPoints), while the call above is free(myMapPolys) ?
      stg_points_destroy(curPointsChunk); // it was copied by stg_model_init_points

      ++myCurPointsChunkIdx;
    }


    //ArLog::log(ArLog::Normal, "MapLoader::process(): myCurPointsChunkIdx: %d, myNumMapPointsChunks: %lu", myCurPointsChunkIdx, myNumMapPointsChunks);
    if(myCurPointsChunkIdx >= myNumMapPointsChunks)
    {
      //ArLog::log(ArLog::Normal, "Continuing to NEWMAP_STAGECLEAR");
      myProcessState = MapLoader::NEWMAP_STAGECLEAR;
      DEBUG_LOG_NEW_STATE()
    }
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGEINITPOINT section finished at %d msec.", NewMapStageTime.mSecSince());
  }


  // Clear everything from the Stage world (leave this atomic unless absolutely necessary) // TODO: moving this downward toward NEWMAP_STAGEINSERT
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_STAGECLEAR))
    return true;
  if (myProcessState == MapLoader::NEWMAP_STAGECLEAR)
  {
    DEBUG_LOG_STATE_ACTION();
    // Clear any existing map models
    for(std::set<stg_model_t*>::const_iterator i = mapModels.begin(); i != mapModels.end(); ++i)
    {
      assert(*i);
      stg_world_remove_model(world, *i);
      //printf("clearMap: destroying model \"%s\"...\n", stg_model_get_token(*i));
      stg_model_destroy(*i);
    }
    mapModels.clear();

    myProcessState = MapLoader::NEWMAP_STAGEINSERT;
    DEBUG_LOG_NEW_STATE()
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGECLEAR section finished at %d msec.", NewMapStageTime.mSecSince());
  }


  // Insert polys into the Stage world
  //if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_STAGEINSERT)) // TODO: This check was removed to make the NEWMAP_STAGECLEAR/INSERT atomic. Pretty sure it won't cause problems, but leaving the line in case it needs to be reimplemented
  //  return true;
  if (myProcessState == MapLoader::NEWMAP_STAGEINSERT)
  {
    DEBUG_LOG_STATE_ACTION();

    // Remember the cairn objects model
    mapModels.insert(myMapModel); // Still contains Cairn objects


    // Remember the polygons models
    for(size_t i = 0; i < myNumMapPolysChunks; ++i)
    {
      mapModels.insert(myMapPolysModels[i]);            // Remember this model locally
      stg_world_add_model(world, myMapPolysModels[i]);  // Add model to stage world. Just an entry in the hash table.
    }

    // Remember the points models
    for(size_t i = 0; i < myNumMapPointsChunks; ++i)
    {
      mapModels.insert(myMapPointsModels[i]);            // Remember this model locally
      stg_world_add_model(world, myMapPointsModels[i]);  // Add model to stage world. Just an entry in the hash table.
    }


    myProcessState = MapLoader::NEWMAP_CUSTTYPESTART;
    DEBUG_LOG_NEW_STATE();
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGEINSERT section finished at %d msec.", NewMapStageTime.mSecSince());
    //ArLog::log(ArLog::Normal, "MapLoader::process(): NEWMAP_STAGE* sections took %d msec.", NewMapLoadTime.mSecSince());
  }


  // Process the Custom type definitions
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CUSTTYPESTART))
    return true;
  if (myProcessState == MapLoader::NEWMAP_CUSTTYPESTART)
  {
    DEBUG_LOG_STATE_ACTION();
    // Check special simulator attributes of custom map object type definitions,
    // and create models for objects as neccesary.
    // TODO check Color0 and Color1, SonarReflect.

    // Built in reflector type always has a high laser retun value by default
    ObjectClass reflector_class("Reflector");
    reflector_class.laser_return = 2;
    myObjectClasses["Reflector"] = reflector_class;

    myCusType_it = map->getMapInfo()->begin();

    myProcessState = MapLoader::NEWMAP_CUSTTYPECONT;
    DEBUG_LOG_NEW_STATE();
  }

  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CUSTTYPECONT))
    return true;
  if (myProcessState == MapLoader::NEWMAP_CUSTTYPECONT)
  {
    DEBUG_LOG_STATE_ACTION();
    for(; myCusType_it != map->getMapInfo()->end(); ++myCusType_it)
    {
#if SINGLE_PROCESS_CUSTTYPE
      if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CUSTTYPECONT))
        return true;
#endif

      const char *type_name = (*myCusType_it)->getArg(1);
      if( strncmp(type_name, "Name=", 5) != 0 )
      {
        //stg_print_warning("AMRISim: First MapInfo attribute is not \"Name\", skipping.");
        continue;
      }
      type_name += strlen("Name="); // skip past the "Name=" prefix

      ObjectClass new_class(type_name);

      const char *shape = (*myCusType_it)->getArg(0);
      // XXX BoundaryType not implemented yet
      if(strcmp(shape, "SectorType") != 0)
        continue;

      // Reflectors can be built-in reflectors, or have a name that
      // previous versions of AMRISim interpreted as automatically being
      // reflectors.
      //int val = 1;
      const char* endTag = strrchr(type_name, '.');
      if(   strcmp( type_name, "Sim.Reflector") == 0
        || strcmp(  type_name, "Reflector") == 0
        || (endTag && strcmp(endTag, ".Reflect") == 0)
      )
      {
        new_class.laser_return = 2;
      }

      // Check remaining attributes for special simulator things
      for(size_t a = 2; a < (*myCusType_it)->getArgc(); ++a)
      {
        char buf[256];
        ArUtil::stripQuotes(buf, (*myCusType_it)->getArg(a), 256);

        // Reflective to laser?
        new_class.laser_return = 0;
        if(strncmp(buf, "Sim.LaserReflect=", strlen("Sim.LaserReflect=")) == 0)
        {
          if(strncmp(buf, "Sim.LaserReflect=no", strlen("Sim.LaserReflect=no")) == 0 || strncmp(buf, "Sim.LaserReflect=false", strlen("Sim.LaserReflect=false")) == 0)
            new_class.laser_return = 0;
          else
            new_class.laser_return = atoi( buf + strlen("Sim.LaserReflect=") ) + 1;   // Need to add one because Stage starts highly reflective objects at 2, but SICK/Aria at 1
          stg_print_msg("AMRISim: Will use reflection value %d for objects with type %s (from Sim.LaserReflect attribute in MapInfo)", new_class.laser_return, type_name);
        }

        // To sonar?
        if(strncmp(buf, "Sim.SonarReflect=no", strlen("Sim.SonarReflect=no")) == 0 || strncmp(buf, "Sim.SonarReflect=false", strlen("Sim.SonarReflect=false")) == 0)
        {
          new_class.sonar_return = false;
          stg_print_msg("AMRISim: Objects of type %s %s be visible to sonar (from Sim.SonarReflect attribute in MapInfo)", type_name, new_class.sonar_return?"will":"will not");
        }


        new_class.obstacle = false;

        // Obstacle to robot?
        if(strcasecmp(buf, "Sim.Obstacle=yes") == 0 || strcasecmp(buf, "Sim.Obstacle=true") == 0)
        {
          stg_print_msg("AMRISim: Objects of type %s will be represented as obstacles (from Sim.Obstacle attribute in MapInfo for %s)", type_name, type_name);
          new_class.obstacle = true;
        }

        // Non-obstacle to robot?
        if(strcasecmp(buf, "Sim.Obstacle=no") == 0 || strcasecmp(buf, "Sim.Obstacle=false") == 0)
        {
          stg_print_msg("AMRISim: Objects of type %s will be represented as non-obstacle objects (from Sim.Obstacle attribute in MapInfo for %s)", type_name, type_name);
          new_class.obstacle = false;
        }
      }

      myObjectClasses[type_name] = new_class;
    }

    // If the process has reached the end of the list, move on to the next state
    myProcessState = MapLoader::NEWMAP_CAIRNSTART;
    DEBUG_LOG_NEW_STATE();
  }


  // Process the Cairn objects
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CAIRNSTART))
    return true;
  if (myProcessState == MapLoader::NEWMAP_CAIRNSTART)
  {
    DEBUG_LOG_STATE_ACTION();
    myCairnObj_it = map->getMapObjects()->begin();
    myProcessState = MapLoader::NEWMAP_CAIRNCONT;
    DEBUG_LOG_NEW_STATE();
  }

  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CAIRNCONT))
    return true;
  if (myProcessState == MapLoader::NEWMAP_CAIRNCONT)
  {
    DEBUG_LOG_STATE_ACTION();
    // Create special objects for certain Cairn objects.
    // Reflector: make a line in the map with bright reflectance.
    // Sim.BoxObstacle: Make a box that the user can move.
    for(; myCairnObj_it != map->getMapObjects()->end(); ++myCairnObj_it)
    {
#if SINGLE_PROCESS_CAIRNOBJ
      if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CAIRNCONT))
        return true;
#endif

      ArMapObject* obj = (*myCairnObj_it);
      if(obj == NULL) continue;

      stg_model_t* model = NULL;

      //bool builtinReflector = false;

      // XXX this needs to be refactored a bit, probably eliminate the seperate
      // loadReflector and LoadBoxObstacle functions, just set unique properties
      // seperately.

      std::map<std::string, ObjectClass>::iterator c = myObjectClasses.find(obj->getType());
      if(c != myObjectClasses.end())
      {
        // Built-in Reflector objects have special fixed properties (color, shape, etc.)
        if(strcmp(obj->getType(), "Reflector") == 0 || c->second.laser_return > 1)
        {
          //builtinReflector = true;
          model = loadReflector(obj, myMapModel, c->second.laser_return);
        }
        // Is the object of a type (class) that should be an obstacle?
        else if (c->second.obstacle)
        {
          model = loadBoxObstacle(obj, myMapModel, c->second);
        }
        // Otherwise, ignore it.
      }

      // TODO support line-shaped obstacles, and reflective thngs that aren't also
      // obstacles (they're phantom reflectors)

      if(model == NULL) continue;  // no simulator obstacle was created for this map object

      // store file it was loaded from and current time
      stg_model_set_property(model, "source", (void*)mapfile.c_str(), mapfile.size()+1);
      time_t t = time(NULL);
      stg_model_set_property(model, "creation_time", &t, sizeof(t));

      mapModels.insert(model);
    }

    // If the process has reached the end of the list, move on to the next state
    myProcessState = MapLoader::NEWMAP_RESIZE;
    DEBUG_LOG_NEW_STATE();
  }


  // Process the resize
  // Note: this portion will probably always blow the maxTime. The effect is that it will delay the clientOutput/stageUpdate
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_RESIZE))
    return true;
  if (myProcessState == MapLoader::NEWMAP_RESIZE)
  {
    DEBUG_LOG_STATE_ACTION();
    // resize world
    /// @todo Only resize if it got bigger. Also should optimize this, it takes forever.
    stg_world_resize_to_contents(world, 10);

    //stg_world_unlock(world);

    myProcessState = MapLoader::NEWMAP_CALLBACK;
    DEBUG_LOG_NEW_STATE();
  }


  // Process the callback
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_CALLBACK))
    return true;
  if (myProcessState == MapLoader::NEWMAP_CALLBACK)
  {
    DEBUG_LOG_STATE_ACTION();
    //if(callback)
    //  invokeMapLoadedCallback(callback, true, mapfile, map);
    //ArLog::log(ArLog::Normal, "MapLoader::process(): invoking all mapLoaded callbacks");
    if(!callbacks.empty())
    {
      for(std::set<MapLoadedCallback>::iterator cb_it = callbacks.begin(); cb_it != callbacks.end(); ++cb_it)
      {
        //ArLog::log(ArLog::Normal, "MapLoader::process(): invoking callback: %p", (void*)*cb_it);
        invokeMapLoadedCallback(*cb_it, true, mapfile, map);
      }
    }
    myProcessState = MapLoader::NEWMAP_RESET;
    DEBUG_LOG_NEW_STATE();
  }


  // Finish the process by executing the reset()
  if (!processTimeCheck(maxTime, mapProcessStart, MapLoader::NEWMAP_RESET))
    return true;
  if (myProcessState == MapLoader::NEWMAP_RESET)
  {
    DEBUG_LOG_STATE_ACTION();
    reset();
    myProcessState = MapLoader::NEWMAP_INACTIVE;
    DEBUG_LOG_NEW_STATE();
    return true;
  }

  // We've somehow lost our STATE
  //ArLog::log(ArLog::Normal, "MapLoader::process(): Failed to find matching enum label for myProcessState: %d", myProcessState);
  return false;
}

bool MapLoader::processTimeCheck(unsigned int maxTime, ArTime mapProcessStart, NewMapProcessState checkProcessState, bool logPrint)
{
  if (maxTime == 0 || myProcessState > checkProcessState)
    return true;


  // TODO: GET GOOD VALUES FOR THIS
  #define PROCESSWINDOW_LOADLINE     5
  #define PROCESSWINDOW_LOADPOINT    5
  #define PROCESSWINDOW_LOADCHECK    5
  #define PROCESSWINDOW_STGCREATE    5
  #define PROCESSWINDOW_STGGRID      5
  #define PROCESSWINDOW_STGINITPOLY  100 // NOTE: this portion will probably always blow the maxTime, unless the resize is reworked
  #define PROCESSWINDOW_STGINITPOINT 100 // NOTE: this portion will probably always blow the maxTime, unless the resize is reworked
  #define PROCESSWINDOW_STGCLEAR     5
  #define PROCESSWINDOW_STGINSERT    5
  #define PROCESSWINDOW_CUSTTYPE     5
  #define PROCESSWINDOW_CAIRNOBJ     5
  #define PROCESSWINDOW_RESIZE       100 // NOTE: this portion will probably always blow the maxTime, unless the resize is reworked
  #define PROCESSWINDOW_CALLBACK     20
  #define PROCESSWINDOW_RESET        75

  long long processWindow = 0;

  std::string stateDesc = stateName(checkProcessState);

  switch (checkProcessState)
  {
    case MapLoader::NEWMAP_INACTIVE:       stateDesc = "INACTIVE";       processWindow = 0;                          break; // TODO: obsolete, now that this function was created
    case MapLoader::NEWMAP_LOADLINESTART:  stateDesc = "LOADLINESTART";  processWindow = PROCESSWINDOW_LOADLINE;     break;
    case MapLoader::NEWMAP_LOADLINECONT:   stateDesc = "LOADLINECONT";   processWindow = PROCESSWINDOW_LOADLINE;     break;
    case MapLoader::NEWMAP_LOADPOINTSTART: stateDesc = "LOADPOINTSTART"; processWindow = PROCESSWINDOW_LOADPOINT;    break;
    case MapLoader::NEWMAP_LOADPOINTCONT:  stateDesc = "LOADPOINTCONT";  processWindow = PROCESSWINDOW_LOADPOINT;    break;
    case MapLoader::NEWMAP_LOADCHECK:      stateDesc = "LOADCHECK";      processWindow = PROCESSWINDOW_LOADCHECK;    break;
    case MapLoader::NEWMAP_STAGECREATE:    stateDesc = "STAGECREATE";    processWindow = PROCESSWINDOW_STGCREATE;    break;
    //case MapLoader::NEWMAP_STAGEGRID:      stateDesc = "STAGEGRID";      processWindow = PROCESSWINDOW_STGGRID;      break;
    case MapLoader::NEWMAP_STAGEINITPOLY:  stateDesc = "STAGEINITPOLY";  processWindow = PROCESSWINDOW_STGINITPOLY;  break;
    case MapLoader::NEWMAP_STAGEINITPOINT: stateDesc = "STAGEINITPOINT"; processWindow = PROCESSWINDOW_STGINITPOINT; break;
    case MapLoader::NEWMAP_STAGECLEAR:     stateDesc = "STAGECLEAR";     processWindow = PROCESSWINDOW_STGCLEAR;     break;
    case MapLoader::NEWMAP_STAGEINSERT:    stateDesc = "STAGEINSERT";    processWindow = PROCESSWINDOW_STGINSERT;    break;
    case MapLoader::NEWMAP_CUSTTYPESTART:  stateDesc = "CUSTTYPESTART";  processWindow = PROCESSWINDOW_CUSTTYPE;     break;
    case MapLoader::NEWMAP_CUSTTYPECONT:   stateDesc = "CUSTTYPECONT";   processWindow = PROCESSWINDOW_CUSTTYPE;     break;
    case MapLoader::NEWMAP_CAIRNSTART:     stateDesc = "CAIRNSTART";     processWindow = PROCESSWINDOW_CAIRNOBJ;     break;
    case MapLoader::NEWMAP_CAIRNCONT:      stateDesc = "CAIRNCONT";      processWindow = PROCESSWINDOW_CAIRNOBJ;     break;
    case MapLoader::NEWMAP_RESIZE:         stateDesc = "RESIZE";         processWindow = PROCESSWINDOW_RESIZE;       break;
    case MapLoader::NEWMAP_CALLBACK:       stateDesc = "CALLBACK";       processWindow = PROCESSWINDOW_CALLBACK;     break;
    case MapLoader::NEWMAP_RESET:          stateDesc = "RESET";          processWindow = PROCESSWINDOW_RESET;        break;

    default:                               stateDesc = "INACTIVE";       processWindow = 0;                          break;
  }
  const long long timeLeft = (long long) maxTime - mapProcessStart.mSecSinceLL();
  if (logPrint)
    ArLog::log(ArLog::Normal, "\nMapLoader: myProcessState: NEWMAP_%s, timeLeft: %lld", stateDesc.c_str(), timeLeft);

  if (timeLeft == maxTime || timeLeft >= processWindow)
    return true;
  else
  {
    //ArLog::log(ArLog::Normal, "Exiting MapLoader::process(), processWindow is too short for next task");
    return false;
  }
}


bool MapLoader::shouldWaitForOthers()
{
  // This is a special state created so the MapLoader::process()
  //   would wait after ArMap::readFile() for the main loop to
  //   process 'stage simulation' and 'clients output' at least
  //   once, since it probably took a while (like ~400ms).

  // When both of them have processed, the timers will allow the
  //   main loop to call MapLoader::process(), and myProcessState
  //   will change away from NEWMAP_STARTPROCESS, and allow
  //   process() to run.

  if (myProcessState == MapLoader::NEWMAP_STARTPROCESS)
    return true;
  else
    return false;
}


/** Does not lock world */
stg_model_t* MapLoader::loadReflector(ArMapObject* obj, stg_model_t* /*map_model*/, stg_laser_return_t laser_return)
{
  if(!obj->hasFromTo()) 
  {
    stg_print_warning("AMRISim: Found a Reflector Cairn in the map, but the line has no 'from' and 'to' position; skipping.");
    return NULL;
  }


  /*
  stg_print_msg("Found a Reflector in the map file at pose (%fmm,%fmm); a line from (%fmm,%fmm) to (%fmm,%fmm)",
  obj->getPose().getX(), obj->getPose().getY(), 
  obj->getFromPose().getX(), obj->getFromPose().getY(), 
  obj->getToPose().getX(), obj->getToPose().getY());
  */

  // Make a very thin magenta box aligned with the reflector line

  stg_pose_t reflector_pose;




  double line_x1 = (obj->getFromPose().getX() / 1000.0);
  double line_x2 = (obj->getToPose().getX() / 1000.0);
  double line_y1 = (obj->getFromPose().getY() / 1000.0);
  double line_y2 = (obj->getToPose().getY() / 1000.0);
  double line_dx = line_x2 - line_x1;
  double line_dy = line_y2 - line_y1;


  // Turn it into a box with some thickness
  double line_angle = atan( fabs(line_dy) / fabs(line_dx) );
  double reflector_theta = fabs(line_angle); // ?? + reflector_pose.a); ??
  double disp_x = fabs(ReflectorThickness * sin(reflector_theta));
  double disp_y = fabs(ReflectorThickness * cos(reflector_theta));

  // If an object in stage has size (x or y) == 0, then it become
  // infinitely large!
  if(disp_x <= 0.000001) disp_x = 0.000001;
  if(disp_y <= 0.000001) disp_y = 0.000001;

  // Use cairn pose:
  //reflector_pose.x = obj->getPose().getX() / 1000.0;
  //reflector_pose.y = obj->getPose().getY() / 1000.0;
  //reflector_pose.a = obj->getPose().getTh(); // Specifically *don't* use rotation, ARIA doesn't.
  //reflector_pose.a = 0;

  // Alternatively, figure pose from line endpoints:
  reflector_pose.x = line_x1 + (line_dx / 2.0);
  reflector_pose.y = line_y1 + (line_dy / 2.0);
  reflector_pose.a = 0;

  // Find a unique name
  char token[32];
  strncpy(token, "Reflector:0", 31);
  token[31] = '\0';
  size_t i = 0;
  for(; stg_world_model_name_lookup(world, token) != NULL; i++)
  {
    snprintf(token, 32, "Reflector:%lu", i); // try next number
  }

  // find a unique id number
  stg_id_t id = 0;
  for(id = 0; stg_world_get_model(world, id) != NULL; id++)
    ;

  // create reflector model.  Note, not making map a parent, this causes
  // problems currently. TODO use stg_world_new_model instead.
  stg_model_t* reflector_model = stg_model_create(world, NULL, id, token, "model", "model", i, NULL, FALSE);
  assert(reflector_model);


  // set size and position
  stg_size_t reflector_size = { fabs(line_dx) + disp_x, fabs(line_dy) + disp_y };
  stg_model_set_size(reflector_model, reflector_size);
  stg_model_set_pose(reflector_model, reflector_pose);

  // bright laser return
  stg_model_set_property(reflector_model, "laser_return", &laser_return, sizeof(laser_return));

  // color
  stg_color_t color = stg_lookup_color("magenta");
  stg_model_set_property(reflector_model, "color", &color, sizeof(color));

  // shape
  stg_polygon_t* reflector_polys = stg_polygons_create(1);
  stg_point_t v1 = { line_x1, line_y1 };
  stg_point_t v2 = { line_x1 + disp_x, line_y1 - disp_y };
  stg_point_t v3 = { line_x2 + disp_x, line_y2 - disp_y };
  stg_point_t v4 = { line_x2, line_y2 };
  stg_polygon_append_points(&reflector_polys[0], &v1, 1);
  stg_polygon_append_points(&reflector_polys[0], &v2, 1);
  stg_polygon_append_points(&reflector_polys[0], &v3, 1);
  stg_polygon_append_points(&reflector_polys[0], &v4, 1);
  stg_model_init_polygons(reflector_model, reflector_polys, 1);

  int reflector_movemask = 1|2; // movable|rotatable
  int reflector_outline = 0;    // no outline
  stg_model_set_property(reflector_model, "mask", &reflector_movemask, sizeof(int));
  stg_model_set_property(reflector_model, "outline", &reflector_outline, sizeof(int));

  stg_world_add_model(world, reflector_model);
  //printf("loadMap: added reflector model \"%s\" to world.\n", stg_model_get_token(reflector_model));
  return reflector_model;
}

/** Does not lock world */
stg_model_t* MapLoader::loadBoxObstacle(ArMapObject* obj, stg_model_t* /*map_model*/, ObjectClass object_class)
{
  // Todo, look for a MapInfo declaration that defines properties like color for this
  // type of obstacle. (e.g. Sim.SonarOnly, Sim.Invisible, etc.)
  if(!obj->hasFromTo())
  {
    stg_print_warning("AMRISim: Found a Sim.BoxObstacle Cairn in the map, but the line has no 'from' and 'to' position. Skipping.");
    return NULL;
  }

  stg_print_msg("AMRISim: Found a Box Obstacle in the map file at pose (%.0fmm,%.0fmm,%.0fdeg); a box from (%.0fmm,%.0fmm) to (%.0fmm,%.0fmm)",
  obj->getPose().getX(), obj->getPose().getY(), obj->getPose().getTh(), 
  obj->getFromPose().getX(), obj->getFromPose().getY(), 
  obj->getToPose().getX(), obj->getToPose().getY());

  // Find a unique name and id
  char token[32];
  strncpy(token, "Box:0", 31);
  token[31] = '\0';
  size_t i = 0;
  for(; stg_world_model_name_lookup(world, token) != NULL; i++)
  {
    snprintf(token, 32, "Box:%lu", i); // try next number
  }

  // find a unique name
  stg_id_t id = 0;
  for(id = 0; stg_world_get_model(world, id) != NULL; id++)
    ;

  // create model. note, using null parent. making the map a parent causes
  // problems right now. TODO use stg_world_new_model instead!
  stg_model_t* box_model = stg_model_create(world, NULL, id, token, "model", "model", i, NULL, FALSE);


  // size
  stg_size_t box_size;
  box_size.x = (obj->getToPose().getX() - obj->getFromPose().getX()) / 1000.0;
  box_size.y = (obj->getToPose().getY() - obj->getFromPose().getY()) / 1000.0;
  stg_model_set_size(box_model, box_size);

  // pose. Map object pose is not used for some reason, it needs to be
  // positioned according to the "from" and "to" poses, in this slightly
  // complex way.
  stg_pose_t box_pose;
  double sinObjTh, cosObjTh;
  STG_SINCOS(DTOR(obj->getPose().getTh()), sinObjTh, cosObjTh);
  double x = (obj->getFromPose().getX() / 1000.0) + (box_size.x / 2.0);
  double y = (obj->getFromPose().getY() / 1000.0) + (box_size.y / 2.0);
  box_pose.x = (x * cosObjTh) - (y * sinObjTh);
  box_pose.y = (x * sinObjTh) + (y * cosObjTh);
  box_pose.a = DTOR(obj->getPose().getTh());
  stg_model_set_pose(box_model, box_pose);

  // color
  if(object_class.laser_return > 1)
  {
      stg_color_t color = stg_lookup_color("magenta");
      stg_model_set_property(box_model, "color", &color, sizeof(color));
  }
  else
  {
    stg_color_t color = stg_lookup_color("light green");
    stg_model_set_property(box_model, "color", &color, sizeof(color));
  }

  // shape
  stg_polygon_t* box_polys = stg_polygons_create(1);
  //stg_point_t v1 = { obj->getFromPose().getX()/1000.0, obj->getFromPose().getY()/1000.0 };
  //stg_point_t v2 = { obj->getToPose().getX()/1000.0, obj->getFromPose().getY()/1000.0 };
  //stg_point_t v3 = { obj->getToPose().getX()/1000.0, obj->getToPose().getY() / 1000.0 };
  //stg_point_t v4 = { obj->getFromPose().getX()/1000.0, obj->getToPose().getY()/1000.0 };
  stg_point_t v1 = { -box_size.x/2.0, -box_size.y/2.0 };   // actually equivalent... ?
  stg_point_t v2 = { box_size.x/2.0, -box_size.y/2.0 };
  stg_point_t v3 = { box_size.x/2.0, box_size.y/2.0 };
  stg_point_t v4 = { -box_size.x/2.0, box_size.y/2.0 };
  stg_polygon_append_points(&box_polys[0], &v1, 1);
  stg_polygon_append_points(&box_polys[0], &v2, 1);
  stg_polygon_append_points(&box_polys[0], &v3, 1);
  stg_polygon_append_points(&box_polys[0], &v4, 1);
  stg_model_init_polygons(box_model, box_polys, 1);

  int box_movemask = 1|2; // movable and rotatable
  int box_outline = 1;  // has a black outline
  stg_model_set_property(box_model, "mask", &box_movemask, sizeof(int));
  stg_model_set_property(box_model, "outline", &box_outline, sizeof(int));

  int box_obstacle_return = object_class.obstacle?1:0;
  stg_model_set_property(box_model, "obstacle_return", &box_obstacle_return, sizeof(box_obstacle_return));
  stg_model_set_property(box_model, "laser_return", &object_class.laser_return, sizeof(object_class.laser_return));
  int ranger_return = object_class.sonar_return?1:0;
  stg_model_set_property(box_model, "ranger_return", &ranger_return, sizeof(ranger_return));

  stg_world_add_model(world, box_model);
  //printf("loadMap: added box model \"%s\" to world.\n", stg_model_get_token(box_model));
  return box_model;
}

bool MapLoader::shouldReloadMap(const std::string& newmapfile)
{
  struct stat filestat;
  int s = stat(newmapfile.c_str(), &filestat);
  if(s != 0)
  return true;  // no way to know modification time, force reload

  if(hostHasEM)
  {
    const bool printTimeCompare = true;
    if (printTimeCompare)
    {
      char filemod_buff[50], modrec_buff[50];
      struct tm * timeinfo;
      timeinfo = localtime (&(filestat.st_mtime));
      strftime(filemod_buff, sizeof(filemod_buff), "%b %d %H:%M:%S", timeinfo);
      //delete(timeinfo);
      //if (lastMapReloadTime != 0)
      {
        timeinfo = localtime (&(lastMapReloadTime));
        strftime(modrec_buff, sizeof(modrec_buff), "%b %d %H:%M:%S", timeinfo);
      }
      //delete(timeinfo);
      //ArLog::log(ArLog::Normal, "shouldReloadMap: %s's last update: %s, lastMapReloadTime: %s", newmapfile.c_str(), filemod_buff, modrec_buff);
    }

    if (difftime( filestat.st_mtime, lastMapReloadTime ) > 0.0)
    {
      lastMapReloadTime = filestat.st_mtime;
      return true;
    }
    else
      return false;
  }

  bool hadModelsFromThatMap = false;
  for(std::set<stg_model_t*>::const_iterator i = mapModels.begin(); i != mapModels.end(); ++i)
  {
    size_t len;
    char *source = (char*)stg_model_get_property(*i, "source", &len);
    //ArLog::log(ArLog::Normal, "MapLoader::shouldReloadMap(): newmapfile (%s), source (%s), len (%p)", newmapfile.c_str(), source, len);
    if(source && len > 0 && newmapfile == source)
    {
      // model came from this map file
      hadModelsFromThatMap = true;
      time_t *modelTime = (time_t*)stg_model_get_property_fixed(*i, "creation_time", sizeof(time_t));
      if(!modelTime) {
        //ArLog::log(ArLog::Normal, "MapLoader::shouldReloadMap(): model %d had no creation_time. Reloading map", *i);
        stg_world_unlock(world);
        return true;  // no creation_time, no way to know if it's newer, always reload map file
      }
      if( difftime( filestat.st_mtime, *modelTime ) > 0.0) {
        //ArLog::log(ArLog::Normal, "MapLoader::shouldReloadMap(): map file has modification time %d (%s), newer than model %s's creation time %d (%s). must reload.\n", filestat.st_mtime, ctime(&filestat.st_mtime), stg_model_get_token(*i), *modelTime, ctime(modelTime));
        stg_world_unlock(world);
        return true; 
      }
    }
  }
  if(hadModelsFromThatMap) {
    return false;
  } else {
    //ArLog::log(ArLog::Normal, "MapLoader::shouldReloadMap(): had no models from that map");
    return true;   // force reload if no models came from that map (it's a completely new map)
  }
}


void MapLoader::invokeMapLoadedCallback(MapLoadedCallback cb, int status, const std::string& filename, ArMap* map) const
{
  if(!cb) return;
  MapLoadedInfo info;
  //ArLog::log(ArLog::Normal, "MapLoader::invokeMapLoadedCallback(): DEBUG 03");
  if(map)
  {
    info.map = map;
    ArMapObject *home = map->findFirstMapObject(NULL, "RobotHome");
    if(!home)
    {
      home = map->findFirstMapObject(NULL, "Dock");
    }
    if(home)
    {
      info.have_home = true;
      info.home_x = home->getPose().getX();
      info.home_y = home->getPose().getY();
      info.home_th = home->getPose().getTh();
    }
    info.min_x = map->getLineMinPose().getX();
    info.min_y = map->getLineMinPose().getY();
    info.max_x = map->getLineMaxPose().getX();
    info.max_y = map->getLineMaxPose().getY();
  }
  info.filename = filename;
  info.status = status;
  //ArLog::log(ArLog::Normal, "MapLoader::invokeMapLoadedCallback(): calling cb: %p", (void*)cb);
  cb->invoke(info);
  //ArLog::log(ArLog::Normal, "MapLoader::invokeMapLoadedCallback(): successful callback");
}


void MapLoader::invokeMapLoadedCallbacks(int status, const std::string& filename, ArMap* map) const // JPL - experimental
{
  if (callbacks.empty())
    return;

}

void MapLoader::reset()
{
  //ArLog::log(ArLog::Normal, "MapLoader::reset()");

  loading = false;
  if(map && created_map)
  {
    delete map;
  }
  map = nullptr;


  myNumPolys = myNumLines = myNumPoints = 0;
  myMapModel = nullptr;
  myMapPolysChunks.clear();// TODO: ask Reid about why this doesn't cause a memory leak, and why it crashes when I try to free() the memory when it != NULL
  myMapPointsChunks.clear();// TODO: ask Reid about why this doesn't cause a memory leak, and why it crashes when I try to free() the memory when it != NULL
  myMapPolysModels.clear();
  myMapPointsModels.clear();
  myModelsToInit.clear();
}

const char *MapLoader::stateName(NewMapProcessState state)
{
  switch (state)
  {
    case MapLoader::NEWMAP_INACTIVE:       return "INACTIVE";
    case MapLoader::NEWMAP_STARTPROCESS:          return "STARTPROCESS";
    case MapLoader::NEWMAP_LOADLINESTART:  return "LOADLINESTART";
    case MapLoader::NEWMAP_LOADLINECONT:   return "LOADLINECONT";
    case MapLoader::NEWMAP_LOADPOINTSTART: return "LOADPOINTSTART";
    case MapLoader::NEWMAP_LOADPOINTCONT:  return "LOADPOINTCONT";
    case MapLoader::NEWMAP_LOADCHECK:      return "LOADCHECK";
    case MapLoader::NEWMAP_STAGECREATE:    return "STAGECREATE";
    //case MapLoader::NEWMAP_STAGEGRID:      return "STAGEGRID");
    case MapLoader::NEWMAP_STAGEINITPOLY:  return "STAGEINITPOLY";
    case MapLoader::NEWMAP_STAGEINITPOINT: return "STAGEINITPOINT";
    case MapLoader::NEWMAP_STAGECLEAR:     return "STAGECLEAR";
    case MapLoader::NEWMAP_STAGEINSERT:    return "STAGEINSERT";
    case MapLoader::NEWMAP_CUSTTYPESTART:  return "CUSTTYPESTART";
    case MapLoader::NEWMAP_CUSTTYPECONT:   return "CUSTTYPECONT";
    case MapLoader::NEWMAP_CAIRNSTART:     return "CAIRNSTART";
    case MapLoader::NEWMAP_CAIRNCONT:      return "CAIRNCONT";
    case MapLoader::NEWMAP_RESIZE:         return "RESIZE";
    case MapLoader::NEWMAP_CALLBACK:       return "CALLBACK";
    case MapLoader::NEWMAP_RESET:          return "RESET";
  }
  return "<unknown>";
}
