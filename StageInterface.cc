/*

  (C) Copyright 2005, ActivMedia Robotics, LLC
  (C) Copyright 2006-2010 MobileRobots, Inc.
  (C) Copyright 2011-2015 Adept MobileRobots <http://www.mobilerobots.com>
  (C) Copyright 2016-2017 Omron Adept Technologies

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/

// to maybe get extensions like sincos():
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#ifndef __USE_GNU
#define __USE_GNU 1
#endif


#include "StageInterface.hh"
#include "EmulatePioneer.hh"

#include "ariaUtil.h"
#include <map>
#include <deque>
#include <vector>

#include <math.h>


#include "stage.h"


StageInterface::StageInterface(stg_world_t* _world, std::string _robotModel, std::string _robotName) :
  RobotInterface(_robotName), params(NULL),
  stageWorld(_world), robotModel(_robotModel), robotName(_robotName),
  positionModel(NULL), sonarModel(NULL), //laserModel(NULL), //messagesModel(NULL),
  subscribedToSonar(false),
  openedSonar(false),
  motorsEnabled(true)
{
}

StageInterface::StageInterface(stg_world_t* _world, stg_model_t* _model, std::string _robotModel, std::string _robotName) :
  RobotInterface(_robotName), params(NULL),
  stageWorld(_world), robotModel(_robotModel), robotName(_robotName),
  positionModel(_model), sonarModel(NULL), //laserModel(NULL), //messagesModel(NULL),
  subscribedToSonar(false),
  openedSonar(false),
  motorsEnabled(true)
{
}

StageInterface::Laser::Laser(size_t i, stg_model_t* model) :
    subscribed(false), opened(false), stageModel(model), startAngle(-90.0), endAngle(90.0)
{
  DeviceInfo::name = stg_model_get_token(model);
  DeviceInfo::type = stg_model_get_instance_type_name(model);
  DeviceInfo::basetype = stg_model_get_base_type_name(model);
  DeviceInfo::which = i;
}

StageInterface::~StageInterface()
{
  //if(messagesModel)
  //{
  //  stg_world_remove_model(stageWorld, messagesModel);
  //  stg_model_destroy(messagesModel);
  //}

  //printf("StageInterface destructor\n");

  for(std::vector<Laser>::iterator i = lasers.begin(); i != lasers.end(); ++i)
  {
    (*i).destroy();
  }
  if(sonarModel)
  {
    stg_world_remove_model(stageWorld, sonarModel);
    stg_model_destroy(sonarModel);
  }
  if(positionModel)
  {
    //print_debug("~StageInterface: removing and destroying position model and any children:");
    //stg_model_print_tree(positionModel);

    stg_world_remove_model(stageWorld, positionModel);
    stg_model_destroy_tree(positionModel); // destroy children just in case
  }
}

StageInterface::Laser::~Laser() {
}

void StageInterface::Laser::destroy() {
  if(stageModel) {
    if(subscribed)
      stg_model_unsubscribe(stageModel);
    if(stg_model_get_world(stageModel))
      stg_world_remove_model(stg_model_get_world(stageModel), stageModel);
    stg_model_destroy(stageModel);
    stageModel = 0;
  }
}

/// @todo move parameter-related stuff into params class (or a stage-specific class for params)
/// @todo create paramater accessor functions that query stage each time, so
/// that they can be dynamically changed in gui, or mark as unchangable at
/// runtime.
void StageInterface::connect(RobotParams* params)
{
  //ArTime timer;
  //timer.setToNow();


  /* Set initial robot subtype from name the user used to make it */
  strncpy(params->RobotSubclass, robotModel.c_str(), ROBOT_IDENTIFIER_LEN);

  // initialize command struct for position commands:
  memset(&positionCmd, 0, sizeof(positionCmd));

  /* find stage models and subscribe */

  // Get position model if neccesary
  if(!positionModel)
  {
    positionModel = stg_world_model_name_lookup(stageWorld, robotName.c_str());
    //print_debug("found position model for \"%s\": %p", robotName.c_str(), positionModel);
  }
  if(!positionModel) 
  {
    error("could not find a base robot position model named \"%s\" in the world", robotName.c_str());
    shutdown(-10);
    return; 	// todo return an error code so emulatepioneer thread can stop
  }

  stg_model_subscribe(positionModel);

  // Get the robot subtype from the position model, that's important:
  size_t len;
  const char* s = (const char*) stg_model_get_property(positionModel, "pioneer_robot_subtype", &len);
  if(s != NULL)
  {
    strncpy(params->RobotSubclass, s, ROBOT_IDENTIFIER_LEN);
  }

  s = (const char*) stg_model_get_property(positionModel, "pioneer_robot_type",
&len);
  if(s != NULL)
    strncpy(params->RobotClass, s, ROBOT_IDENTIFIER_LEN);

  // Get other models that are children of the position: 

  //if(stg_world_gui_enabled(stageWorld)) {
  //  messagesModel = stg_model_find_first_child_with_type(positionModel, (char*) "messages");
  //}

  sonarModel = stg_model_find_first_child_with_type(positionModel, (char*) "ranger");
  if(!sonarModel && !stg_world_get_quiet(stageWorld)) 
  {
    warn_s("No sonar model defined for this robot");
  }

  /* Laser model */
  stg_model_t* laserModel = stg_model_find_first_child_with_type(positionModel, (char*) "laser");
  if(laserModel) {
    stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(laserModel, "laser_cfg", sizeof(stg_laser_config_t));
    assert(lasercfg);
    lasers.push_back(Laser(0, laserModel));
    lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(laserModel, "laser_cfg", sizeof(stg_laser_config_t));
    assert(lasercfg);
    // stage only has symmetrical laser  at some fov
    double laserStart, laserEnd;
    if(lasercfg->reverse_scan)
    {
      laserEnd = -(RTOD(lasercfg->fov) / 2.0);
      laserStart = (RTOD(lasercfg->fov) / 2.0);
    } else {
      laserStart = -(RTOD(lasercfg->fov) / 2.0);
      laserEnd = (RTOD(lasercfg->fov) / 2.0);
    }
    lasers[0].startAngle = laserStart;
    lasers[0].endAngle = laserEnd;
  }
  else
  {
    if(!stg_world_get_quiet(stageWorld))
      inform_s("Note: No laser model defined for this robot");
  }



  /* Get our custom parameters from Stage's model definitions (these properties
  * were registered by MobileSim's main.cc): */

  /// @todo Need to get *all* robot parameters, including speed config etc. !!

  this->params = params;  // make a copy to store the original defaults
  //assert(positionModel);

  float *f;
  int *i;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_distconv", sizeof(float));
  if(f) params->DistConvFactor = *f;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_diffconv", sizeof(float));
  if(f) params->DiffConvFactor = *f;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_angleconv", sizeof(float));
  if(f) params->AngleConvFactor = *f;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_rangeconv", sizeof(float));
  if(f) params->RangeConvFactor = *f;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_vel2div", sizeof(float));
  if(f) params->Vel2DivFactor = *f;

  f = (float*)stg_model_get_property_fixed(positionModel, "pioneer_velconv", sizeof(float));
  if(f) params->VelConvFactor = *f;

  //i = (int*)stg_model_get_property_fixed(positionModel, "pioneer_sip_cycle", sizeof(int));
  //if(i) params->SIPFreq = *i;

  i = (int*)stg_model_get_property_fixed(positionModel, "pioneer_watchdog", sizeof(int));
  if(i) params->WatchdogTime = *i;

  if(sonarModel)
  {
    i = (int*) stg_model_get_property_fixed(sonarModel, "pioneer_max_readings_per_packet", sizeof(int));
    if(i) params->Sim_MaxSonarReadingsPerSIP = *i;
  }

  i = (int*)stg_model_get_property_fixed(positionModel, "pioneer_batterytype", sizeof(int));
  if(!i) i = (int*)stg_model_get_property_fixed(positionModel, "pioneer_battery_type", sizeof(int));
  if(i) 
  {
    log("Read battery type %d from model definition", *i);
    params->BatteryType = *i;
  }
  else 
  {
    params->BatteryType = 0;
  }

  f = (float*) stg_model_get_property_fixed(positionModel, "pioneer_gps_pos_x", sizeof(float));
  if(f) params->GPSPosX = (int) (*f * 1000.0);
  f = (float*) stg_model_get_property_fixed(positionModel, "pioneer_gps_pos_y", sizeof(float));
  if(f) params->GPSPosY = (int) (*f * 1000.0);


// For Debugging:
/*
puts("StageInterface: now set up and ready to use with the following stage model(s):");
stg_model_print_children(positionModel);
*/

} 


void StageInterface::disconnect()
{
  stop();
  enableMotors(false);
  closeSonar();
  for(std::vector<Laser>::iterator i = lasers.begin(); i != lasers.end(); ++i)
    (*i).close();
  setOdom(0, 0, 0);
  if(positionModel)
  {
    stg_model_unsubscribe(positionModel);
  }
  params = NULL;
}




void StageInterface::enableMotors(bool e) 
{
  motorsEnabled = e;
  if(!e) stop();
}


void StageInterface::transVel(int v) 
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  positionCmd.x = (v / 1000.0);  // mm to m
  positionCmd.transmode = STG_POSITION_CONTROL_VELOCITY;
  positionCmd.override_decel.x = 0.0;
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::latVel(int v)
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  positionCmd.y = (v / 1000.0);   // mm t m
  positionCmd.transmode = STG_POSITION_CONTROL_VELOCITY;
  positionCmd.override_decel.y = 0.0;
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::rotVel(int v)
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  positionCmd.a = DTOR((double)v); // degrees to radians
  positionCmd.rotmode = STG_POSITION_CONTROL_VELOCITY;
  positionCmd.override_decel.a = 0.0;
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::move(int m)
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  positionCmd.x = (m / 1000.0);
  positionCmd.transmode = STG_POSITION_CONTROL_RELATIVE;
  positionCmd.override_decel.x = 0.0;
  char active = 1;
  stg_model_set_property(positionModel, "position_rel_ctrl_active_X", &active, sizeof(active));
  stg_model_set_property(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
  stg_pose_t *progress = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "position_rel_ctrl_progress", sizeof(stg_pose_t));
  progress->x = 0.0; // reset if a current move is currenly in progress.
  stg_model_property_changed(positionModel, "position_rel_ctrl_progress");
}


void StageInterface::heading(int h)
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  positionCmd.a = DTOR((double)h);
  positionCmd.rotmode = STG_POSITION_CONTROL_POSITION;
  positionCmd.override_decel.a = 0.0;
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::deltaHeading(int h)
{
  if(!motorsEnabled) return;
  // TODO get property pointer and modify rather than copying new struct in
  // TODO: normalize?
  positionCmd.a = DTOR((double)h);
  positionCmd.rotmode = STG_POSITION_CONTROL_RELATIVE;
  positionCmd.override_decel.a = 0.0;
  char active = 1;
  stg_model_set_property(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
  stg_model_set_property(positionModel, "position_rel_ctrl_active_A", &active, sizeof(active));
  stg_pose_t *progress = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "position_rel_ctrl_progress", sizeof(stg_pose_t));
  progress->a = 0.0; // reset if a current delta heading is currenly in progress.
  stg_model_property_changed(positionModel, "position_rel_ctrl_progress");
}


void StageInterface::stop()
{
  // TODO get property pointer and modify rather than copying new struct in
  // clear the position command to all zero
  memset(&positionCmd, 0, sizeof(positionCmd));
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::stop(int transDecel, int rotDecel)
{
  memset(&positionCmd, 0, sizeof(positionCmd));
  positionCmd.override_decel.x = positionCmd.override_decel.y = (transDecel / 1000.0);
  positionCmd.override_decel.a = DTOR(rotDecel);
  stg_model_set_property_locked(positionModel, "position_cmd", &positionCmd, sizeof(positionCmd));
}

void StageInterface::stall(bool stalled)
{
  stg_position_stall_t* stall = (stg_position_stall_t*)stg_model_get_property_fixed(positionModel, "position_stall", sizeof(stg_position_stall_t));
  if(stalled) *stall = 1;
  else        *stall = 0;
  stg_model_property_changed(positionModel, "position_stall");
}

void StageInterface::setAccel(int a)
{
  if(a < 0)
  {
    warn("Negative rot. acceleration value requested! Ignoring.");
    return;
  }
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_accel.x = (a/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setDecel(int d)
{
  if(d < 0)
  {
    warn("Negative trans. deceleration value requested! Ignoring.");
    return;
  }
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_decel.x = (d/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}


void StageInterface::setRotAccel(int a)
{
  if(a < 0)
  {
    warn("Negative rot. acceleration value requested! Ignoring.");
    return;
  }
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_accel.a = DTOR(a);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setRotDecel(int d)
{
  if(d < 0)
  {
    warn("Negative rot. deceleration value requested! Ignoring.");
    return;
  }
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_decel.a = DTOR(d);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setLatAccel(int a)
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_accel.y = (a/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setLatDecel(int d)
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->current_decel.y = (d/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setMaxVel(int v) 
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->max_speed.x = (v/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setMaxRotVel(int v)
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->max_speed.a = DTOR(v);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setDefaultVel(int v)
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->default_speed.x = (v/1000.0);
  stg_model_property_changed(positionModel, "position_speed_config");
}

void StageInterface::setDefaultRotVel(int v)
{
  stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*)stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
  speed_cfg->default_speed.a = DTOR(v);
  stg_model_property_changed(positionModel, "position_speed_config");
}

bool StageInterface::haveGripper() {
  return false;
}

bool StageInterface::haveSonar() {
  return sonarModel != NULL;
    // TODO front || rear
}

bool StageInterface::haveFrontSonar() {
  return haveSonar();
  // TODO check sonar model for # sonar and poses? or a user property in the
  // model?
}

bool StageInterface::haveRearSonar() {
  return haveSonar();
  // TODO check sonar model for # sonar and poses? or a user property in the
  // model?
}

void StageInterface::setOdom(int x, int y, int theta)
{
  stg_position_data_t* odomData = (stg_position_data_t*) stg_model_get_property_fixed(positionModel, "position_data", sizeof(stg_position_data_t));
  if(odomData)
  {
    odomData->pose.x = (x / 1000.0);
    odomData->pose.y = (y / 1000.0);
    odomData->pose.a = DTOR((double)theta);
    stg_model_property_changed(positionModel, "position_data");
  }
}

void StageInterface::setSimulatorPose(long int x, long int y, long int /*z*/, int theta)
{
  stg_pose_t* p = (stg_pose_t*) stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  p->x = (x / 1000.0);
  p->y = (y / 1000.0);
  p->a = DTOR((double)theta);
  stg_model_property_changed(positionModel, "pose");
}

void StageInterface::resetSimulatorPose()
{
  stg_model_reset_pose(positionModel);
}

bool StageInterface::havePositionData() {
  return positionModel != NULL;
}

bool StageInterface::haveLaser(size_t i) {
  return (i < lasers.size());
}

void StageInterface::getPosition(int &x, int &y, int &theta) {
  stg_position_data_t* posdata = stagePositionData();
  x = (int) ArMath::roundInt(posdata->pose.x * 1000.0);
  y = (int) ArMath::roundInt(posdata->pose.y * 1000.0);
  theta = (int) ArMath::roundInt(RTOD(posdata->pose.a));
}

void StageInterface::getVelocity(int &x, int &y, int &theta) {
  stg_velocity_t* veldata = stageVelocityData();
  x = (int) ArMath::roundInt(veldata->x * 1000.0);
  y = (int) ArMath::roundInt(veldata->y * 1000.0);
  theta = (int) ArMath::roundInt(RTOD(veldata->a));
}


void StageInterface::getMotionState(int &x, int &y, int &theta, int &transVel, int &rotVel, bool &stalled, bool &enabled) {
  stg_position_data_t* posdata = stagePositionData();
  x = (int) ArMath::roundInt(posdata->pose.x * 1000.0);
  y = (int) ArMath::roundInt(posdata->pose.y * 1000.0);
  theta = (int) ArMath::roundInt(RTOD(posdata->pose.a));
  stg_velocity_t* veldata = stageVelocityData();
  transVel = (int) ArMath::roundInt(veldata->x * 1000.0);
  rotVel = (int) ArMath::roundInt(RTOD(veldata->a));
  stg_position_stall_t* stall = (stg_position_stall_t*)stg_model_get_property_fixed(positionModel, "position_stall", sizeof(stg_position_stall_t));
  stalled = ((*stall) != 0);
  enabled = motorsEnabled;
}

int StageInterface::xpos() {
  int x;
  stg_position_data_t* data = stagePositionData();
  if(data)
    x = (int) ArMath::roundInt(data->pose.x * 1000.0);
  else
    x = 0;
  return x;
}

int StageInterface::ypos() {
  int y;
  stg_position_data_t* data = stagePositionData();
  if (data) 
    y = (int) ArMath::roundInt(data->pose.y * 1000.0);
  else
    y = 0;
  return y;
}


int StageInterface::theta() {
  int th;
  stg_position_data_t* data = stagePositionData();
  if(data)
    th = (int) ArMath::roundInt(RTOD(data->pose.a));
  else
    th = 0;
  return th;
}

int StageInterface::xspeed() {
  stg_velocity_t* data = stageVelocityData();
  int x;
  if(data)
    x = (int) ArMath::roundInt(data->x * 1000.0);
  else
    x = 0;
  return x;
}

int StageInterface::yspeed() {
  stg_velocity_t* data = stageVelocityData();
  int y;
  if(data)
    y = (int) ArMath::roundInt(data->y * 1000.0);
  else 
    y = 0;
  return y;
}

int StageInterface::rotspeed() {
  stg_velocity_t* data = stageVelocityData();
  int r;
  if(data) r = (int) ArMath::roundInt(RTOD(data->a));
  else r = 0;
  return r;
}

bool StageInterface::stalled() {
  stg_position_stall_t* stall = (stg_position_stall_t*)stg_model_get_property_fixed(positionModel, "position_stall", sizeof(stg_position_stall_t));
  bool s = (stall && (*stall) != 0);
  return s;
}

size_t StageInterface::numSonarReadings() {
  if(!subscribedToSonar) {
    return 0;
  }
  size_t len = 0;
  stg_model_get_property(sonarModel, "ranger_data", &len);
  return len / sizeof(stg_ranger_sample_t);
}

size_t StageInterface::numLaserReadings(size_t i) {
  if(i >= lasers.size()) {
    return 0;
  }
  return lasers[i].numReadings();
}

size_t StageInterface::Laser::numReadings() {
  if(!opened || !subscribed) return 0;
  size_t len = 0;
  stg_model_get_property(stageModel, "laser_data", &len);
  return len / sizeof(stg_laser_sample_t);
}

int StageInterface::getSonarReading(int i) {
  assert(subscribedToSonar);
  size_t len = 0;
  stg_ranger_sample_t* data = (stg_ranger_sample_t*)stg_model_get_property(sonarModel, "ranger_data", &len);
  int numSonarReadings = len / sizeof(stg_ranger_sample_t);
  assert(i <= numSonarReadings && i >= 0);
  int r =(int) ArMath::roundInt(data[i].range * 1000.0); 
  return  r;
}

size_t StageInterface::forEachSonarReading(SonarReadingFunc &func, const size_t &start) {
  size_t len = 0;
  stg_ranger_sample_t *data = (stg_ranger_sample_t*)stg_model_get_property(sonarModel, "ranger_data", &len);
  if(!data) {
    //puts(">> sonar data is null.");
    return 0;
  }
  size_t n = len / sizeof(stg_ranger_sample_t);
  size_t i;
  //printf(">> XXX forEachSonarReading: there are %d sonar readings. start=%d\n", n, start);
  for(i = start; i < n; ++i) {
    if(!func( (int) ArMath::roundInt(data[i].range * 1000.0 ))) break;
  }
  // printf(">> XXX forEachSonarReading: did %d readings\n", i-start);
  return i - start;
}

int StageInterface::getLaserReading(size_t lasernum, int i) {
  if(lasernum >= lasers.size())
    return 0;
  return lasers[lasernum].getReading(i);
}

int StageInterface::Laser::getReading(int i) {
  if(!stageModel || !subscribed || !opened ) return 0;
  size_t len = 0;
  stg_laser_sample_t* data = (stg_laser_sample_t*)stg_model_get_property(stageModel, "laser_data", &len);
  int numLaserReadings = len / sizeof(stg_laser_sample_t);
  assert(i <= numLaserReadings && i >= 0);
  return data[i].range;
}

int StageInterface::getLaserReflectance(size_t lasernum, int i) {
  if(lasernum >= lasers.size())
    return 0;
  return lasers[lasernum].getReflectance(i);
}

int StageInterface::Laser::getReflectance(int i) {
  if(!stageModel || !subscribed || !opened)
    return 0;
  size_t len = 0;
  stg_laser_sample_t* data = (stg_laser_sample_t*)stg_model_get_property(stageModel, "laser_data", &len);
  int numLaserReadings = len / sizeof(stg_laser_sample_t);
  assert(i <= numLaserReadings && i >= 0);
  return data[i].reflectance;
}

size_t StageInterface::forEachLaserReading(size_t lasernum, LaserReadingFunc &func, const size_t &start) {
  if(lasernum >= lasers.size())
    return 0;
  return lasers[lasernum].forEachReading(func, start);
}

size_t StageInterface::Laser::forEachReading(LaserReadingFunc &func, const size_t &start) {
  if(!stageModel || !subscribed || !opened)
    return 0;
  size_t len = 0;
  stg_laser_sample_t *data = (stg_laser_sample_t*)stg_model_get_property(stageModel, "laser_data", &len);
  if(!data)
  {
    return 0;
  }
  size_t n = len / sizeof(stg_laser_sample_t);
  //printf("forEachLaserReading: have %d laser samples. (starting at %d)\n", n, start);
  size_t i;
  size_t x = 0;
  for(i = start; i < n; ++i) {
    bool r = func(data[i].range, data[i].reflectance);
    ++x;
    if(!r) break;
  }
  //return i - start;
  return x;
}

void StageInterface::openSonar() {
  RobotInterface::openSonar();
  if(!sonarModel)
  {
    return;
  }
  if(openedSonar) {
    return;
  }
  openedSonar = true;
  if(!subscribedToSonar) {
    stg_model_subscribe(sonarModel);
    subscribedToSonar = true;
  }
  if(!stg_world_get_quiet(stageWorld))
    inform("Sonar on");
}

void StageInterface::closeSonar() {
  RobotInterface::closeSonar();
  if(!openedSonar) {
    return;
  }
  openedSonar = false;
  //memset(sonarData, 0, sizeof(sonarData));
  //numSonarSamples = 0;
  if(sonarModel && subscribedToSonar) {
    stg_model_unsubscribe(sonarModel);
    subscribedToSonar = false;
  }
  if(!stg_world_get_quiet(stageWorld))
    inform("Sonar off");
}

void StageInterface::Laser::open() {
  if(!stageModel) return;
  if(opened) return;
  opened = true;
  if(!subscribed)
  {
    stg_model_subscribe(stageModel);
    subscribed = true;
  }
}

void StageInterface::openLaser(size_t i) {
  if(i >= lasers.size())
    return;
  lasers[i].open();
}


void StageInterface::Laser::close() {
  if(!opened) return;
  if(stageModel && subscribed) {
    stg_model_unsubscribe(stageModel);
    subscribed = false;
  }
  opened = false;
}

void StageInterface::closeLaser(size_t i) {
  if(i >= lasers.size())
    return;
  lasers[i].close();
}

char StageInterface::gripperState() {
  // TODO 
  return 0;
}

void StageInterface::setLaserResolution(size_t i, double inc) {
  if(i >= lasers.size())
  {
    warn("don't have laser %d, can't set resolution");
    return;
  }
  lasers[i].setResolution(inc);
}

void StageInterface::Laser::setResolution(double inc) {
  if(!stageModel) {
    return;
  }
  stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(stageModel, "laser_cfg", sizeof(stg_laser_config_t));
  assert(lasercfg);
  if(inc == 0)
    lasercfg->samples = 0;
  else
    lasercfg->samples = (int) ceil( ceil(RTOD(lasercfg->fov)) / inc) + 1;  // readings should fall at beginning of each degree as well as the end of the last degree
  stg_model_property_changed(stageModel, "laser_cfg");
}

double StageInterface::getLaserResolution(size_t i) {
  if(i >= lasers.size()) {
    warn("this robot has no laser %d, can't get its resolution.", i);
    return 0.0;
  }
  return lasers[i].getResolution();
}

double StageInterface::Laser::getResolution() {
  stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(stageModel, "laser_cfg", sizeof(stg_laser_config_t));
  assert(lasercfg);
  //fprintf(stderr, "StageInterface::getLaserResolution(): returning %f\n", RTOD(lasercfg->fov) / lasercfg->samples);
  double res;
  if(lasercfg->samples == 0)
    res = 0;
  else
    res = RTOD(lasercfg->fov) / lasercfg->samples;
  return res;
}

void StageInterface::setLaserFOV(size_t i, double deg) {
  if(i >= lasers.size()) return;
  lasers[i].setFOV(deg);
}

void StageInterface::Laser::setFOV(double deg) {
  if(!stageModel) return ;
  stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(stageModel, "laser_cfg", sizeof(stg_laser_config_t));
  assert(lasercfg);
  lasercfg->fov = DTOR(deg);
  stg_model_property_changed(stageModel, "laser_cfg");
  startAngle = 0.0-(deg/2.0);
  endAngle = (deg/2.0);
}

double StageInterface::getLaserFOV(size_t i) {
  if(i >= lasers.size()) return 0;
  return lasers[i].getFOV();
}

double StageInterface::Laser::getFOV() {
  if(!stageModel) return 0.0;
  stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(stageModel, "laser_cfg", sizeof(stg_laser_config_t));
  assert(lasercfg);
  double fov = RTOD(lasercfg->fov);
  return fov;
}

double StageInterface::getLaserStartAngle(size_t i) {
  if(i >= lasers.size()) return 0.0;
  return lasers[i].startAngle;
}

double StageInterface::getLaserEndAngle(size_t i) {
  if(i >= lasers.size()) return 0.0;
  return lasers[i].endAngle;
}

void StageInterface::setLaserAngles(size_t i, double start, double end) {
  if(i >= lasers.size()) return;
  lasers[i].setAngles(start, end);
}

void StageInterface::Laser::setAngles(double start, double end) {
  if(!stageModel) return;
  // stage only has symmetrical laser at some fov angle
  stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(stageModel, "laser_cfg", sizeof(stg_laser_config_t));
  assert(lasercfg);
  //fprintf(stderr, "StageInterface::setLaserAngles(%f,%f): setting fov to %f rad. reverse scan? -> %d.\n", start, end, DTOR( fabs( fabs(end+360.0) - fabs(start+360.0) ) ), (end < start));
  lasercfg->fov = DTOR( fabs( fabs(end+360.0) - fabs(start+360.0) ) );
  lasercfg->reverse_scan = (end < start);
  startAngle = start;
  endAngle = end;
  stg_model_property_changed(stageModel, "laser_cfg");
}

stg_position_data_t* StageInterface::stagePositionData() {
  stg_position_data_t* data = (stg_position_data_t*) stg_model_get_property_fixed(positionModel, "position_data", sizeof(stg_position_data_t));
  assert(data);
  return data;
}

stg_velocity_t* StageInterface::stageVelocityData() {
  stg_velocity_t* data = (stg_velocity_t*) stg_model_get_property_fixed(positionModel, "velocity", sizeof(stg_velocity_t));
  assert(data);
  return data;
}

void StageInterface::getSimulatorPose(long &x, long &y, long &z, int &theta)
{
  stg_pose_t *pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  x = (long)(pose->x * 1000.0);
  y = (long)(pose->y * 1000.0);
  z = 0;
  theta = (int)ArMath::roundInt(RTOD(pose->a));
}

long StageInterface::getSimulatorPoseX() {
  stg_pose_t* pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  assert(pose);
  int x = (int)(pose->x * 1000.0);
  return x;
}

long StageInterface::getSimulatorPoseY() {
  stg_pose_t* pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  assert(pose);
  int y = (int)(pose->y * 1000.0);
  return y;
}

int StageInterface::getSimulatorPoseTheta() {
  stg_pose_t* pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  assert(pose);
  int th = (int)ArMath::roundInt(RTOD(pose->a));
  return th;
}

int StageInterface::getLastInterval() { 
  int i = stg_world_get_last_interval(stageWorld);
  return i;
}

int StageInterface::getSimInterval() {
  int i = stg_world_get_sim_interval(stageWorld);
  return i;
}

int StageInterface::getRealInterval() {
  int i = stg_world_get_real_interval(stageWorld);
  return i;
}

void StageInterface::error_s(const char* message)
{
  stg_world_display_message_s(stageWorld, 0, robotName.c_str(), STG_MSG_CRITICAL, message);

  //if(messagesModel) stg_model_lock(messagesModel);
  /*stg_message_t* msg =*/ 
  //stg_messages_send(messagesModel, (char*) robotName.c_str(), STG_MSG_CRITICAL, message);
  // can't call this from any old thread //   if(msg) while(!msg->displayed) stg_world_update(stageWorld, FALSE);
  //if(messagesModel) stg_model_unlock(messagesModel);
}

void StageInterface::warn_s(const char* message)
{
  stg_world_display_message_s(stageWorld, 0, robotName.c_str(), STG_MSG_WARNING, message);

  //if(messagesModel) stg_model_lock(messagesModel);
  //stg_message_t* msg = 
  //stg_messages_send(messagesModel, robotName.c_str(), STG_MSG_WARNING, message);
  // can't call this from any old thread // if(msg) while(!msg->displayed) stg_world_update(stageWorld, FALSE);
  //if(messagesModel) stg_model_unlock(messagesModel);
  //ArUtil::sleep(10); // hack to work around messages model habit of clobering old messages with new ones.
}

void StageInterface::inform_s(const char* message)
{
  stg_world_display_message_s(stageWorld, 0, robotName.c_str(), STG_MSG_INFORMATION, message);

  //if(messagesModel) stg_model_lock(messagesModel);
  //stg_message_t* msg = 
  //stg_messages_send(messagesModel, robotName.c_str(), STG_MSG_INFORMATION, message);
  // can't call this from any old thread //  if(msg) while(!msg->displayed) stg_world_update(stageWorld, FALSE);
  //if(messagesModel) stg_model_unlock(messagesModel);
}


void StageInterface::log_s(const char* message)
{
  stg_print_msg("%s: %s", robotName.c_str(), message);
}

void StageInterface::log_error_s(const char* message)
{
  stg_print_error("%s: %s", robotName.c_str(), message);
  stg_flush_log_file();
}

void StageInterface::shutdown(int errorcode)
{
  stg_quit_request_code(errorcode + 1); // 1 means to quit normally, >1 for an error
}

std::vector< RobotInterface::DeviceInfo > StageInterface::getDeviceInfo()
{
  std::vector<RobotInterface::DeviceInfo> devs;
  GPtrArray* models = stg_model_get_children_ptr(positionModel);
  for(size_t i = 0; i < models->len; i++)
  {
    stg_model_t* m = (stg_model_t*)g_ptr_array_index(models, i);
    if(m)
    {
      DeviceInfo inf;
      inf.name = stg_model_get_token(m);
      inf.basetype = stg_model_get_base_type_name(m);
      inf.type = stg_model_get_instance_type_name(m);
      inf.which = stg_model_get_instance_index(m);
      inf.status = 0;
      devs.push_back(inf);
    }
  }
  return devs;
}

void StageInterface::logState()
{
  RobotInterface::logState();

  stg_world_log_stats(stageWorld);

  log("%lu zero-interval warnings so far, %lu 10%%-too-long warnings.", 
    (unsigned long) stg_world_num_zero_interval_warnings(stageWorld),
    (unsigned long) stg_world_num_interval_too_long_warnings(stageWorld)
  );

  stg_position_cmd_t *cmd = (stg_position_cmd_t*) stg_model_get_property(positionModel, "position_cmd", NULL);
  log("Current command: x=%dmm/s y=%dmm/s th=%ddeg/s, transmode=%s, rotmode=%s", (int)(cmd->x / 1000.0), (int)(cmd->y / 1000.0), (int)RTOD(cmd->a ),
    stg_position_control_mode_name(cmd->transmode),
    stg_position_control_mode_name(cmd->rotmode)
  );
  char *rel_active_x  = (char*) stg_model_get_property(positionModel, "position_rel_ctrl_active_X", NULL);
  char *rel_active_a  = (char*) stg_model_get_property(positionModel, "position_rel_ctrl_active_A", NULL);
  stg_pose_t *prog = (stg_pose_t*) stg_model_get_property(positionModel, "position_rel_ctrl_progress", NULL);
  log(" relctrl_active_x=%d, relctrl_active_a=%d, progress_x=%d mm, progress_a=%d deg.",
    *rel_active_x, *rel_active_a, (int)(prog->x / 1000.0), (int)RTOD(prog->a) );
  stg_flush_log_file();
}

bool StageInterface::haveStateOfCharge()
{
  if(!params) return false;
  return (params->BatteryType == 2);
}

void StageInterface::updateStateOfCharge()
{
  if(!haveStateOfCharge())
    return;

  size_t len = 0;
  stg_batt_soc_t* cur_soc_ptr = (stg_batt_soc_t*) stg_model_get_property( positionModel, "battery_soc", &len);
  if(cur_soc_ptr != NULL && len != 0)
  {
    stg_batt_soc_t cur_soc = (*cur_soc_ptr);
    setStateOfCharge((float)cur_soc);
  }
}

void StageInterface::setInvisible(bool s)
{
  int r = s?0:1;
  stg_model_lock(positionModel);
  stg_model_set_all_child_properties(positionModel, "laser_return", &r, sizeof(r));
  stg_model_set_all_child_properties(positionModel, "ranger_return", &r, sizeof(r));
  stg_model_unlock(positionModel);
}

void StageInterface::setEphemeral(bool s)
{
  int r = s?0:1;
  stg_model_set_all_child_properties(positionModel, "obstacle_return", &r, sizeof(r));
}

/// Initializing a robot model via TCP
void StageInterface::configPosition(ArRobotPacket *pkt)
{
  // The macro file only listed these attributes on the base unit (poineer),
  //   so I'm assming they should be true for all robot bodes
  int val = 0;
  val = 1;  // TRUE
  stg_model_set_property( positionModel, "nose", &val, sizeof(val) ); // for "gui_nose"
  //val = 0;  // FALSE
  //stg_model_set_property( positionModel, "boundary", &val, sizeof(val) ); // for "gui_boundary" // seems unsupported
  stg_obstacle_return_t obs = 1;
  stg_model_set_property( positionModel, "obstacle_return", &obs, sizeof(obs));
  stg_laser_return_t lsr = 1;
  stg_model_set_property( positionModel, "laser_return", &lsr, sizeof(lsr));
  stg_ranger_return_t rng = 1;
  stg_model_set_property( positionModel, "ranger_return", &rng, sizeof(rng));
  stg_blob_return_t blb = 1;
  stg_model_set_property( positionModel, "blob_return", &blb, sizeof(blb)); // seemed improperly labeled in PioneerRobotModels.world.inc
  stg_fiducial_return_t fid = 2;
  stg_model_set_property( positionModel, "fiducial_return", &fid, sizeof(fid) );  // may be disabled by #define

  // This is only listed on the "pioneer". I'm gussing this should stay the same for all robots
  stg_position_data_t* data = (stg_position_data_t*) stg_model_get_property_fixed( positionModel, "position_data", sizeof(stg_position_data_t));
  assert( data );
  data->localization = STG_POSITION_LOCALIZATION_ODOM;  // for localization = "odom"

  // This is only listed on the "pioneer". I'm gussing this should stay
  //   the same for all robots. It's hard coded in Aram for commercial.
  {
    stg_pose_t gpose;
    double cosa, sina, dx, dy;

    data->origin.x = ((double) pkt->bufToByte2())/10000.0;  // 4-decimal place precision between 0.0-1.0
    data->origin.y = ((double) pkt->bufToByte2())/10000.0;
    data->origin.a = ((double) pkt->bufToByte2())/10000.0;

    // compute our localization pose based on the origin and true pose
    stg_model_get_global_pose( positionModel, &gpose );

    data->pose.a = NORMALIZE( gpose.a - data->origin.a );
    STG_SINCOS(data->pose.a, sina, cosa);
    dx = gpose.x - data->origin.x;
    dy = gpose.y - data->origin.y;
    data->pose.x = dx * cosa + dy * sina;
    data->pose.y = dy * cosa - dx * sina;

    // zero position error: assume we know exactly where we are on startup
    memset( &data->pose_error, 0, sizeof(data->pose_error));
  }

  // This is only listed on the "pioneer". I'm gussing this should stay
  //   the same for all robots. It's hard coded in Aram for commercial.
  data->integration_error.x = ((double) pkt->bufToByte2())/10000.0;  // 4-decimal place precision between 0.0-1.0
  data->integration_error.y = ((double) pkt->bufToByte2())/10000.0;
  data->integration_error.a = ((double) pkt->bufToByte2())/10000.0;

  stg_model_property_changed( positionModel, "position_data" );

  // Get the robot's shape from the ArRobotPacket
  std::vector< std::pair<int,int> > shapePoints;
  int numShapePoints = (int) pkt->bufToByte();
  for(int i = 0; i < numShapePoints; ++i)
  {
    int x = pkt->bufToByte2();
    int y = pkt->bufToByte2();
    shapePoints.push_back(std::make_pair(x, y));
  }

  // Set all polygon points
  // While doing this, record the min/max x&y, because their dumb normalizing needs to be reversed using the 'size' to scale
  stg_polygon_t* polys = NULL;
  int min_x, max_x, min_y, max_y;
  for(unsigned int i = 0; i < shapePoints.size(); ++i)
  {
    if (i == 0)
    {
      min_x = max_x = shapePoints[i].first;
      min_y = max_y = shapePoints[i].second;
      polys = stg_polygons_create( 1 );
    }
    else
    {
      if (shapePoints[i].first < min_x)
        min_x = shapePoints[i].first;
      else if(shapePoints[i].first > max_x)
        max_x = shapePoints[i].first;
      if (shapePoints[i].second < min_y)
        min_y = shapePoints[i].second;
      else if(shapePoints[i].second > max_y)
        max_y = shapePoints[i].second;
    }
  }

  stg_size_t size;          // for size [0.64 0.48]  // lynx
  size.x = ((double) (max_x - min_x)) / 1000.0;
  size.y = ((double) (max_y - min_y)) / 1000.0;
  //ArLog::log(ArLog::Normal, "size: x: %.2f y: %.2f", size.x, size.y );
  stg_model_set_size(positionModel, size);

  for(unsigned int i = 0; i < shapePoints.size(); ++i)
  {
    // append the point to the polygon
    stg_point_t pt;
    //snprintf(key, sizeof(key), "polygon[%d].point[%d]", l, p );
    pt.x = ((double)shapePoints[i].first)/1000.0; //wf_read_tuple_length(id, key, 0, 0);
    pt.y = ((double)shapePoints[i].second)/1000.0; //wf_read_tuple_length(id, key, 1, 0);
    //ArLog::log(ArLog::Normal, "point %d x: %.2f y: %.2f", i, pt.x, pt.y );
    stg_polygon_append_points( &polys[0], &pt, 1 );
  }
  stg_model_set_scaling(positionModel, false);
  stg_model_init_polygons(positionModel, polys, 1);

  // Load the drive type. Currently only Differential and Omni-Directional
  //   are supported. Others may include Mecanum, Trike, Ackerman, etc
  char driveBuff[256];
  pkt->bufToStr((char*)driveBuff,255);
  stg_position_drive_mode_t drive = STG_POSITION_DRIVE_DIFFERENTIAL;  // This is the default drive
  if (strcmp(driveBuff, "omni") == 0)
    drive = STG_POSITION_DRIVE_OMNI;
  stg_model_set_property( positionModel, "position_drive", &drive, sizeof(drive));

  // Get/Set the robot's custom conversion params
  this->params->Vel2DivFactor =   (double) pkt->bufToByte2();           // for "pioneer_vel2div"
  this->params->VelConvFactor =   ((double) pkt->bufToByte8())/1000000; // for "pioneer_velconv"
  this->params->RangeConvFactor = ((double) pkt->bufToByte8())/1000000; // for "pioneer_rangeconv"
  this->params->DiffConvFactor =  ((double) pkt->bufToByte8())/1000000; // for "pioneer_diffconv" // OBSOLETE? Ignored on this side in commercial runs
  this->params->DistConvFactor =  ((double) pkt->bufToByte8())/1000000; // for "pioneer_distconv"
  this->params->AngleConvFactor = ((double) pkt->bufToByte8())/1000000; // for "pioneer_angleconv"
  this->params->BatteryType =     pkt->bufToByte();                     // for "pioneer_batterytype"      2  // This will always be SoC battery for commercial
  pkt->bufToStr(this->params->RobotSubclass, ROBOT_IDENTIFIER_LEN);     // for pioneer_robot_subtype "mt400"   // TODO: Where do I get this on the robot side?
  //ArLog::log(ArLog::Normal, "RobotSubclass:   %s", this->params->RobotSubclass);   // for "pioneer_robot_subtype"   this->params->Vel2DivFactor;

  // The robot model's color (this should be irrelevant to the robot itself,
  //   since colors in MP are set by MP, whatever MobileSim is doing doesn't matter at all)
  stg_color_t col = stg_lookup_color( "grey" );             // for color "grey"
  stg_model_set_property( positionModel, "color", &col, sizeof(col));
}


void StageInterface::configPositionVelVals(ArRobotPacket *pkt)
{
  // Load values from the packet
  int dataBytes = pkt->getDataLength();

  int configAMTransVel    = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getTransVelMax());
  int configAMTransVelNeg = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getTransNegVelMax());
  int configAMTransAccel  = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getTransAccel());
  int configAMTransDecel  = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getTransDecel());

  int configAMRotVel      = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getRotVelMax());
  int configAMRotAccel    = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getRotAccel());
  int configAmRotDecel    = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getRotDecel());

  int configAMLatVel      = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getLatVelMax());
  int configAMLatAccel    = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getLatAccel());
  int configAMLatDecel    = pkt->bufToByte2();  // from p.byte2ToBuf((int)myRobot->getLatDecel());

  // speed limits
  {
    stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*) stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
    assert(speed_cfg);

    ArLog::log(ArLog::Normal, "configAMTransVel    = %d,\nconfigAMTransVelNeg = %d,\nconfigAMTransAccel  = %d,\nconfigAMTransDecel  = %d,\nconfigAMRotVel      = %d,\nconfigAMRotAccel    = %d,\nconfigAmRotDecel    = %d,\nconfigAMLatVel    = %d,\nconfigAMLatAccel  = %d,\nconfigAMLatDecel  = %d;",
        configAMTransVel,configAMTransVelNeg,configAMTransAccel,configAMTransDecel,configAMRotVel,configAMRotAccel,configAmRotDecel,configAMLatVel,configAMLatAccel,configAMLatDecel);

    speed_cfg->max_speed.x = speed_cfg->default_speed.x = ((double)configAMTransVel)/1000.0;
    speed_cfg->max_speed.y = speed_cfg->default_speed.y = ((double)configAMLatVel)/1000.0;
    speed_cfg->max_speed.a = speed_cfg->default_speed.a = DTOR((double)configAMRotVel);

    speed_cfg->max_accel.x = speed_cfg->current_accel.x = ((double)configAMTransAccel)/1000.0;
    speed_cfg->max_accel.y = speed_cfg->current_accel.y = ((double)configAMLatAccel)/1000.0;
    speed_cfg->max_accel.a = speed_cfg->current_accel.a = DTOR((double)configAMRotAccel);

    speed_cfg->max_decel.x = speed_cfg->current_decel.x = ((double)configAMTransDecel)/1000.0;
    speed_cfg->max_decel.y = speed_cfg->current_decel.y = ((double)configAMLatDecel)/1000.0;
    speed_cfg->max_decel.a = speed_cfg->current_decel.a = DTOR((double)configAmRotDecel);
#ifdef DEBUG_PROPERTY_LOAD
    printf("DEBUG loaded position_speed_config property from world file: max speed=[%g,%g,%g] max accel=[%g,%g,%g] "
        "max decel=[%g,%g,%g]; current accel=[%g,%g,%g] decel=[%g,%g,%g]; "
        "default speed=[%g,%g,%g]\n",
        speed_cfg->max_speed.x, speed_cfg->max_speed.y, speed_cfg->max_speed.a,
        speed_cfg->max_accel.x, speed_cfg->max_accel.y, speed_cfg->max_accel.a,
        speed_cfg->max_decel.x, speed_cfg->max_decel.y, speed_cfg->max_decel.a,
        speed_cfg->current_accel.x, speed_cfg->current_accel.y, speed_cfg->current_accel.a,
        speed_cfg->current_decel.x, speed_cfg->current_decel.y, speed_cfg->current_decel.a,
        speed_cfg->default_speed.x, speed_cfg->default_speed.y, speed_cfg->default_speed.a
      );
#endif
    stg_model_property_changed(positionModel, "property_speed_config"); // ? this was the old line. is this wrong?
    stg_model_property_changed(positionModel, "position_speed_config");
  }
}


void StageInterface::configPositionVelMaxVals(ArRobotPacket *pkt)
{
  // Load values from the packet
  int dataBytes = pkt->getDataLength();

  int configAMTransVel    = 0,
      configAMTransVelNeg = 0,
      configAMTransAccel  = 0,
      configAMTransDecel  = 0,
      configAMRotVel      = 0,
      configAMRotAccel    = 0,
      configAmRotDecel    = 0,
      configAMLatVel    = 0,
      configAMLatAccel  = 0,
      configAMLatDecel  = 0;

  configAMTransVel    = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMTransVel);
  configAMTransVelNeg = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMTransNegVel);
  configAMTransAccel  = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMTransAccel);
  configAMTransDecel  = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMTransDecel);

  configAMRotVel      = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMRotVel);
  configAMRotAccel    = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMRotAccel);
  configAmRotDecel    = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMRotDecel);

  if(dataBytes/2 > 7) // from if (myRobot->hasLatVel())
  {
    configAMLatVel        = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMLatVel);
    configAMLatAccel      = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMLatAccel);
    configAMLatDecel      = pkt->bufToByte2();  // from sending.byte2ToBuf(myConfigAMLatDecel);
  }

  // speed limits
  {
    stg_position_speed_config_t* speed_cfg = (stg_position_speed_config_t*) stg_model_get_property_fixed(positionModel, "position_speed_config", sizeof(stg_position_speed_config_t));
    assert(speed_cfg);

    //ArLog::log(ArLog::Normal, "configAMTransVel    = %d,\nconfigAMTransVelNeg = %d,\nconfigAMTransAccel  = %d,\nconfigAMTransDecel  = %d,\nconfigAMRotVel      = %d,\nconfigAMRotAccel    = %d,\nconfigAmRotDecel    = %d,\nconfigAMLatVel    = %d,\nconfigAMLatAccel  = %d,\nconfigAMLatDecel  = %d;",
    //    configAMTransVel,configAMTransVelNeg,configAMTransAccel,configAMTransDecel,configAMRotVel,configAMRotAccel,configAmRotDecel,configAMLatVel,configAMLatAccel,configAMLatDecel);

    speed_cfg->max_speed.x = ((double)configAMTransVel)/1000.0;
    speed_cfg->max_speed.y = ((double)configAMLatVel)/1000.0;
    speed_cfg->max_speed.a = DTOR((double)configAMRotVel);

    speed_cfg->max_accel.x = ((double)configAMTransAccel)/1000.0;
    speed_cfg->max_accel.y = ((double)configAMLatAccel)/1000.0;
    speed_cfg->max_accel.a = DTOR((double)configAMRotAccel);

    speed_cfg->max_decel.x = ((double)configAMTransDecel)/1000.0;
    speed_cfg->max_decel.y = ((double)configAMLatDecel)/1000.0;
    speed_cfg->max_decel.a = DTOR((double)configAmRotDecel);

#ifdef DEBUG_PROPERTY_LOAD
    printf("DEBUG loaded position_speed_config property from world file: max speed=[%g,%g,%g] max accel=[%g,%g,%g] "
        "max decel=[%g,%g,%g]; current accel=[%g,%g,%g] decel=[%g,%g,%g]; "
        "default speed=[%g,%g,%g]\n",
        speed_cfg->max_speed.x, speed_cfg->max_speed.y, speed_cfg->max_speed.a,
        speed_cfg->max_accel.x, speed_cfg->max_accel.y, speed_cfg->max_accel.a,
        speed_cfg->max_decel.x, speed_cfg->max_decel.y, speed_cfg->max_decel.a,
        speed_cfg->current_accel.x, speed_cfg->current_accel.y, speed_cfg->current_accel.a,
        speed_cfg->current_decel.x, speed_cfg->current_decel.y, speed_cfg->current_decel.a,
        speed_cfg->default_speed.x, speed_cfg->default_speed.y, speed_cfg->default_speed.a
      );
#endif
    stg_model_property_changed(positionModel, "property_speed_config"); // ? this was the old line. is this wrong?
    stg_model_property_changed(positionModel, "position_speed_config");
  }
}

void StageInterface::addLaser()
{
  // Laser model
  stg_model_t* new_laser_model = stg_world_new_model_ex(stageWorld, "laser", NULL, positionModel, -1, FALSE);
  if(new_laser_model)
    lasers.push_back(Laser(lasers.size(), new_laser_model));
  else
  {
    if(!stg_world_get_quiet(stageWorld))
      ArLog::log(ArLog::Normal, "StageInterface::addLaser(): Failed to add laser to this model");
  }
}

void StageInterface::configLaser(ArRobotPacket *pkt)
{
  int whichLaser = pkt->bufToByte();  // Which laser we're talking about

  if(lasers.size() < (whichLaser+1))
  {
    if(!stg_world_get_quiet(stageWorld))
      ArLog::log(ArLog::Normal, "Laser number %d is out of range. There are currently %d laser models", whichLaser, lasers.size());
    return;
  }

  stg_model_t *laserModel = lasers[whichLaser].stageModel;
  if(laserModel == NULL)
  {
    if(!stg_world_get_quiet(stageWorld))
      ArLog::log(ArLog::Normal, "Laser number %d is an invalid laser. The stageModel value == NULL", whichLaser);
    return;
  }

  // Laser model
  if(laserModel) {

    stg_laser_config_t* lasercfg = (stg_laser_config_t*)stg_model_get_property_fixed(laserModel, "laser_cfg", sizeof(stg_laser_config_t));
    assert(lasercfg);

    // Set the laser's position
    stg_pose_t pose;
    memset( &pose, 0, sizeof(pose));
    pose.x = ((double)pkt->bufToByte2())/1000.0; // LaserX  (in meters)
    pose.y = ((double)pkt->bufToByte2())/1000.0; // LaserY  (in meters)
    pose.a = DTOR((double)pkt->bufToByte2());    // LaserTh (in radians)
    stg_model_set_property( laserModel, "pose", &pose, sizeof(pose) );
    stg_model_set_property( laserModel, "initial_pose", &pose, sizeof(pose) );
    lasercfg->reverse_scan = pkt->bufToByte();   // Reverse Scan
    lasercfg->reverse_scan = 0; // TODO: THE ABOVE 'true' REVERSE SCAN IS INCORRECT! THE ONLY REASON IT EVER WORKED WAS THAT IT WAS OVERWRITTEN TO 'false' BY ANOTHER INCOMING PACKET BEFORE ANY UPDATES EXECUTED

    // range_min is set to 0, then Aram ignores all readings
    //   within a certain clipping distance (inside robot).
    lasercfg->range_min = 0.0;
    // range_max will be either 8, 16, or 32 meters from the
    //   laser on Aram. Actually using this value results in
    //   an arc of ghost readings at the max range if there
    //   is no closer laser return. Adding the magic number
    //   '0.01' seems to fix this.
    unsigned int max_range = pkt->bufToByte4();
    lasercfg->range_max = (((double)max_range)/1000.0) + 0.01;
    // fov will be determined by the startAngle and endAngle
    //   sent from Aram. Stage only handles symmetrical lasers
    double laserStart, laserEnd, increment;
    laserStart = (double)pkt->bufToByte2();    // laserStart (in degrees)
    laserEnd   = (double)pkt->bufToByte2();    // laserEnd   (in degrees)
    increment  = ((double)pkt->bufToByte2())/100.0;    // increment  (in degrees, multiplied then divided by 100 for resolution < 1deg)
    if(laserEnd < laserStart)
      lasercfg->reverse_scan = true; // TODO: Is this necessary? We've seen above that it malfunctions if true
    lasercfg->reverse_scan = 0; // TODO: THE ABOVE 'true' REVERSE SCAN IS INCORRECT! THE ONLY REASON IT EVER WORKED WAS THAT IT WAS OVERWRITTEN TO 'false' BY ANOTHER INCOMING PACKET BEFORE ANY UPDATES EXECUTED
    setLaserFOV(0, fabs(laserEnd - laserStart) );
    setLaserResolution(0, increment);

    // It seems all laser models have these sensor return values
    stg_laser_return_t lsr = 1;
    stg_model_set_property( laserModel, "laser_return", &lsr, sizeof(lsr));
    stg_ranger_return_t rng = 1;
    stg_model_set_property( laserModel, "ranger_return", &rng, sizeof(rng));
    stg_blob_return_t blb = 0;
    stg_model_set_property( laserModel, "blob_return", &blb, sizeof(blb)); // seemed improperly labeled in PioneerRobotModels.world.inc
    stg_fiducial_return_t fid = 0;
    stg_model_set_property( laserModel, "fiducial_return", &fid, sizeof(fid) );  // may be disabled by #define

    // Get the laser's model name (we may remove this later if it's uneccessary)
    char modelBuff[256];
    pkt->bufToStr((char*)modelBuff,255);

    // Load the laser model's parameters
    if(strcmp(modelBuff, "lms200") == 0)
    {
      stg_color_t color = stg_lookup_color("LightBlue");    // for color   LightBlue
      stg_model_set_property( laserModel, "color", &color, sizeof(color));
      stg_size_t size;    // for size  [0.155 0.15]
      size.x = 0.155;
      size.y = 0.15;
      stg_model_set_size(laserModel, size);
#ifdef ENABLE_LASER_NOISE
      lasercfg->noise = 0.005;                // for noise   0.005
      lasercfg->reading_angle_error = 0.0007; // for reading_angle_error   0.0007
#else
      lasercfg->noise = 0.0;
      lasercfg->reading_angle_error = 0.0;
#endif
      stg_meters_t lbh = 0.115; // for laser_beam_height 0.115
      stg_model_set_property(laserModel, "laser_beam_height", &lbh, sizeof(lbh));  // disabled by comment (laser_raytrace_match_height is never called)
      stg_meters_t val = -0.2; // for height_offset -0.2
      stg_model_set_property(laserModel, "height_offset", &val, sizeof(val));      // disabled by comment (laser_raytrace_match_height is never called)


      // Laser rules, for converting one return value into another
      stg_laser_rule_t *prev_rule = lasercfg->rules;

      // The rule to # Change any reflector value greater than 1 into just 1, if it's more than 30 meters away:
      stg_laser_rule_t* new_rule = (stg_laser_rule_t*)malloc(sizeof(stg_laser_rule_t));
      assert(new_rule);
      new_rule->next = NULL;

      new_rule->model_value_eq = -1;
      new_rule->model_value_gt = 1;
      new_rule->model_value_lt = -1;

      new_rule->result = _stg_laser_rule_struct::STG_LASER_RULE_RETURN_VALUE; // STG_LASER_RULE_RETURN_VALUE;  // only result currently
      new_rule->result_value.detect = 1;
      new_rule->stop = 0;

      new_rule->condition = _stg_laser_rule_struct::STG_LASER_RULE_COND_OUTSIDE_RANGE; // STG_LASER_RULE_COND_OUTSIDE_RANGE;
      new_rule->condition_value.range = 30;

      if(prev_rule == NULL)
        lasercfg->rules = new_rule;
      else
        prev_rule->next = new_rule;
      prev_rule = new_rule;

      // The rule to # Change reflector values >1 into 1 if more than 90deg away:
      new_rule = NULL;
      new_rule = (stg_laser_rule_t*)malloc(sizeof(stg_laser_rule_t));
      assert(new_rule);
      new_rule->next = NULL;

      new_rule->model_value_eq = -1;
      new_rule->model_value_gt = 1;
      new_rule->model_value_lt = -1;

      new_rule->result = _stg_laser_rule_struct::STG_LASER_RULE_RETURN_VALUE; // STG_LASER_RULE_RETURN_VALUE;  // only result currently
      new_rule->result_value.detect = 1;
      new_rule->stop = 0;

      new_rule->condition = _stg_laser_rule_struct::STG_LASER_RULE_COND_OUTSIDE_ANGLE; // STG_LASER_RULE_COND_OUTSIDE_RANGE;
      new_rule->condition_value.angle = DTOR(90);

      if(prev_rule == NULL)
        lasercfg->rules = new_rule;
      else
        prev_rule->next = new_rule;
      prev_rule = new_rule;

      // The rule to # Change the specific reflector value 2 into 33 (which is the actual value the real SICK returns to ARIA):
      new_rule = NULL;
      new_rule = (stg_laser_rule_t*)malloc(sizeof(stg_laser_rule_t));
      assert(new_rule);
      new_rule->next = NULL;

      new_rule->model_value_eq = 2;
      new_rule->model_value_gt = -1;
      new_rule->model_value_lt = -1;

      new_rule->result = _stg_laser_rule_struct::STG_LASER_RULE_RETURN_VALUE; // STG_LASER_RULE_RETURN_VALUE;  // only result currently
      new_rule->result_value.detect = 33;
      new_rule->stop = 0;

      new_rule->condition = _stg_laser_rule_struct::STG_LASER_RULE_COND_NONE; // STG_LASER_RULE_COND_OUTSIDE_RANGE;

      if(prev_rule == NULL)
        lasercfg->rules = new_rule;
      else
        prev_rule->next = new_rule;
      prev_rule = new_rule;
    }
    else if(strcmp(modelBuff, "s300") == 0)
    {
      stg_color_t color = stg_lookup_color("yellow");    // for color   yellow
      stg_model_set_property( laserModel, "color", &color, sizeof(color));
      stg_size_t size;    // for size  [0.155 0.15]
      size.x = 0.102;
      size.y = 0.105;
      stg_model_set_size(laserModel, size);
#ifdef ENABLE_LASER_NOISE
      lasercfg->noise = 0.006;                // for noise   0.006
      lasercfg->reading_angle_error = 0.0007; // for reading_angle_error   0.0007
#else
      lasercfg->noise = 0.0;
      lasercfg->reading_angle_error = 0.0;
#endif
    }
    stg_model_property_changed(laserModel, "laser_cfg");
  }
}

void StageInterface::configSonar(ArRobotPacket *pkt)
{
  // Ranger model
  stg_model_t* new_ranger_model = stg_world_new_model_ex(stageWorld, "ranger", NULL, positionModel, -1, FALSE);
  if(new_ranger_model == NULL)
  {
    if(!stg_world_get_quiet(stageWorld))
      ArLog::log(ArLog::Normal, "StageInterface::configSonar(): Failed to add ranger to this model");
    return;
  }

  // Load the sensor-return values
  stg_laser_return_t lsr = 0;
  stg_model_set_property( new_ranger_model, "laser_return", &lsr, sizeof(lsr));
  stg_blob_return_t blb = 0;
  stg_model_set_property( new_ranger_model, "blob_return", &blb, sizeof(blb)); // seemed improperly labeled in PioneerRobotModels.world.inc
  stg_fiducial_return_t fid = 0;
  stg_model_set_property( new_ranger_model, "fiducial_return", &fid, sizeof(fid) );  // may be disabled by #define


  // Load the common settings
  stg_size_t common_size; // for ssize [0.01 0.04] native
  common_size.x = 0.01;
  common_size.y = 0.021;

  double common_min, common_max, common_fov, common_noise; // for sview [0.1 5.0 30]  native
  common_min = 0.1;
  common_max = 5.0;
  common_fov = 115;  // The spec sheet says the FOV is 115 +/- 15 degrees. Not sure this matters, considering we're using "single" projection on every simulation ranger type
#ifdef ENABLE_RANGER_NOISE
  common_noise = 0.0005;  // for noise 0.0005  native
#endif

  char proj_type[256];
  stg_ranger_projection_t projection_type;
  strcpy(proj_type, "single");  // for projection_type "single"  native
  if(strcmp(proj_type, "single") == 0)
    projection_type = STG_RANGER_SINGLE_RAY;
  else if(strcmp(proj_type, "closest") == 0)
    projection_type = STG_RANGER_CLOSEST_RAY;
  else
  {
    if(!stg_world_get_quiet(stageWorld))
      ArLog::log(ArLog::Normal, "Ranger (Sonar) configuration: unknown projection type \"%s\". Using default \"single\".", proj_type);
    projection_type = STG_RANGER_SINGLE_RAY;
  }

  double resolution, throwaway_thresh, throwaway_prob;
  int enable_throwaway;
  resolution = 6.0;        // for #projection_res     6 native (revert to default)
  enable_throwaway = 0;    // for #enable_throwaway   1 native (revert to default)
  throwaway_thresh = 0.75; // for #throwaway_thresh 0.4 native (revert to default)
  throwaway_prob = 0.4;    // for #throwaway_prob   0.8 native (revert to default)

  // Load the number of sensors
  int scount = pkt->bufToByte2(); // for scount
  stg_ranger_config_t* configs = (stg_ranger_config_t*) calloc( sizeof(stg_ranger_config_t), scount );

  // Init all sensors
  for(int i = 0; i < scount; i++)
  {
    configs[i].size.x = common_size.x;
    configs[i].size.y = common_size.y;
    configs[i].bounds_range.min = common_min;
    configs[i].bounds_range.max = common_max;
    configs[i].fov = common_fov;
    configs[i].noise = common_noise;
    configs[i].projection_type = projection_type;
    configs[i].resolution = resolution;
    configs[i].enable_throwaway = enable_throwaway;
    configs[i].throwaway_thresh = throwaway_thresh;
    configs[i].throwaway_prob = throwaway_prob;

    // Settings for each individual range sensor
    configs[i].pose.x = ((double)pkt->bufToByte2())/1000.0;       // for wf_read_tuple_length(wf_id, key, 0, 0);
    configs[i].pose.y = ((double)pkt->bufToByte2())/1000.0;       // for wf_read_tuple_length(wf_id, key, 1, 0);
    configs[i].pose.a = DTOR((double)pkt->bufToByte2());          // for wf_read_tuple_angle(wf_id, key, 2, 0);
    configs[i].bounds_range.max = ((double)pkt->bufToByte2())/1000.0;
  }

  stg_model_set_property( new_ranger_model, "ranger_cfg", configs, scount * sizeof(stg_ranger_config_t) );

  free( configs );

  sonarModel = new_ranger_model;
  openSonar();
}

void StageInterface::configBattery(ArRobotPacket *pkt)
{
  double battCapacity = ((double) pkt->bufToByte8())/100;
//  ArLog::log(ArLog::Normal, "StageInterface::configBattery(): battCapacity: %f", battCapacity);
  stg_batt_soc_t batt_capacity = (stg_batt_soc_t) battCapacity;
  stg_model_set_property(positionModel, "battery_capacity", &batt_capacity, sizeof(stg_batt_soc_t));
  stg_model_set_property(positionModel, "battery_charge", &batt_capacity, sizeof(stg_batt_soc_t));
  stg_batt_op_state_t initState = BOS_DISABLED;
  stg_model_set_property(positionModel, "battery_operation_state", &initState, sizeof(int));

  stg_charge_rate_t soc_rates;

  double chargeRateDocked = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): chargeRateDocked: %f Wh/h", chargeRateDocked);
  chargeRateDocked /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): chargeRateDocked: %f Wh/s", chargeRateDocked);
  soc_rates.docked = chargeRateDocked;

  double dischargeRateRobotHotel = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateRobotHotel: %f Wh/h", dischargeRateRobotHotel);
  dischargeRateRobotHotel /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateRobotHotel: %f Wh/s", dischargeRateRobotHotel);
  soc_rates.robothotel = dischargeRateRobotHotel;

  double dischargeRateDriving = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateDriving: %f Wh/h", dischargeRateDriving);
  dischargeRateDriving /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateDriving: %f Wh/s", dischargeRateDriving);
  soc_rates.constvel = dischargeRateDriving;

  double dischargeRateAccel = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateAccel: %f Wh/h", dischargeRateAccel);
  dischargeRateAccel /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateAccel: %f Wh/s", dischargeRateAccel);
  soc_rates.accel = dischargeRateAccel;

  double dischargeRateDecel = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateDecel: %f Wh/h", dischargeRateDecel);
  dischargeRateDecel /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRateDecel: %f Wh/s", dischargeRateDecel);
  soc_rates.decel = dischargeRateDecel;

  double dischargeRatePayloadHotel = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRatePayloadHotel: %f Wh/h", dischargeRatePayloadHotel);
  dischargeRatePayloadHotel /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRatePayloadHotel: %f Wh/s", dischargeRatePayloadHotel);
  soc_rates.payloadhotel = dischargeRatePayloadHotel;

  double dischargeRatePayloadAtGoal = ((double) pkt->bufToByte8())/100;
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRatePayloadAtGoal: %f Wh/h", dischargeRatePayloadAtGoal);
  dischargeRatePayloadAtGoal /= 3600; // Convert from Wh/h to Wh/s
  //ArLog::log(ArLog::Normal, "StageInterface::configBattery(): dischargeRatePayloadAtGoal: %f Wh/s", dischargeRatePayloadAtGoal);
  soc_rates.payloadatgoal = dischargeRatePayloadAtGoal;

  stg_model_set_property(positionModel, "battery_soc_rate", &soc_rates, sizeof(stg_charge_rate_t));
}

void StageInterface::updateBatteryChargeState(ArRobotPacket *pkt)
{
  // If we're updating the state of charge, we can assume that we have BatteryType = 2
  // TODO: move this to some other part of the position init?
  params->BatteryType = 2;

  int batt_op_state = pkt->bufToByte();
  ArLog::log(ArLog::Normal, "StageInterface::updateBatteryChargeState(): state: %d", batt_op_state);
  stg_model_set_property(positionModel, "battery_operation_state", &batt_op_state, sizeof(int));
}


float StageInterface::getSimGPSDOP()
{
  float *f = (float*) stg_model_get_property_fixed(positionModel, "pioneer_gps_dop", sizeof(float));
  if(f) return *f;
  else return 1.0;
}

double StageInterface::getSimulatorOdomErrorX()
{
  stg_position_data_t *data = (stg_position_data_t*) stg_model_get_property_fixed(positionModel, "position_data", sizeof(stg_position_data_t));
  assert(data);
  return data->integration_error.x * 1000.0;
}

double StageInterface::getSimulatorOdomErrorY()
{
  stg_position_data_t *data = (stg_position_data_t*) stg_model_get_property_fixed(positionModel, "position_data", sizeof(stg_position_data_t));
  assert(data);
  return data->integration_error.y * 1000.0;
}

double StageInterface::getSimulatorOdomErrorTh()
{
  stg_position_data_t *data = (stg_position_data_t*) stg_model_get_property_fixed(positionModel, "position_data", sizeof(stg_position_data_t));
  assert(data);
  return RTOD(data->integration_error.a);
}

