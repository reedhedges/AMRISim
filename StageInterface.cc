
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
  stageModel(model),
  subscribed(false), opened(false), 
  startAngle(-90.0), endAngle(90.0)
{
  DeviceInfo::name = stg_model_get_token(model);
  DeviceInfo::type = stg_model_get_instance_type_name(model);
  DeviceInfo::basetype = stg_model_get_base_type_name(model);
  DeviceInfo::which = (unsigned int)i;
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
  if(params) strncpy(params->RobotSubclass, robotModel.c_str(), ROBOT_IDENTIFIER_LEN);

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

  // Get the robot subtype from the position model, that's important for
  // EmulatePioneer:
  size_t len;
  const char* s = (const char*) stg_model_get_property(positionModel, "pioneer_robot_subtype", &len);
  if(params && s)
  {
    strncpy(params->RobotSubclass, s, ROBOT_IDENTIFIER_LEN);
  }

  s = (const char*) stg_model_get_property(positionModel, "pioneer_robot_type", &len);
  if(params && s)
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
  * were registered by AMRISim's main.cc): */

  if(params)
  {

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

  }

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
  p->x = (stg_meters_t)((double)x / 1000.0);
  p->y = (stg_meters_t)((double)y / 1000.0);
  p->a = (stg_meters_t) DTOR((double)theta);
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
  int numSonarReadings = (int)(len / sizeof(stg_ranger_sample_t));
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
  int numLaserReadings = (int)(len / sizeof(stg_laser_sample_t));
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
  int numLaserReadings = (int)(len / sizeof(stg_laser_sample_t));
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
  return (long)(pose->x * 1000.0);
}

long StageInterface::getSimulatorPoseY() {
  stg_pose_t* pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  assert(pose);
  return (long)(pose->y * 1000.0);
}

int StageInterface::getSimulatorPoseTheta() {
  stg_pose_t* pose = (stg_pose_t*)stg_model_get_property_fixed(positionModel, "pose", sizeof(stg_pose_t));
  assert(pose);
  return ArMath::roundInt(RTOD(pose->a));
}

int StageInterface::getLastInterval() { 
  return (int) stg_world_get_last_interval(stageWorld);
}

int StageInterface::getSimInterval() {
  return (int) stg_world_get_sim_interval(stageWorld);
}

int StageInterface::getRealInterval() {
  return (int) stg_world_get_real_interval(stageWorld);
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
      inf.which = (unsigned int) stg_model_get_instance_index(m);
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

