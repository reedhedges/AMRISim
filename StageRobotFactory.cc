/*  
    (C) Copyright 2005, ActivMedia Robotics LLC <http://www.activmedia.com>
    (C) Copyright 2006-2010 MobileRobots, Inc. <http://www.mobilerobots.com>
    (C) Copyright 2011-2015 Adept Technology
    (C) Copyright 2016-2017 Omron Adept Technologies

    This is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include "MobileSim.hh"
#include "StageRobotFactory.hh"
#include "StageInterface.hh"

StageRobotFactory::StageRobotFactory(stg_world_t* world, const std::string& modelName, 
  double start_x, double start_y, double start_th, 
  const MobileSim::Options *userOpts) 
:
  RobotFactory(modelName, userOpts),
  myWorld(world),
  myUseFixedStartPos(true)
{
  myStartPos.x = start_x / 1000.0;
  myStartPos.y = start_y / 1000.0;
  myStartPos.a = start_th / 1000.0;
}

StageRobotFactory::StageRobotFactory(stg_world_t *world, const std::string& modelName, 
  mobilesim_get_pose_cb_t *get_start_cb,
  mobilesim_get_bounds_cb_t *get_bounds_cb, bool start_outside_bounds,
  const MobileSim::Options *userOpts)
:
  RobotFactory(modelName, userOpts),
  myWorld(world),
  myUseFixedStartPos(false),
  myGetStartCB(get_start_cb),
  myGetBoundsCB(get_bounds_cb),
  myStartOutsideBounds(start_outside_bounds)
{
}

RobotInterface* StageRobotFactory::createRobot(const std::string& modelName, const std::string& requestedRobotName)
{
  //stg_world_lock(myWorld);
  stg_model_t* model;
  if(requestedRobotName == "") 
  {
    stg_print_msg("StageRobotFactory: Creating robot of type %s...", modelName.c_str());
    model = stg_world_new_model(myWorld, (char*)modelName.c_str(), NULL, NULL);
  }
  else
  {
    stg_print_msg("StageRobotFactory: Creating robot of type %s with name %s...", modelName.c_str(), requestedRobotName.c_str());
    model = stg_world_new_model(myWorld, (char*)modelName.c_str(), NULL, (char*) requestedRobotName.c_str()); 
  }
  //print_debug("created new position model 0x%x", model);
    // cast should be safe, new_model should be making copies of those strings 
  if(!model) {
    stg_print_error("StageRobotFactory: stage could not create a model of type \"%s\".", modelName.c_str());
    return NULL;
  }

  //if(!stg_model_lock(model)) return NULL; // destroyed somehow
  stg_model_set_property(model, "source", (void*)"StageRobotFactory", strlen("StageRobotFactory"));
  std::string name = stg_model_get_token(model);
  if(myUseFixedStartPos)
  {
    stg_print_msg("StageRobotFactory: New robot will have starting position (fixed) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }
  else if(myStartOutsideBounds && myGetBoundsCB)
  {
    double min_x, max_y;
    myGetBoundsCB(&min_x, NULL, NULL, &max_y);
    myStartPos.x = (min_x / 1000.0) - 2.0;
    myStartPos.y = (max_y / 1000.0);
    myStartPos.a = 0;
    //stg_print_msg("Robot Factory: New robot will have starting position (outside map bounds) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }
  else if(myGetStartCB)
  {
    myGetStartCB(&myStartPos.x, &myStartPos.y, &myStartPos.a);
    myStartPos.x /= 1000.0;
    myStartPos.y /= 1000.0;
    myStartPos.a = DTOR(myStartPos.a);
    stg_print_msg("StageRobotFactory: New robot will have starting position (obtained) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }

  // add a messages model (for gui messages)
  //stg_model_t *msgs = stg_world_new_model(myWorld, "messages", model, NULL);
  //assert(msgs);
  //if(!stg_model_lock(msgs)) return NULL;  // messages model destroyed (somehow!)
  //stg_messages_send(msgs, NULL, STG_MSG_INFORMATION, "Robot factory for model type %s created a new robot (%s).", modelName.c_str(), name.c_str());
  //stg_model_unlock(msgs);
  

  // TODO: test. does this cause the crash?
  //stg_world_display_message(myWorld, 0, NULL, STG_MSG_INFORMATION, "Robot factory for model type %s created a new robot (%s).", modelName.c_str(), name.c_str());

  //stg_model_unlock(model);
  //stg_world_unlock(myWorld);
  StageInterface *retInterface = new StageInterface(myWorld, model, modelName, name);

  return retInterface;
}


RobotInterface* StageRobotFactory::createStubRobot(const std::string& modelName, const std::string& requestedRobotName)
{
  //stg_world_lock(myWorld);
  stg_model_t* model;
  if(requestedRobotName == "")
  {
    stg_print_msg("StageRobotFactory: Creating robot of type %s...", "position");
    model = stg_world_new_model(myWorld, "position", NULL, NULL);
  }
  else
  {
    stg_print_msg("StageRobotFactory: Creating robot of type %s with name %s...", "position", requestedRobotName.c_str());
    model = stg_world_new_model(myWorld, "position", NULL, (char*) requestedRobotName.c_str());
  }
  //print_debug("created new position model 0x%x", model);
    // cast should be safe, new_model should be making copies of those strings
  if(!model) {
    stg_print_error("StageRobotFactory: stage could not create a model of type \"%s\".", "position");
    return NULL;
  }

  //if(!stg_model_lock(model)) return NULL; // destroyed somehow
  stg_model_set_property(model, "source", (void*)"StageRobotFactory", strlen("StageRobotFactory"));
  std::string name = stg_model_get_token(model);
  if(myUseFixedStartPos)
  {
    stg_print_msg("StageRobotFactory: New robot will have starting position (fixed) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }
  else if(myStartOutsideBounds && myGetBoundsCB)
  {
    double min_x, max_y;
    myGetBoundsCB(&min_x, NULL, NULL, &max_y);
    myStartPos.x = (min_x / 1000.0) - 2.0;
    myStartPos.y = (max_y / 1000.0);
    myStartPos.a = 0;
    //stg_print_msg("Robot Factory: New robot will have starting position (outside map bounds) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }
  else if(myGetStartCB)
  {
    myGetStartCB(&myStartPos.x, &myStartPos.y, &myStartPos.a);
    myStartPos.x /= 1000.0;
    myStartPos.y /= 1000.0;
    myStartPos.a = DTOR(myStartPos.a);
    stg_print_msg("StageRobotFactory: New robot will have starting position (obtained) %f,%f,%f.", myStartPos.x, myStartPos.y, myStartPos.a);
    stg_model_set_pose(model, myStartPos);
  }

  // add a messages model (for gui messages)
  //stg_model_t *msgs = stg_world_new_model(myWorld, "messages", model, NULL);
  //assert(msgs);
  //if(!stg_model_lock(msgs)) return NULL;  // messages model destroyed (somehow!)
  //stg_messages_send(msgs, NULL, STG_MSG_INFORMATION, "Robot factory for model type %s created a new robot (%s).", modelName.c_str(), name.c_str());
  //stg_model_unlock(msgs);


  // TODO: test. does this cause the crash?
  //stg_world_display_message(myWorld, 0, NULL, STG_MSG_INFORMATION, "Robot factory for model type %s created a new robot (%s).", modelName.c_str(), name.c_str());

  //stg_model_unlock(model);
  //stg_world_unlock(myWorld);
  StageInterface *retInterface = new StageInterface(myWorld, model, "position", name);

  return retInterface;
}
