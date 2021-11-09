
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


#ifndef STAGE_ROBOT_FACTORY_HH_
#define STAGE_ROBOT_FACTORY_HH_

#include "AMRISim.hh"
#include "RobotFactory.hh"
#include "Aria/ariaUtil.h"

#include "stage.h"

class StageRobotFactory : public virtual RobotFactory {
public:
  StageRobotFactory(stg_world_t *world, const std::string& modelName, double
start_x, double start_y, double start_th, const AMRISim::Options *userOpts);
  StageRobotFactory(stg_world_t *world, const std::string& modelName, 
    mobilesim_get_pose_cb_t get_start_cb, 
    mobilesim_get_bounds_cb_t get_bounds_cb, bool start_outside_bounds,
    const AMRISim::Options *userOpts);
  //virtual ~StageRobotFactory() {} 
protected:
  virtual std::shared_ptr<RobotInterface> createRobot(const std::string& modelName, const std::string& requestedRobotName = "") override;
  virtual std::shared_ptr<RobotInterface> createStubRobot(const std::string& modelName, const std::string& requestedRobotName = "") override;
  virtual void log_s(const char *msg) override { stg_print_msg("Robot Factory: %s", msg); }
  virtual void log(const char *fmt, ...) override
  {
    va_list args;
    va_start(args, fmt);
    stg_print_msg_v(fmt, args);
    va_end(args);
  }
private:
  stg_world_t* myWorld;
  bool myUseFixedStartPos;
  stg_pose_t myStartPos;
  mobilesim_get_pose_cb_t *myGetStartCB;
  mobilesim_get_bounds_cb_t *myGetBoundsCB;
  bool myStartOutsideBounds;
};


#endif
