
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

#ifndef EP_STAGE_INTERFACE_HH_
#define EP_STAGE_INTERFACE_HH_

#include "RobotInterface.hh"
#include "util.h"

#include <string>
#include <set>
#include <deque>
#include <map>

#include "stage.h"

#define MAX_SONAR_SAMPLES 128
#define MAX_LASER_SAMPLES (361*2)



/** Provides access to Stage commands and data
 */
class StageInterface final : public virtual RobotInterface
{
  public:
    StageInterface(stg_world_t* _world, const std::string& _robotModel, const std::string& _robotName);
    StageInterface(stg_world_t* _world, stg_model_t* _model, const std::string& _robotModel, const std::string& _robotName);
    virtual void connect(RobotParams* params) override;
    virtual void disconnect() override;
    void lockStageWorld() {
#ifdef STAGE_MUTEX_LOG
      printf("StageInterface::lockStage()... ");
#endif
      stg_world_lock(stageWorld);
      //stageWorldMutex.lock();
    }
    void unlockStageWorld() {
#ifdef STAGE_MUTEX_LOG
      printf("StageInterface::unlockStage()... ");
#endif
      stg_world_unlock(stageWorld);
      //stageWorldMutex.unlock();

    }

    virtual ~StageInterface();

    virtual void logState() override;

  protected:

    RobotParams* params;
    stg_world_t* stageWorld;
    std::string robotModel;
    std::string robotName;

    stg_model_t* positionModel;
    stg_model_t* sonarModel;

  public:
    class Laser : public RobotInterface::DeviceInfo {
    public:
      stg_model_t *stageModel;
      bool subscribed, opened;
      // these are set for real in connect(), or by commands:
      double startAngle, endAngle;
      Laser(size_t i, stg_model_t* model);
      ~Laser(); ///< does not remove or destroy stage models. see destroy().
      void destroy(); ///< remove stage models from world, destroy stage models and other internal objects
      void open();
      void close();
      void setResolution(double inc);
      double getResolution();
      void setFOV(double fov);
      double getFOV();
      void setAngles(double start, double end);
      size_t numReadings();
      unsigned int getReading(size_t i);
      [[deprecated]] unsigned int getReading(int i) { assert(i >= 0); return getReading((size_t)i); }
      int getReflectance(int i);
      size_t forEachReading(LaserReadingFunc &func, const size_t &start);
    };
    std::vector<Laser> lasers;

  protected:
    stg_position_cmd_t positionCmd;

    //StageMapLoader mapLoader;

    /** @groupname Utilities for accessing Stage */
    //@{
    
  private:
    //static ArMutex stageWorldMutex; ///< Mutex for calls to Stage affecting the world (models can be individually locked)

  protected:
    /// easier accossor than stage's property interface. does NOT lock stage mutex. may reurn NULL
    stg_position_data_t* stagePositionData();
    /// easier accossor than stage's property interface. does NOT lock stage mutex. may return NULL
    stg_velocity_t* stageVelocityData();
    //@}


    /* Implement the RobotInterface for stage: */

  private:
    bool subscribedToSonar, openedSonar;
    bool areMotorsEnabled;

  public:
    virtual void openSonar() override;
    virtual void closeSonar() override;
    virtual bool sonarOpen() override { return subscribedToSonar && openedSonar; }
    virtual void openLaser(size_t i) override;
    virtual void closeLaser(size_t i) override;
    virtual bool laserOpen(size_t i) override { return i < lasers.size() && lasers[i].opened; }
    virtual void enableMotors(bool e) override;
    virtual void transVel(int v) override;
    virtual void latVel(int v) override;
    virtual void rotVel(int v) override;
    virtual void move(int m) override;
    virtual void heading(int h) override;
    virtual void deltaHeading(int h) override;
    virtual void stop() override;
    virtual void stop(int transdecel, int rotdecel) override;
    virtual void setAccel(int a) override;
    virtual void setDecel(int d) override;
    virtual void setRotAccel(int a) override;
    virtual void setRotDecel(int d) override;
    virtual void setLatAccel(int a) override;
    virtual void setLatDecel(int d) override;
    virtual void setMaxVel(int v) override ;
    virtual void setMaxRotVel(int v) override;
    virtual void setDefaultVel(int v) override;
    virtual void setDefaultRotVel(int v) override;
    virtual void setOdom(int x, int y, int theta) override;
    virtual void setSimulatorPose(long int x, long int y, long int z, int theta) override;
    virtual void setSimulatorPose(const Pose& pose) override;
    virtual void resetSimulatorPose() override;
    virtual void stall(bool stalled) override;

    // data:
    virtual bool havePositionData() override;
    virtual bool haveGripper() override;
    virtual bool haveFrontSonar() override;
    virtual bool haveRearSonar() override;
    virtual bool haveSonar() override;
    virtual bool haveLaser(size_t i) override;
    virtual int xpos() override;
    virtual int ypos() override;
    virtual int theta() override;
    virtual int xspeed() override;
    virtual int yspeed() override;
    virtual int rotspeed() override;
    virtual bool getMotionState(int &x, int &y, int &theta, int &transVel, int &rotVel, bool &stallflag, bool &enabled) override;
    virtual void getPosition(int &x, int &y, int &theta) override;
    virtual void getVelocity(int &x, int &y, int &theta) override;
    virtual bool motorsEnabled() const override { return areMotorsEnabled; }
    virtual long getSimulatorPoseX() override;
    virtual long getSimulatorPoseY() override;
    virtual int getSimulatorPoseTheta() override;
    virtual void getSimulatorPose(long &x, long &y, long &z, int &theta) override;
    virtual Pose getSimulatorPose() override;
    virtual bool stalled() override;
    virtual size_t numSonarReadings() override;
    virtual unsigned int getSonarRange(size_t i) override;
    virtual RobotInterface::Pose getSonarSensorPose(size_t i) override;
    virtual unsigned int getMaxSonarRange() override;
    virtual size_t forEachSonarReading(SonarReadingFunc &func, const size_t &start = 0) override;
    virtual char gripperState() override;
    virtual size_t numLasers() override { return lasers.size(); }
    virtual std::string laserName(size_t i) override { if(i < lasers.size()) LIKELY return lasers[i].name; else return ""; }
    virtual std::string laserType(size_t i) override { if(i < lasers.size()) LIKELY return lasers[i].type; else return ""; }
    virtual size_t numLaserReadings(size_t lasernum) override;
    virtual unsigned int getLaserReading(size_t lasernum, size_t i) override;
    virtual size_t forEachLaserReading(size_t lasernum, LaserReadingFunc &func, const size_t &start = 0) override;
    virtual int getLaserReflectance(size_t lasernum, int i) override;
    virtual double getLaserResolution(size_t lasernum) override;  ///< Degrees between readings
    virtual void setLaserResolution(size_t lasernum, double deg) override;  ///< Degrees between readings
    virtual double getLaserFOV(size_t lasernum) override; ///< Total laser FOV
    virtual void setLaserFOV(size_t lasernum, double deg) override; ///< Total laser FOV
    virtual double getLaserStartAngle(size_t lasernum) override;  ///< Angle of first reading relative to robot
    virtual double getLaserEndAngle(size_t lasernum) override;  ///< Angle of first reading relative to robot
    virtual void setLaserAngles(size_t lasernum, double first, double last) override; ///< Angle of first and last readings relative to robot
    virtual void error_s(const char* message) override; ///< critical error! locks stage mutex
    virtual void warn_s(const char* message) override;  ///< locks stage mutex
    virtual void inform_s(const char* message) override; ///< locks stage mutex
    virtual void log_s(const char* message) override; ///< locks stage mutex
    virtual void log_error_s(const char *message) override;
    virtual void shutdown(int errorcode = 0) override;
    virtual int getLastInterval() override;
    virtual int getSimInterval() override;
    virtual int getRealInterval() override;
    virtual std::vector< RobotInterface::DeviceInfo > getDeviceInfo() override;
    virtual bool haveStateOfCharge() override;
    virtual void updateStateOfCharge() override;

    virtual double getSimulatorOdomErrorX() override;
    virtual double getSimulatorOdomErrorY() override;
    virtual double getSimulatorOdomErrorTh() override;
    virtual bool haveSimulatorOdomError() override { return true; }


    /// Set whether this robot (and all of its component models) can be seen by other robots' sensors (laser,
    /// sonar)
    virtual void setInvisible(bool s) override;

    /// Set whether other robots can collide with this robot (and any of its component submodels)
    virtual void setEphemeral(bool s) override;

    virtual float getSimGPSDOP() override;

};

 

#endif
