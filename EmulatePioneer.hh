/*
 *  Pioneer emulator over TCP.
 *  Copyright (C) 2005, ActivMedia Robotics
 *  Copyright (C) 2006-2010 MobileRobots, Inc.
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

#ifndef EP_EMULATE_PIONEER_HH_
#define EP_EMULATE_PIONEER_HH_

#include "RobotInterface.hh"
#include "MapLoader.hh"

#include "ClientInterface.hh"

#include <assert.h>
//#include <pthread.h>
#include <string>
#include <set>
#include <list>

#include "ariaUtil.h"
#include "ArSocket.h"
#include "ArFunctor.h"
#include "ArRobotPacket.h"
#include "ArTcpConnection.h"
#include "ClientPacketReceiver.h"
#include "ArRobotPacketSender.h"

class ArDeviceConnection;



/* Constants required for compatability, shouldn't change these */
#define DEFAULT_PIONEER_SIM_PORT  8101
#define ROBOT_IDENTIFIER_LEN      20    // Max length of robot type, subtype and name strings.
#define MAX_PACKET_PAYLOAD_SIZE   200   // Ignore or truncate packets which claim to have more bytes of data than this

/* What version of ARCOS to claim we are in the CONFIG packet */
#define FIRMWARE_VER_MAJ 'S'  
#define FIRMWARE_VER_MIN '0'

/* Defauts for settings in the config file: */
#define DEFAULT_SIP_FREQ       100        // defaut SIP cycle time, ms.
#define DEFAULT_WATCHDOG_TIME  2000       // ms of no activity to stop robot.
#define DEFAULT_MAX_SONAR_READINGS_PER_SIP 32  // Set to a low number like 4 to crudely simulate sonar timing delay
#define DEFAULT_ROBOT_NAME     "MobileSim"
#define DEFAULT_ROBOT_CLASS       "Pioneer"
#define DEFAULT_ROBOT_SUBCLASS    "p3dx"
#define DEFAULT_DIFFCONV          0.0056
#define DEFAULT_ANGLECONV         0.001534
#define DEFAULT_DISTCONV          0.485
#define DEFAULT_VELCONV           1.0
#define DEFAULT_RANGECONV         1.0
#define DEFAULT_VEL2DIV           20.0
#define DEFAULT_STALL_ON_ROT      false
#define DEFAULT_BATTERY_TYPE      0
#define DEFAULT_BATTERY_VOLTAGE   13.0
#define DEFAULT_STATE_OF_CHARGE   100
#define DEFAULT_DIGIN_STATE 0xFF
#define DEFAULT_DIGOUT_STATE 0xFF

/* These are values clients can set during a session, but not saved between. */
#define DEFAULT_TRANSVELMAX       3000
#define DEFAULT_TRANSACCEL        300
#define DEFAULT_TRANSDECEL        300
#define DEFAULT_ROTVELMAX         150
#define DEFAULT_ROTACCEL          100
#define DEFAULT_ROTDECEL          100
#define DEFAULT_LATVELMAX         3000
#define DEFAULT_LATACCEL          300
#define DEFAULT_LATDECEL          300
#define DEFAULT_BUMPSTALL_FRONT   false
#define DEFAULT_BUMPSTALL_REAR    false


/* We don't have any "Top" limits like ARCOS does, you can
 * set any acceleration you want.  But we need to tell
 * the client something, so use these values.
 */
#define FAKE_TRANSVELTOP  9999
#define FAKE_ROTVELTOP    9999
#define FAKE_ROTACCTOP    9999
#define FAKE_TRANSACCTOP  9999
#define FAKE_LATVELTOP    9999
#define FAKE_LATACCTOP    9999


/* Operation codes used in the SIM_CTRL command */
enum {
  SIM_CTRL_LOAD_MAP = 1,
  SIM_CTRL_MASTER_LOAD_MAP = 2,
  SIM_CTRL_CLEAR_MASTER_MAP = 3,
  SIM_CTRL_ROTATE_LOGFILES = 4,
  SIM_CTRL_LOG_STATE = 5,
  SIM_CTRL_SIM_INFO = 6,
  SIM_CTRL_SET_GHOST = 7,
  SIM_CTRL_SNAPSHOT = 8
};


/** Robot parameters. There are read from the 
 *  Stage (or other) configuration. Clients cannot change these at runtime. 
 *  @todo Read more params from model definition in Stage world. Need mechanism
 *  in Stage worldfile for adding arbitrary properties to models.
 */
class RobotParams {
public:
    char RobotName[ROBOT_IDENTIFIER_LEN];  ///< Note, this is used in the MobileSim UI, but the name is always reported to clients as "MobileSim".
    char RobotClass[ROBOT_IDENTIFIER_LEN];
    char RobotSubclass[ROBOT_IDENTIFIER_LEN];
    int SIPFreq;
    int WatchdogTime;
    double DistConvFactor;
    double DiffConvFactor;
    double AngleConvFactor;
    double RangeConvFactor;
    double Vel2DivFactor;
    double VelConvFactor;

    int RotVelMax;
    int RotAccel;
    int RotDecel;
    int TransVelMax;
    int TransAccel;
    int TransDecel;
    int LatVelMax;
    int LatAccel;
    int LatDecel;

    // Note, we don't have any "Top" limits like ARCOS does. You can set the
    // vels and accels to any value you want.

    /** Set this to a low number for a crude stand-in for sonar timing delay: */
    int Sim_MaxSonarReadingsPerSIP;

    /** Normally motor stall flag is only set if trying to drive forward and
     * backwards.  Set this to true if stall flag should be set even if 
     * there is no translational velocity; however this could make it hard
     * for robots to extract themselves from a collision by rotating.
     * (Real robots usually have enough power to scrape out of a collision
     * through rotation, their motors will only stall on a direct hit.)
     */
    bool Sim_StallOnRot;

    /** What type of batteries the robot has. 
        As of Nov. 2011/ARCOS 3 Battery Type 2 means nickel with State of Charge, all others are either lead or act the same.
     */
    int BatteryType;

    int GPSPosX, GPSPosY; // GPS mounting position, robot local coordinate system, mm

    RobotParams() :
      SIPFreq(DEFAULT_SIP_FREQ), 
      WatchdogTime(DEFAULT_WATCHDOG_TIME),

      DistConvFactor(DEFAULT_DISTCONV),
      DiffConvFactor(DEFAULT_DIFFCONV),
      AngleConvFactor(DEFAULT_ANGLECONV),
      RangeConvFactor(DEFAULT_RANGECONV),
      Vel2DivFactor(DEFAULT_VEL2DIV),
      VelConvFactor(DEFAULT_VELCONV),

      RotVelMax(DEFAULT_ROTVELMAX),
      RotAccel(DEFAULT_ROTACCEL),
      RotDecel(DEFAULT_ROTDECEL),
      TransVelMax(DEFAULT_TRANSVELMAX),
      TransAccel(DEFAULT_TRANSACCEL),
      TransDecel(DEFAULT_TRANSDECEL),
      LatVelMax(DEFAULT_LATVELMAX),
      LatAccel(DEFAULT_LATACCEL),
      LatDecel(DEFAULT_LATDECEL),

      Sim_MaxSonarReadingsPerSIP(DEFAULT_MAX_SONAR_READINGS_PER_SIP),
      Sim_StallOnRot(DEFAULT_STALL_ON_ROT),

      BatteryType(DEFAULT_BATTERY_TYPE),
      GPSPosX(0), GPSPosY(0)
    {
      memset(RobotName, 0, ROBOT_IDENTIFIER_LEN);
      memset(RobotClass, 0, ROBOT_IDENTIFIER_LEN);
      memset(RobotSubclass, 0, ROBOT_IDENTIFIER_LEN);
      strncpy(RobotName, DEFAULT_ROBOT_NAME, ROBOT_IDENTIFIER_LEN);
      strncpy(RobotClass, DEFAULT_ROBOT_CLASS, ROBOT_IDENTIFIER_LEN);
      strncpy(RobotSubclass, DEFAULT_ROBOT_SUBCLASS, ROBOT_IDENTIFIER_LEN);
    }
};

/** These are values clients can set during a session, but not saved between */
class CurrentSettings {
public:
  int TransVelMax;
  int TransAccel;
  int TransDecel;
  int RotVelMax;
  int RotAccel;
  int RotDecel;
  int LatVelMax;
  int LatAccel;
  int LatDecel;
  bool BumpStallFront, BumpStallRear;
public:
  CurrentSettings(RobotParams* params = NULL) :
    TransVelMax(DEFAULT_TRANSVELMAX),
    TransAccel(DEFAULT_TRANSACCEL),
    TransDecel(DEFAULT_TRANSACCEL),
    RotVelMax(DEFAULT_ROTVELMAX),
    RotAccel(DEFAULT_ROTACCEL),
    RotDecel(DEFAULT_ROTACCEL),
    BumpStallFront(DEFAULT_BUMPSTALL_FRONT),
    BumpStallRear(DEFAULT_BUMPSTALL_REAR)
  {
    if(params)
    {
      TransVelMax = params->TransVelMax;
      TransAccel = params->TransAccel;
      TransDecel = params->TransDecel;
      RotVelMax = params->RotVelMax;
      RotAccel = params->RotAccel;
      RotDecel = params->RotDecel;
      LatVelMax = params->LatVelMax;
      LatAccel = params->LatAccel;
      LatDecel = params->LatDecel;
    }
  }
};


/** Base class for packet generators */
class PacketGenerator 
{
protected:
  RobotInterface* robotInterface; ///< Robot interface to obtain data from
  RobotParams* params; ///< Pointer to global robot paremeters struct
  bool started;        ///< Whether we should return packets or NULL
  ArRobotPacket pkt;   ///< A packet object to set data fields in, then return in getPacket()
  unsigned short deviceIndex; ///< If there is more than one packet generator, this identifies them.

public:

  /** Constructor, initialize driver table and parameters, and initialize
   * 'started' to false. */
  PacketGenerator(RobotInterface* _iface = 0, RobotParams* _params = 0, unsigned short _deviceIndex = 0) :
    robotInterface(_iface),
    params(_params),
    started(false),
    deviceIndex(_deviceIndex)
  {
    //print_debug("PacketGenerator[0x%x]::PacketGenerator: robotInterface=0x%x, params=0x%x", this, robotInterface, params);
  }

  void init(RobotInterface *_if, RobotParams* _params)
  {
    robotInterface = _if;
    params = _params;
  }

  virtual ~PacketGenerator()
  {}

  /** set started to true. */
  void start() {
    started = true;
  }

  /** Set started to false. */
  void stop() {
    started = false;
  }

  /** Return value of started. */
  bool isStarted() const { return started; }

  /** Fill in a packet (a member object, pkt) with new data and return its
   * pointer (so please don't deallocate the returned object!)
   */
  virtual ArRobotPacket* getPacket() = 0;

  RobotInterface* getRobotInterface() const { return robotInterface; }

  unsigned short getDeviceIndex() const { return deviceIndex; }
};

/** Packet generator for standard SIPs.
 *  Create one instance of this class per client session.
 *
 *  @todo One possible optimization is instead of recreating the SIP all the time from scratch,
 *  use a quickly updatable SIP: keep pointers into its buffer for the things
 *  you could change (However, you will still need to truncate it back and add
 *  the sonar data in each time, then add the stuff that comes after it).
 */
class SIPGenerator  : public virtual PacketGenerator
{
protected:
  ArTypes::Byte2 xPosAccum, yPosAccum;
  int firstSonarReadingToSend;
  bool logDataSent;
  bool robotMoved;
public:
  SIPGenerator(RobotInterface* _robotInterface = 0, RobotParams* _params = 0) :
    PacketGenerator(_robotInterface, _params),
    xPosAccum(0), yPosAccum(0), 
    firstSonarReadingToSend(0), 
    logDataSent(false),
    robotMoved(false)
  {
  }

  void setLogDataSent(bool l) { logDataSent = l; }

  void setRobotMoved(bool m = true) { robotMoved = m; }

  /** Get a finalized SIP with current data.
   *  @return packet filled with current data, or NULL if we haven't been
   *  "started" yet.
   */
  virtual ArRobotPacket* getPacket();

};


/** Generate laser data packets */
class LaserPacketGenerator  : public virtual PacketGenerator
{
protected:
  size_t currentReading;
  bool extendedInfoFormat;
  bool robotMoved;
public:
  LaserPacketGenerator(RobotInterface* _interface = 0, RobotParams* _params = 0, unsigned short deviceIndex = 0) :
    PacketGenerator(_interface, _params, deviceIndex),
    currentReading(0), extendedInfoFormat(false), robotMoved(false)
  {}

  /* Construct one laser data packet, if available. This will only contain some of the laser
   * readings: there are too many to put in one packet.  After a call to this
   * method returns the last packet in a group, it returns NULL. The next call
   * will return a packet with the first group of packets.   
   * If there is no laser data available, or we haven't been started with
   * start(), then return NULL.
   * If the ExtraReadingInfo flag is set, then the packet ID
   * will be 0x61 instead of 0x60, and each reading in the packet will be followed 
   * by two bytes of extra info. The first 4 bits of the first byte contains a 
   * reflectance value (0 for normal).
   *
   * So, you can use this method like this:
   * @code
   * ArRobotPacket *laserPacket;
   * while(laserPacket = laserGenerator.getPacket())
   *     send(laserPacket);
   * @endcode
   */
  virtual ArRobotPacket* getPacket();

  virtual void start(bool enableExtraReadingInfo = false) {
    extendedInfoFormat = enableExtraReadingInfo;
    robotInterface->openLaser();
    PacketGenerator::start();
  }

  virtual void stop() {
    robotInterface->closeLaser();
    PacketGenerator::stop();
  }

  /** A debugging tool, assert that this is a laser packet, then print it out */
  void printLaserPacket(ArRobotPacket* pkt) const;

  void setRobotMoved(bool m = true) {
    robotMoved = true;
  }
};


/** This class represents the state of one client session. It is created at the
 * beginning of the session when the client connects, and is destroyed when the client disconnects. 
 */
class Session {
public:
    Session();
    bool requestedOpenSonar;
    ArTime started;
    bool inWatchdogState;
    bool eStopInProgress;
    bool sendingSimstats;
    ArTcpConnection connection;
    ClientPacketReceiver packetReceiver;
    ArRobotPacketSender packetSender;
    CurrentSettings settings;
    //ArSocket *clientSocket; use EmulatePioneer::myClientSocket instead
    int syncSeq;
    int handshakeAttempt;
    ArTime syncStart;
    SIPGenerator sipGen;
    LaserPacketGenerator laserGen;
    ArTime sentLastSIP;
    ArTime gotLastCommand;
    ArTime gotLastValidCommand;
    unsigned long packetsSent;
    unsigned long packetsReceived;
    unsigned long laserPacketsSent;
    unsigned long miscPacketsSent;
    ArTime loggedStats;
    ArTime lastSimSetPoseTime;
    ArPose lastSimSetPose;
    bool gotSimSetPose;

    void init(RobotInterface *robotInterface, RobotParams *params, bool logSipsSent) {
      sipGen.init(robotInterface, params);
      sipGen.setLogDataSent(logSipsSent);
      laserGen.init(robotInterface, params);
      sentLastSIP.setToNow();
      gotLastCommand.setToNow();
      gotLastValidCommand.setToNow();
      sendingSimstats = false;
      eStopInProgress = false;
      inWatchdogState = false;
      requestedOpenSonar = false;
      packetsSent = 0;
      packetsReceived = 0;
      laserPacketsSent = 0;
      miscPacketsSent = 0;
      gotSimSetPose = false;
      //print_debug("Session[0x%x]::init(params=0x%x): sipGen=0x%x, laserGen=0x%x", this, params, &sipGen, &laserGen);
    }

    void checkLogStats(LogInterface *l);
    void gotPacket();
    void sentPacket();
    void sentLaserPacket();
    void sentMiscPacket();
    void logStats(LogInterface *l);
    void gotSetSimPose(int x, int y, int th)
    {
      gotSimSetPose = true;
      lastSimSetPoseTime.setToNow();
      lastSimSetPose.setPose(x, y, th);
      laserGen.setRobotMoved();
      sipGen.setRobotMoved();
    }
};


/** Pioneer emulator over TCP.
 *
 *  Communicates with a client over a TCP socket, recieve ARCOS command
 *  packets and call the appropriate methods in a RobotInterface object, and
 *  recieve data from the RobotInterface to return in information packets.
 *
 *  EmulatePioneer can be used in two ways.  You can either give it
 *  an already-connected client socket when constructing it (see
 *  EmulatePioneer::EmulatePioneer(RobotInterface*, std::string, ArTCPSocket*)), 
 *  or it can listen for new clients itself on the first available port number
 *  after the default (see EmulatePioneer::EmulatePioneer(RobotInterface*,
 *  std::string, int)
 *
 *  @todo eliminate some of the useless accessors
 * 
 */
class EmulatePioneer : public ClientInterface, public LogInterface
{
  public:

    // exception to request deletion of this EmulatePioneer instance, thrown by a callback function within that instance
    class DeletionRequest : public virtual MobileSim::DeletionRequest
    {
    private:
      EmulatePioneer* instance;
    public:
      DeletionRequest(EmulatePioneer* _inst) : instance(_inst) {}
      virtual void doDelete();
    };

    // indicates that a session disconnected and no longer active, and should not be processed for cu
    class Disconnected : public std::runtime_error
    {
    public:
      Disconnected() : std::runtime_error("Session disconnected and inactive") {}
    };


    /** Create a new EmulatePioneer using the given RobotInterface and robot
     *  model, and use the given client TCP socket for communication. 
     *  The socket must already be connected to the client. EmulatePioneer will
     *  not reopen it.
     */
    EmulatePioneer(RobotInterface *rif, std::string robotModel, ArSocket *clientSocket, bool deleteRobotInterfaceOnDisconnect = false, bool deleteClientSocketOnDisconnect = true, const MobileSim::Options *userOptions = NULL);

    /** Create a new EmulatePioneer using the given RobotInterface for the given
     *  robot model. Use @a port as the default when opening a new listening
     *  socket
     */
    EmulatePioneer(RobotInterface *rif, std::string robotModel, int port =
DEFAULT_PIONEER_SIM_PORT, bool deleteRobotInterface = false, bool
trySubsequentPorts = true, const MobileSim::Options *userOptions = NULL);

private:
    void init(const std::string& robotName, const MobileSim::Options *userOptions);   // helper for the above constructors

public:

    virtual ~EmulatePioneer();

    /** Set identifier strings which may be returned in the SIMSTAT packet. */
    void setSimulatorIdentification(const char* appName, const char* appVersion) {
      myApplicationName = appName;
      myApplicationVersion = appVersion;
    }

    void setVerbose(bool v) { 
      myVerbose = v;
    }

    void setWarnUnsupportedCommands(bool v) {
      warn_unsupported_commands = v;
    }

    void setListenAddress(std::string addr) {
      myListenAddress = addr;
    }

    /** Access list of commands to ignore (returns reference to actual stored
     * set, so you can just change or replace it directly)
     */
    std::set<int>& commandsToIgnore() { return myCommandsToIgnore; }

    void setCommandsToIgnore(const std::set<int>& ig) {
      myCommandsToIgnore = ig;
    }

    void setSRISimCompat(bool compat, bool lasercompat) {
      SRISimCompat = compat;
      SRISimLaserCompat = lasercompat;
    }

    typedef enum {
        DISCONNECTED = ArUtil::BIT0,   ///< Client no longer seems to be connected
        GOTDATA = ArUtil::BIT1,        ///< Got data from client
        NODATA = ArUtil::BIT2,         ///< Nothing received from client
        SENTDATA = ArUtil::BIT3,       ///< Sent data to client
        CMDERR = ArUtil::BIT4,         ///< Error interpreting or executing command from client
        CONERR = ArUtil::BIT5,         ///< Tried to connect with a new client but there was an error
        DELETEME = ArUtil::BIT6,       ///< Caller should delete this instance, it's no longer needed
        IDLE = ArUtil::BIT7,           ///< No client connected
    } ProcessStat;

public:
    bool openSocket();

    /** Send SIP packets to client, and update misc. status states.
     * This method must be called at the client's desired SIP frequency interval (currently always 100ms) for correct SIP timing
     * Sets @a status according to what happened.
     *  @sa ProcessStat
     *  @sa processAll()
     *  @sa stat
     */
    bool processSession() throw(DeletionRequest, Disconnected);

    /** Call processSession() on all EmulatePioneer instances.
      * If instance indicated client disconnect and its deleteOnDisconnect value is
      * true, delete it.
      * @return the bitwise or of all processSession() status codes.
      * @param maxTime if nonzero, stop processing after this amount of time (msec). Processing will resume with the next EmulatePioneer instance in the next call to processAll(). Note, this may result in clients not getting packets on time depending on how long until the next call to processAll(). If 0, process each client once.
      */
    static int processAll(int maxTime = 0);

    int status; // could be moved into session

    void loadMapObjects(ArMap *newmap);

protected:    

    /** Called after successfuly handshake; session has started, get ready to send and receive normal commands
    */
    bool beginSession();

    /** Check if a client is trying to connect, and accept TCP connection if so (calls accept() on socket)
        @return false on error
    */
    void acceptNewClient(unsigned int maxTime = 0);

    /** End session, close connection, clean up */
    void endSession() throw (DeletionRequest, Disconnected);

    void handlePacket(ArRobotPacket *pkt) throw (DeletionRequest, Disconnected);
    bool handleSyncPacket(ArRobotPacket *pkt) throw (DeletionRequest, Disconnected);
    bool handleCommand(ArRobotPacket *pkt) throw (DeletionRequest, Disconnected);

    void newSession();
    void setupSessionClientConnection();


    /**
     * If a client is connected, read and process commands from clien
     */
    void readClientInput(unsigned int maxTime);


public:
    /** Get the TCP port either in use if one has been openned, or the
     *  port that will try to be used with when a new socket is opened, or -1 if
     *  not listening for clients.
     */
    int getPort() const;

    /** Get reference to robot's permanent parameters. You should not modify them. */
    const RobotParams* getParams() const { return &params; }

    /** Get robot interface. */
    RobotInterface* getRobotInterface() const { return robotInterface; }

   // static void deleteAllInstances();

    bool deleteOnDisconnect() const { return myDeleteOnDisconnect; }

    /// may be NULL
    ArSocket* getClientSocket() { return myClientSocket; }

    void setLogPacketsReceived(bool l) { logCommandsReceived = l; }

    void setLogSIPsSent(bool l) { logSIPsSent = l; }

  private:
    friend class EmulatePioneer::DeletionRequest;

    static std::list<EmulatePioneer*> ourActiveInstances; ///< @todo change to "our connected instances"
    static std::list<EmulatePioneer*>::iterator ourNextActiveInstance; ///< the next EmulatePioneer* to process in ourActiveInstances list

    RobotInterface* robotInterface;
    RobotParams params;
    
    int myTCPPort, myRequestedTCPPort;
    ArSocket myListenSocket;
    ArSocket *myClientSocket;
    bool portOpened;

    bool sessionActive;

    //ArFunctorC<EmulatePioneer> myClientDisconnectCB;

    // TODO fix copy constructors so we can not use a pointer, and so have slightly faster access to session contents
    Session *session;

    ArDeviceConnection *clientConnection;

    /** Get a signed short integer from a command packet. 
     *  Unpack three bytes from the packet's data buffer. The first byte 
     *  determines the sign of the integer, the second and third bytes are
     *  the absolute value of the integer. 
     *  No check is made that the packet's buffer contains enough bytes.
     */
    ArTypes::Byte2 getIntFromPacket(ArRobotPacket* pkt); 

    long initialPoseX, initialPoseY; 
    int initialPoseTheta;
    bool haveInitialPose;
    std::string myApplicationName;
    std::string myApplicationVersion;
    bool myDeleteOnDisconnect;
    bool myDeleteClientSocketOnDisconnect;
    std::set<int> myCommandsToIgnore;
    bool myTrySubsequentPorts;
    bool myVerbose;
    static bool mapMasterEnabled;
    std::string myListenAddress;
    static unsigned long lifetimeConnectionCount;
    static unsigned long lifetimeFailedConnectionCount;
    static unsigned long currentEmulatorCount;
    bool logCommandsReceived;
    bool logSIPsSent;
    bool sendSIMSTAT(ArDeviceConnection *con);
    bool SRISimCompat;
    bool SRISimLaserCompat;
    bool sendMapChanged(std::string mapname, bool user = false, ArTypes::Byte status=1);
    void newMapLoaded(MapLoadedInfo info);
    ArFunctor1C<EmulatePioneer, MapLoadedInfo> newMapLoadedCB;
    ArFunctor1C<EmulatePioneer, ArRobotPacket*> handlePacketCB;
    ArFunctor1C<EmulatePioneer, unsigned int> readInputCB;
    ArFunctor1C<EmulatePioneer, unsigned int> acceptClientCB;
    bool useListenSocket;
    bool warn_unsupported_commands;

    std::list< std::vector<ArPose> > badGPSSectorVertices; 
    bool insideBadGPSSector(const ArPose& p);

    
    void changeDigout(ArTypes::UByte mask, ArTypes::UByte states)
    {
      const ArTypes::UByte curr = robotInterface->getDigoutState();
      robotInterface->setDigoutState( (curr & ~mask) | (~curr & mask & states) );
      inform("Pioneer digital output changed to %s", MobileSim::byte_as_bitstring(robotInterface->getDigoutState()).c_str());
    }

    void changeDigin(ArTypes::UByte mask, ArTypes::UByte states)
    {
      const ArTypes::UByte curr = robotInterface->getDiginState();
      robotInterface->setDiginState( (curr & ~mask) | (~curr & mask & states) );
      inform("Pioneer digital input changed to %s", MobileSim::byte_as_bitstring(robotInterface->getDiginState()).c_str());
    }


    virtual void error_s(const char *m) {
      if(robotInterface) robotInterface->error_s(m);
      else LogInterface::error_s(m);
    }
    virtual void warn_s(const char *m) {
      if(robotInterface) robotInterface->warn_s(m);
      else LogInterface::warn_s(m);
    }
    virtual void inform_s(const char *m) {
      if(robotInterface) robotInterface->inform_s(m);
      else LogInterface::inform_s(m);
    }
    virtual void log_s(const char *m) {
      if(robotInterface) robotInterface->log_s(m);
      else LogInterface::log_s(m);
    }

};



#endif

