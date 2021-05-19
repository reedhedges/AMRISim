
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

#include "AMRISim.hh"
#include "EmulatePioneer.hh"
#include "MapLoader.hh"
#include "ArMap.h"
#include "ArGPSCoords.h"

#include <unistd.h>
#include <string.h>
#include <vector>
#include <stdint.h>
#include <string>
#include <assert.h>
#include <limits>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <sys/select.h>
#endif

#include <signal.h>


/* Some useful classes from Aria */
#include "ArCommands.h"
#include "ArSocket.h"
#include "ArFunctor.h"
#include "ArTcpConnection.h"
#include "ArRobotPacket.h"
#include "ArRobotPacketSender.h"
#include "ClientPacketReceiver.h"

#include "Socket.hh"

/* Turn these on to get debugging messages: */
//#define DEBUG_SIP_VALUES 1  // Every SIP! Very frequent!
//#define DEBUG_SIP_PACKET_CONTENTS 1 // Every SIP! Very frequent, lots of data!
//#define DEBUG_SIP_SONAR_DATA 1
//#define DEBUG_CLIENT_CONNECTION 1
//#define DEBUG_SYNCS 1
//#define DEBUG_COMMANDS_RECEIVED 1 // TODO replace with check for logCommandsReceived flag

// Maximum number of sonar readings to include in one SIP. Note, we don't 
// simulated sonar polling timing.  This number must be small enough to
// avoid sending a SIP larger than 200 bytes. 
#define MAX_SONAR_READINGS_PER_SIP 16

// Amout of time to sleep after failing to open a listening TCP port before trying the same port again. (Note, will block execution during sleep.)
#define TCP_PORT_RETRY_TIME 2000 //ms

// Number of times to try opening the user's desired TCP port to listen for new clients before giving up and trying another port number
#define TCP_PORT_RETRY_LIMIT 3

// How long to allow packet receiver (therefore tcp connection therefore the non-blocking socket) to wait for data before returning if there is none
// If 0, no waiting
#define RECEIVE_WAIT_TIMEOUT 0 //ms

// Issue a warning if the total time it takes to receive a client packet is more than this amout
#define RECEIVE_TIME_WARNING 200 //ms

// How long to allow packet receiver to wait for SYNC packet during client
// handshake. This should be longer than normal RECEIVE_WAIT_TIMEOUT.
#define SYNC_READ_TIMEOUT      200 

// Maximum time limit for a whole sync handshake to last. If it goes on this long, give up.
#define SYNC_TIMEOUT 10000

// If we receive no data from a client after this long, close the connection.
#define CLIENT_CONNECTION_TIMEOUT 60000 // ms (60 sec.)

// There are four ways to deal with the sync handshake:
// First is passive, just echo back what the client sends, but warn on out of
// sequence packets. This is done if no preprocessor symbol is defined.
// The last three attempt to enforce correct sequence.
// RESTART_ON_SYNC_ERROR means to restart the handshake over at 0 (this is the spec'ed behavior that 
// ARCOS does, and ARIA should deal with.), TOLERATE_SYNC_ERROR is to just try the same sync
// sequence number again, DISCONNECT_ON_SYNC_ERROR is to close the client connection (which is the
// default if neither is defined). In any case, we disconnect if the handshake
// hasn't been successful after 10 seconds (TOLERATE) or iterations (RESTART or
// ignore error) of trying.
//#define RESTART_ON_SYNC_ERROR 1
//#define TOLERATE_SYNC_ERROR 1
//#define DISCONNECT_ON_SYNC_ERROR 1

using namespace std;

unsigned long EmulatePioneer::lifetimeConnectionCount = 0;
unsigned long EmulatePioneer::lifetimeFailedConnectionCount = 0;
std::list<EmulatePioneer*> EmulatePioneer::ourActiveInstances;
std::list<EmulatePioneer*>::iterator EmulatePioneer::ourNextActiveInstance = EmulatePioneer::ourActiveInstances.end();

#define WARN_DEPRECATED(oldname, oldnum, newname, newnum) { \
  if(myVerbose) warn("Deprecated simulator %s command (%d) received. Use %s (%d) instead.", oldname, oldnum, newname, newnum); \
}

#define WARN_DEPRECATED_QUIETLY(oldname, oldnum, newname, newnum) { \
  if(myVerbose || logCommandsReceived) log("Deprecated simulator %s command (%d) received. Use %s (%d) instead in future versions.", oldname, oldnum, newname, newnum); \
}


std::string toPrintable(unsigned char c)
{
  char buf[12];
  if(c >= 32 && c <= 126)
  {
    snprintf(buf, 11, "%02x('%c') ", c, c);
    return std::string(buf);
  }
  if(c == 0x0d) return "0d(CR) ";
  if(c == 0x0a) return "0a(LF) ";
  if(c == 0x04) return "04(EOF) ";
  snprintf(buf, 11, "%02x ", c);
  return std::string(buf);
}


/** The first byte of a command indicates the type of the following argument */
class ArgTypes
{
public:
  enum {
    POSINT = 0x3B,
    NEGINT = 0x1B,
    STRING = 0x2B
  };
};

/** Some commands for partial compatability with SRISim */
enum
{
  OLD_LRF_ENABLE = 35,
  OLD_LRF_CFG_START = 36,
  OLD_LRF_CFG_END = 37,
  OLD_LRF_CFG_INC = 38,
  OLD_END_SIM = 62,
  OLD_SET_TRUE_X = 66,
  OLD_SET_TRUE_Y = 67,
  OLD_SET_TRUE_TH = 68,
  OLD_RESET_TO_ORIGIN = 69
};


bool EmulatePioneer::mapMasterEnabled = false;


// Constructor:
// Retrieve options from the configuration file, and get ready to open
// TCP socket to listen on. Call openSocket() to start listening for a client; when client is accepted, the listening socket is temporarily closed until client closes connection, then it is reopened.
EmulatePioneer::EmulatePioneer(RobotInterface* rif, const std::string& robotName, 
    int port,  bool deleteOnDisconnect, bool trySubsequentPorts, const AMRISim::Options *userOptions) :
      LogInterface(robotName),
      status(0),
      robotInterface(rif),
      myTCPPort(port),
      myRequestedTCPPort(port),
      myClientSocket(NULL),
      portOpened(false),
      sessionActive(false),
      //myClientDisconnectCB(this, &EmulatePioneer::endSession),
      session(0),
      haveInitialPose(false), 
      myApplicationName(""), myApplicationVersion(""),
      myDeleteOnDisconnect(deleteOnDisconnect),
      myDeleteClientSocketOnDisconnect(false),
      myTrySubsequentPorts(trySubsequentPorts),
      myVerbose(false),
      myListenAddress(""),
      logCommandsReceived(false),
      logSIPsSent(false),
      SRISimCompat(false),
      SRISimLaserCompat(true),
      newMapLoadedCB(this, &EmulatePioneer::newMapLoaded),
      handlePacketCB(this, &EmulatePioneer::handlePacket),
      readInputCB(this, &EmulatePioneer::readClientInput),
      acceptClientCB(this, &EmulatePioneer::acceptNewClient),
      useListenSocket(true),
      warn_unsupported_commands(false)
{
  init(robotName, userOptions);
}

// Constructor: just set our client socket, assumed to already be connected
// to a client
EmulatePioneer::EmulatePioneer(RobotInterface *rif, const std::string& robotName, ArSocket *clientSocket, 
    bool deleteOnDisconnectOnDisconnect, bool deleteClientSocketOnDisconnect, const AMRISim::Options *userOptions) :
      LogInterface(robotName),
      status(0),
      robotInterface(rif),
      myTCPPort(-1),
      myRequestedTCPPort(-1),
      myClientSocket(clientSocket),
      portOpened(false),
      sessionActive(false),
      //myClientDisconnectCB(this, &EmulatePioneer::endSession),
      session(NULL),
      haveInitialPose(false),
      myApplicationName(""), myApplicationVersion(""),
      myDeleteOnDisconnect(deleteOnDisconnectOnDisconnect),
      myDeleteClientSocketOnDisconnect(deleteClientSocketOnDisconnect),
      myTrySubsequentPorts(false), // irrelevant in this mode, actually.
      myVerbose(false),
      myListenAddress(""),
      logCommandsReceived(false),
      logSIPsSent(false),
      SRISimCompat(false),
      SRISimLaserCompat(true),
      newMapLoadedCB(this, &EmulatePioneer::newMapLoaded),
      handlePacketCB(this, &EmulatePioneer::handlePacket),
      readInputCB(this, &EmulatePioneer::readClientInput),
      acceptClientCB(this, &EmulatePioneer::acceptNewClient),
      useListenSocket(false),
      warn_unsupported_commands(false)
{
  init(robotName, userOptions);
  //print_debug("EmulatePioneer[%p]::EmulatePioneer(): handlePacketCB = %p", this, &handlePacketCB);
  //print_debug("EmulatePioneer[%p]::EmulatePioneer(): readInputCB = %p", this, &readInputCB);
  //print_debug("EmulatePioneer[%p]::EmulatePioneer(): acceptClientCB = %p", this, &acceptClientCB);

  //ArLog::log(ArLog::Normal, "EmulatePioneer::EmulatePioneer(): recording robotInterface (%u) and &newMapLoadedCB (%u) in constructor", (unsigned int)robotInterface, (unsigned int)&newMapLoadedCB);


  AMRISim::Sockets::addSocketCallback(clientSocket, &readInputCB, "EmulatePioneer client i/o (callback to handle client input), socket provided in EP ctor");
  //print_debug("EmulatePioneer: added input socket callback %p", &readInputCB);
  newSession();

  // Register the newMapLoadedCB with mapLoader
  //ArLog::log(ArLog::Normal, "EmulatePioneer::EmulatePioneer(): adding callback: %p", (void*)&newMapLoadedCB);
  mapLoader.addCallback(&newMapLoadedCB);
}


void EmulatePioneer::init(const std::string& robotName, const AMRISim::Options *userOptions)
{
  //++currentEmulatorCount;
  strncpy(params.RobotName, robotName.c_str(), ROBOT_IDENTIFIER_LEN-1);
  params.RobotName[ROBOT_IDENTIFIER_LEN - 1] = '\0';
  robotInterface->setBatteryVoltage(DEFAULT_BATTERY_VOLTAGE);
  robotInterface->setStateOfCharge(DEFAULT_STATE_OF_CHARGE);
  robotInterface->setDigoutState(DEFAULT_DIGOUT_STATE);
  robotInterface->setDiginState(DEFAULT_DIGIN_STATE);

  if(userOptions)
  {
    setVerbose(userOptions->verbose);
    setSRISimCompat(userOptions->srisim_compat, userOptions->srisim_laser_compat);
    setWarnUnsupportedCommands(userOptions->warn_unsupported_commands);
    setLogPacketsReceived(userOptions->log_packets_received);
    setLogSIPsSent(userOptions->log_sips_sent);
    setCommandsToIgnore(userOptions->ignore_commands);
  }

}

// object to handle deletion of EmulatePioneer instances from outside an EmulatePioneer object:
void EmulatePioneer::DeletionRequest::doDelete()
{
  //print_debug("EmulatePioneer deletion request: removing from active list and deleting");
  if(instance) {
    EmulatePioneer::ourActiveInstances.remove(instance);
    delete instance;
  }
}

EmulatePioneer::~EmulatePioneer()
{
  //ArLog::log(ArLog::Normal, "EmulatePioneer::~EmulatePioneer(): deleting robotInterface (%u) and &newMapLoadedCB (%u)", (unsigned int)robotInterface, (unsigned int)&newMapLoadedCB);

  // Trying to NULLify the MapLoader's callback. MapLoader will heed this
  //   request ONLY IF this instance's newMapLoadedCB matches the one saved
  //   inside mapLoader
  //ArLog::log(ArLog::Normal, "EmulatePioneer::~EmulatePioneer(): removing callback: %p", (void*)&newMapLoadedCB);
  //mapLoader.nullifyCallback(&newMapLoadedCB);
  mapLoader.removeCallback(&newMapLoadedCB);

   //std::string name =  robotInterface->getRobotName();
   //printf("*** ~EmulatePioneer %s...\n", name.c_str()); fflush(stdout);

  sessionActive = false;
  if(myDeleteOnDisconnect && robotInterface)
  {
    //print_debug("Deleting robot interface %p...", robotInterface);
    delete robotInterface;
    //robotInterface = NULL;
    //print_debug("Deleted robot interface.");
  }

  if(myClientSocket)
    AMRISim::Sockets::removeSocketCallback(myClientSocket);

  if(myDeleteClientSocketOnDisconnect && myClientSocket)
  {
    delete myClientSocket;
    //myClientSocket = NULL;
  }

  AMRISim::Sockets::removeSocketCallback(&myListenSocket);

  //ourActiveInstances.remove(this); removed in processAll()
  //--currentEmulatorCount;
   //printf("*** ~EmulatePioneer %s done!\n", name.c_str()); fflush(stdout);
}

int EmulatePioneer::getPort()  const
{
  return myTCPPort;
}


bool EmulatePioneer::openSocket()
{
  //print_debug("opening socket");

  // close socket if  open
  if(myListenSocket.isOpen())
  {
    // remove from global Sockets list. When re-opened (new actual socket) below, callback is added again.
    AMRISim::Sockets::removeSocketCallback(&myListenSocket);
    myListenSocket.close();
  }


  portOpened = false;
  // First try a few times to open the requested port (it may still be allocated by OS
  // to a previous (recently closed) AMRISim process.
  // After opening socket, it's added to global Sockets list at the end of this
  // fuction.
  myRequestedTCPPort = myTCPPort;
  for(size_t i = 0; i < TCP_PORT_RETRY_LIMIT; ++i)
  {
    if(myListenSocket.open(myTCPPort, ArSocket::TCP, (myListenAddress == "" ? NULL : myListenAddress.c_str())))
    {
      portOpened = true;
      inform("Pioneer port opened. Listening on TCP port %d... [%d]", myTCPPort, myListenSocket.getFD());
      break;
    }
    inform("TCP port %d unavailable. Trying again in %d seconds...", myTCPPort, TCP_PORT_RETRY_TIME/1000);
    ArUtil::sleep(TCP_PORT_RETRY_TIME);
  }

  // Couldn't open requested port.
  // Keep trying until we get an open port or hit the
  // upper limit for TCP ports
  if(!portOpened)
  {
    for(++myTCPPort; myTrySubsequentPorts && myTCPPort <= 65535; ++myTCPPort)
    {
      inform("Trying TCP port %d...", myTCPPort);
      if(myListenSocket.open(myTCPPort, ArSocket::TCP, (myListenAddress == "" ? NULL : myListenAddress.c_str())) )
      {
        portOpened = true;
        inform("Pioneer port opened. Listening on TCP port %d... [%d]", myTCPPort, myListenSocket.getFD());
        break;
      }
    }
  }

  if(!portOpened) {
    error("Could not open a TCP port for Pioneer interface!");
    return false;
  }

  if(myRequestedTCPPort != myTCPPort)
    warn("Requested TCP port %d was unavailable. Using %d instead.", myRequestedTCPPort, myTCPPort);
  if(myListenAddress != "")
    inform("Pioneer interface ready for a client to connect to address %s on TCP port %d.", myListenAddress.c_str(), myTCPPort);
  else
    inform("Pioneer interface ready for a client to connect on TCP port %d.", myTCPPort);

  myListenSocket.setReuseAddress();
  myListenSocket.setNonBlock();

  AMRISim::Sockets::addSocketCallback(&myListenSocket, &acceptClientCB, "EmulatePioneer listening socket (callback to accept clients)");

  return true;
}

void EmulatePioneer::acceptNewClient(unsigned int /*maxTime*/)
{
    if(myClientSocket) return; // already have a client socket

    // Accept new client if one is trying to connect
    myClientSocket = new ArSocket();
    //print_debug("EmulatePioneer[%p]::acceptNewClient(): new client socket %p", this, myClientSocket);
    myDeleteClientSocketOnDisconnect = true;
    if(!myListenSocket.accept(myClientSocket))
    {
      warn("Error accepting client connection.");
      delete myClientSocket;
      myClientSocket = nullptr;
      return;
    }
    if(!myClientSocket->isOpen())
    {
      warn("Error accepting client connection (socket closed after accept).");
      // No client connected
      delete myClientSocket;
      myClientSocket = nullptr;
      return;
    }
    inform("Client connected from %s (%s)", myClientSocket->getHostName().c_str(), myClientSocket->getIPString());

    // Close listening socket to prevent other clients from screwing it up by
    // trying to connect.
    AMRISim::Sockets::removeSocketCallback(&myListenSocket);
    myListenSocket.close();

    myClientSocket->setNonBlock();

#ifdef DEBUG_CLIENT_CONNECTION
    //print_debug("Accepted a new client. Client TCP socket fd %d.", myClientSocket->getFD());
   // fflush(stdout);
#endif

    AMRISim::Sockets::addSocketCallback(myClientSocket, &readInputCB, "EmulatePioneer client I/O (callback to read input), socket created by EP listening");
    newSession();

    return;
}

void EmulatePioneer::newSession()
{
  // Replacing session with new one
  // TODO fix Session copy constructors so we can have slightly faster access to session
  // contents (don't use new and don't have to always dereference the pointer)
  //session = Session();
  if(session) delete session;
  session = new Session();
  //print_debug("EmulatePioneer[%p]::newSession(): new session %p", this, session);
  log("Starting new session");
  robotInterface->connect(&params);
  robotInterface->setOdom(0, 0, 0);
  if(!haveInitialPose)
  {
    // Only set these once, but we need to have set up the robot interface
    // before using it.
    long z;
    robotInterface->getSimulatorPose(initialPoseX, initialPoseY, z, initialPoseTheta);
    haveInitialPose = true;
  }
  session->handshakeAttempt = 0;
  session->syncStart.setToNow();

  session->connection.setSocket(myClientSocket);
  session->connection.setStatus(ArDeviceConnection::STATUS_OPEN);
  session->packetReceiver.setSocket(myClientSocket); //DeviceConnection(&(session->connection));
  session->packetReceiver.setProcessPacketCB(&handlePacketCB);
  session->packetSender.setDeviceConnection(&(session->connection));
  sessionActive = true;
}

void EmulatePioneer::readClientInput(unsigned int maxTime)
{
  if(!session || !myClientSocket || !sessionActive)
    return;
  try {
    if(!session->packetReceiver.readData(maxTime))  // will call handlePacket() for each packet received.
    {
      warn("Error reading client data. Ending session.");
      endSession();
    }
  } catch(Disconnected&) {
    ourActiveInstances.remove(this);
  }
}

void EmulatePioneer::handlePacket(ArRobotPacket *pkt) 
{

  if(!session)
  {
    warn("Session unexpectedly ended.");
    //print_debug("session unexpectedly ended.");
    endSession();
    return;
  }

  if(!sessionActive)
  {
    //print_debug("handlePacket: client disconnected. closing connection.");
    if(session->syncSeq < 3)
      warn("Client disconnected during SYNC%d. Closing connection.", session->syncSeq);
    else
      warn("Client disconnected (while trying to handle packet with command %d)", pkt->getID());
    ++lifetimeFailedConnectionCount;
    endSession();
    return;
  }

  session->gotPacket();

  // If still in handshake, expect SYNC packets, else handle as a client command.
  if(session->syncSeq < 3)
    handleSyncPacket(pkt);
  else
    handleCommand(pkt);
}

bool EmulatePioneer::handleSyncPacket(ArRobotPacket *pkt) 
{
  if(session->handshakeAttempt++ >= 10)
  {
    warn("Failed to handshake after 10 iterations of trying! Closing connection.");
    ++lifetimeFailedConnectionCount;
    endSession();
    return false;
  }

  if(session->syncStart.mSecSince() >= SYNC_TIMEOUT)
  {
    warn("Failed to handshake after %d sec of trying! Aborting session and closing connection.", session->syncStart.secSince());
    ++lifetimeFailedConnectionCount;
    endSession();
    return false;
  }

  if(myVerbose) log("\t SYNC %d received.", pkt->getID());

  if(pkt->getID() != session->syncSeq)
  {
#ifdef RESTART_ON_SYNC_ERROR
    warn("Recieved out of sequence SYNC packet: got %d, expecting %d. Start again from SYNC0...", pkt->getID(), session->syncSeq);
    session->syncSeq = 0;
    return true;
#elif defined(TOLERATE_SYNC_ERROR)
    warn("Recieved out of sequence SYNC packet: got %d, expecting %d. Sending %d again...", pkt->getID(), syncSeq, session->syncSeq);
    session->packetSender.com(session->syncSeq);
    return true;
#elif defined(DISCONNECT_ON_SYNC_ERROR)
    // Drop connection instead
    warn("Recieved out of sequence SYNC packet: got %d, expecting %d. Closing connection.", pkt->getID(), session->syncSeq);
    ++lifetimeFailedConnectionCount;
    return false;
#else
    // just warn and use the client's sequence.
    warn("Recieved out of sequence SYNC packet: got %d, expecting %d. Will send %d.", pkt->getID(), session->syncSeq, pkt->getID());
    session->syncSeq = pkt->getID();
#endif
  }
  else
  {
    //warn("JON_DEBUG: \t SYNC %d received (meets expectation).", pkt->getID());
  }

  if(!pkt->verifyCheckSum())
  {
#ifdef RESTART_ON_SYNC_ERROR
    warn("Recieved SYNC%d packet with bad checksum. Start again from SYNC0...", pkt->getID());
    session->syncSeq = 0;
    return true;
#elif defined(TOLERATE_SYNC_ERROR)
    warn("Recieved SYNC%d packet with bad checksum.  Sending %d again...", pkt->getID(), session->syncSeq);
    session->packetSender.com(session->syncSeq);
    return true;
#elif defined (DISCONNECT_ON_SYNC_ERROR)
    // Drop connection instead
    warn("Recieved SYNC%d packet with bad checksum. Closing connection.", pkt->getID());
    ++lifetimeFailedConnectionCount;
    endSession();
    return false;
#else
    warn("Recieved SYNC%d packet with bad checksum (ignored).", pkt->getID());
#endif
  }

  // OK, reply
  if(session->syncSeq == 2)
  {
    if (myVerbose) log("Sending SYNC2 with robot config info.");
    ArRobotPacket rpkt;
    rpkt.setID(2);
    rpkt.strToBuf("AMRISim");
    rpkt.strToBuf(params.RobotClass);
    rpkt.strToBuf(params.RobotSubclass);
    rpkt.finalizePacket();
    //print_debug("writing config info to connection...");
    if(session->connection.write(rpkt.getBuf(), rpkt.getLength()) == -1)
    {
      warn("Error sending robot indentification strings to the client. Closing connection.");
      endSession();
      return false;
    }
    session->sentPacket();

   // print_debug("sent config info.");

     beginSession();
  }
  else
  {
    if(myVerbose) log("Sending SYNC%d...", session->syncSeq);
    if(!session->packetSender.com((unsigned char)session->syncSeq))
    {
      warn("Error sending SYNC%d! Closing connection.", session->syncSeq);
      ++lifetimeFailedConnectionCount;
      endSession();
      return false;
    }
    session->sentPacket();
  }

  ++(session->syncSeq);
  //print_debug("now on syncSeq %d. (sessionActive=%d)", session->syncSeq,  sessionActive);
  return true;
}

bool EmulatePioneer::beginSession()
{
  //puts("beginSession()");fflush(stdout);
  ++lifetimeConnectionCount;
  session->init(robotInterface, &params, logSIPsSent);
  // ARIA assumes sonar are on at start:
  robotInterface->openSonar();
//  robotInterface->setBatteryVoltage(DEFAULT_BATTERY_CHARGE); // todo make this default value configurable in world file.
  //robotInterface->setHaveTemperature(false); // todo make this default value configurable in world file.
  ourActiveInstances.push_back(this);
  //print_debug("**** beginSession(): ourActiveInstances.size() is now %d.", ourActiveInstances.size());
  return true;
}


int EmulatePioneer::processAll(int maxTime)
{

  //printf("processAll(%d) ourActiveInstances.size()==%d\n", maxTime, ourActiveInstances.size());

  if(ourActiveInstances.size() == 0)
    return IDLE;
  if(maxTime < 0)
    maxTime = 0;

  ArTime t;
  t.setToNow();
  int allStat = 0;
  std::list<EmulatePioneer*>::size_type n = 0;
  //if(ourNextActiveInstance == ourActiveInstances.end())
  //  ourNextActiveInstance = ourActiveInstances.begin();
  std::list<EmulatePioneer*>::iterator i = ourNextActiveInstance;
  //unsigned int timeshare = maxTime / ourActiveInstances.size();


  do {
    // wrap to beginning of list if neccesary
    if(i == ourActiveInstances.end())
    {
      i = ourActiveInstances.begin();
    }
    
    // time to stop?
    if(maxTime > 0 && t.mSecSince() >= maxTime)
    {
      ourNextActiveInstance = ++i;
      break;
    }

    EmulatePioneer *ep = *i;

    // process session, delete object if requested
    if( ep == NULL ) {
      print_warning("EP::processAll: Encountered NULL EmulatePioneer* instance! Ignoring");
      continue;
    }

    // Another way to do this other than using exceptions is to have a separate top-level EmulatePioneer
    // function that goes through the active list and checks their 'status' members, and does the
    // deletion/removal then. This function would have to be called from the main loop in main().
    int status = 0;
    try
    {
      ep->processSession();
      status = ep->status;
      //print_debug("EP::processAll: after processSession(%p). Status=0x%x (%s) [DISCONNECTED=%p, DELETEME=%p, DISCONNECTED|DELETEME=%p]", ep, status, AMRISim::byte_as_bitstring(status).c_str(), DISCONNECTED, DELETEME, DISCONNECTED|DELETEME);

      if(status & DISCONNECTED)
      {
        //print_debug("EP::processAll: after processSession: %p disconnected. Status=0x%x (%s) [DISCONNECTED=%p, DELETEME=%p, DISCONNECTED|DELETEME=%p]", ep, status, AMRISim::byte_as_bitstring(status).c_str(), DISCONNECTED, DELETEME, DISCONNECTED|DELETEME);
        //print_debug("EP::processAll: after processSession: removing %p from active list...", ep);
        i = ourActiveInstances.erase(i); // remove from ourActiveInstances and advance to next
        if(status & DELETEME)
        {
          //print_debug("EP::processAll: Also deleting instance (status included DELETEME)");
          //delete ep;
          print_debug("!!!!!!!!! EP::processAll: why did it not throw? !!!!!!!!!!!"); exit(-99);
        }
      }
      else
      {
        ++i; // advance through ourActiveInstances normally
      }
    }
    catch(EmulatePioneer::DeletionRequest&)  // from ep->processSession()
    {
      status = ep->status;
      //print_debug("EP::processAll: catch: %p wants to be deleted. Status=0x%x (%s) [DISCONNECTED=%p, DELETEME=%p, DISCONNECTED|DELETEME=%p]", ep, status, AMRISim::byte_as_bitstring(status).c_str(), DISCONNECTED, DELETEME, DISCONNECTED|DELETEME);
      //print_debug("EP::processAll: catch: removing %p from active list and deleting...", ep);
      i = ourActiveInstances.erase(i); // remove from ourActiveInstances and advance to next
      delete ep;
    }
    catch(EmulatePioneer::Disconnected&)
    {
      //print_debug("EP::processAll: catch: %p disconnected. Status=0x%x (%s) [DISCONNECTED=%p, DELETEME=%p, DISCONNECTED|DELETEME=%p]", ep, status, AMRISim::byte_as_bitstring(status).c_str(), DISCONNECTED, DELETEME, DISCONNECTED|DELETEME);
      //print_debug("EP::processAll: catch: removing %p from active list...", ep);
      i = ourActiveInstances.erase(i);
    }
    catch(...)
    {
      print_debug("unknown exception in EmulatePioneer::processAll!");
      exit(-99);
    }

    allStat |= status;
    ++n;


    if(ourActiveInstances.size() == 0)
      break;

  } while(i != ourNextActiveInstance); // stop if we've wrapped around


  if(n < ourActiveInstances.size())
  {
    print_warning("Processed only %zu out of %zu clients in one loop. This may indicate too many clients, network problems, or some other problem. (The other clients will be processed in next loop.)", n, ourActiveInstances.size());
  }

  return allStat;
}

bool EmulatePioneer::processSession() 
{
    ArTime time;

    status = 0;
/*
    if(session && session->syncSeq < 3)
      print_debug("time since syncStart is %d ms...", session->syncStart.mSecSince());

     // If we're still in SYNC, check to make sure it hasn't timed out.
    if(session && session->syncSeq < 3 && session->syncStart.mSecSince() >= SYNC_TIMEOUT)
    {

      warn("Failed handshake after %d sec of trying! Aborting session and closing connection.", session->syncStart.secSince());
      ++lifetimeFailedConnectionCount;
      endSession();
      return false;
    }
*/

    if(!sessionActive || !session)
    {
      // Either still in SYNC and newSession() hasn't been called yet, or for processSession() was called after endSession() [which should NOT happen]
      //print_debug("EmulatePioneer::processSession: client not connected. returning.");
      return true;
    }



    // If it's been too long since we received a command, 
    // warn (like the robot's watchdog behavior, but don't stop).  
    // If it's been way too long, // exit.
    if(params.WatchdogTime != 0 && session->gotLastValidCommand.mSecSince() > params.WatchdogTime && !session->inWatchdogState)
    {
      warn("Have not received a valid command packet in %d msec. (Watchdog timeout is %d msec.) [no action taken other than this warning]",
          session->gotLastValidCommand.mSecSince(), params.WatchdogTime);
      //robotInterface->stop(); // BUG if we want to do this, we need to resume last motion command when communications are restored
      session->inWatchdogState = true;
    }

    // check for client timeout
    if(session->gotLastCommand.mSecSince() > CLIENT_CONNECTION_TIMEOUT)
    {
      warn("Have not received any data from client in %.1f sec, ", (double)(session->gotLastCommand.mSecSince())/1000.0);
      // rh 3/21/2012 we decided not to close connection due to this timeout.
      //warn("Ending session.");
      //endSession();
      return false;
    }

    // check if we are done performing an estop (Fast stop)
    if(session->eStopInProgress)
    {
      int x, y, theta;
      robotInterface->getVelocity(x, y, theta);
      if(x == 0 && y == 0 && theta == 0)
      {
        if(myVerbose)
          inform("Done ESTOPping");
        session->eStopInProgress = false;
      }
    }

    // Send next set of periodic data packets
 //  if(session->sentLastSIP.mSecSince()+timeshare/4 >= params.SIPFreq)
    {
      // Send SIMSTAT if requested
      if(session->sendingSimstats)
      {
        if(!sendSIMSTAT(&session->connection))
        {
           warn("Error sending SIMSTAT packet to a client. Stopping.");
           session->sendingSimstats = false;
        }
        else
          session->sentPacket();
      }

      ArRobotPacket* sip = session->sipGen.getPacket();
        
      // NOTE if session->sipGen returns NULL because SIP packets were disabled by
      // calling session->sipGen.stop(), then we may never know that a client is
      // disconnected because we will never get an error on write. This is OK
      // if the only time session->sipGen.stop() is called, is from a CLOSE command,
      // since the client connection is forced closed then anyway.
      if(sip)
      {
        //print_debug("Sending SIP; %ld ms since last", session->sentLastSIP.mSecSince());
        if(sip->getLength() > 200)
          warn("I'm sending a SIP that's more than 200 bytes long (%d bytes)! (This violates the protocol and ought not happen.)", sip->getLength());
//puts("sending SIP");fflush(stdout);
        int fd = session->connection.getSocket()->getFD();
        if(session->connection.write(sip->getBuf(), sip->getLength()) < 0)
        {
          warn("Error sending SIP to client. Closing connection. (fd=%d)", fd);
          endSession();       
          return false;
        }
        session->sentPacket();
        status |= SENTDATA;
      }
      session->sentLastSIP.setToNow();

      //puts("SENT SIP");

      // Send all pending laser packets (only some data is in each packet)
      while(ArRobotPacket* laserPkt = session->laserGen.getPacket())
      {
        int fd = session->connection.getSocket()->getFD();
        if(session->connection.write(laserPkt->getBuf(), laserPkt->getLength()) == -1)
        {
          warn("Error sending laser packet to client. Closing connection. (fd=%d)", fd);
          endSession();       
          return false;
        }
        session->sentLaserPacket();
        status |= SENTDATA;
      }


    }

    if(AMRISim::log_stats_freq > 0) {
      session->checkLogStats(this);
    }

    return true;
}    // end processSession()


// Command implementations:
bool EmulatePioneer::handleCommand(ArRobotPacket *pkt) 
{
  //printf("handling command packet with ID %d", pkt->getID());fflush(stdout);

  if(logCommandsReceived)
  {
    robotInterface->log("Received packet with id %d (0x%x), length given as %d (%d bytes of data)", pkt->getID(), pkt->getID(), pkt->getLength(), pkt->getDataLength());
  }
      
  if(session->inWatchdogState)
  {
    inform("Data reception restored, watchdog warning reset.");
    session->inWatchdogState = false;
    // BUG if were previously inWatchdogState, ought to resume previous motion
    // commands.
  }

  // should we ignore this command
  if(myCommandsToIgnore.find(pkt->getID()) != myCommandsToIgnore.end())
  {
    if(myVerbose) warn("Ignoring command %d, as requested in program options.", pkt->getID());
    return true;
  }


  [[maybe_unused]] char argType; // argument type byte must be read from each packet but only really needed for certain commands
  int intVal;
  unsigned int index;
  int x, y, th;
  [[maybe_unused]] char byte1, byte2, byte3;
  double left, right;
  const int charbuf_maxlen = 128;
  char charbuf[charbuf_maxlen];
  char c;
  unsigned char len;
  unsigned char ubyte1, ubyte2;
  ArRobotPacket replyPkt;
  ///@todo make this more robust against bad commands and warn (useful for
  //debugging bugs in client programs too). check arg type, check packet
  //size. might need a fancier command->function map than just switch soon.
  switch(pkt->getID())
  {
    case ArCommands::PULSE:
       break;
    case ArCommands::OPEN:
        if(logCommandsReceived) robotInterface->log("\tOPEN command");
       session->sipGen.start();
       break;
    case ArCommands::CLOSE:
        inform("Got CLOSE command, closing connection.");
        session->sipGen.stop();
        endSession();
        //print_debug("CLOSE: ended session."); fflush(stdout);
        //puts("CLOSE: ended session."); fflush(stdout);
       break;

    case ArCommands::SONAR:
        if(logCommandsReceived) robotInterface->log("\tSONAR command");
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(intVal) {
         //inform("Sonar on");
         robotInterface->openSonar();
         session->requestedOpenSonar = true;
       } else {
         //inform("Sonar off");
         robotInterface->closeSonar();
         session->requestedOpenSonar = false;
       }
       break;

    case ArCommands::ENABLE:
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(logCommandsReceived) robotInterface->log("\tENABLE %d", intVal);
       robotInterface->enableMotors(intVal);
       break;



    case ArCommands::VEL:
       if(session->eStopInProgress) break;
       intVal = getIntFromPacket(pkt);
       if(logCommandsReceived) robotInterface->log("\tVEL %4d mm/s", intVal);
       robotInterface->transVel(intVal);
       break;

    case ArCommands::VEL2:
       if(session->eStopInProgress) break;
       // The byte parameters are signed, but the argument type still
       // applies to each, I guess. Consistent, but confusing. The fact
       // that it's right,left instead of left,right is just plain
       // confusing :)
       intVal = getIntFromPacket(pkt);
       right = (double)((char)intVal) * params.Vel2DivFactor;       // low 8 bits
       left = (double)((char)(intVal>>8)) * params.Vel2DivFactor;   // high 8 bits
#ifdef DEBUG_COMMANDS_RECIEVED
       byte2 = (char)intVal;      // low 8 bits
       byte1 = (char)(intVal >> 8);  // high 8 bits
       print_debug("Pioneer emulation: <command>   VEL2  = L: %d (=> %.4f mm/s), R: %d (=> %.4f mm/s) [arg type: %d; Vel2Div=%f]", byte2, left, byte1, right, argType, params.Vel2DivFactor);
       print_debug("setting rotVel:%f deg/s, transVel:%f mm/s", (float)(ArMath::radToDeg((right - left) / 2.0 ) * params.DiffConvFactor), (float)( (left + right) / 2.0 ));
#endif
       robotInterface->rotVel( (int) ( ArMath::radToDeg((right - left) / 2.0) * params.DiffConvFactor ) );
       robotInterface->transVel( (int) ( (left + right) / 2.0 ) );
       break;




    case ArCommands::RVEL:
    case ArCommands::ROTATE:
       if(session->eStopInProgress) break;
       intVal = getIntFromPacket(pkt);
       if(logCommandsReceived) robotInterface->log("\tRVEL %4d mm/s", intVal);
#ifdef DEBUG_COMMANDS_RECIEVED
       print_debug("Pioneer emulation: <command>   RVEL  =   %4d deg/s", intVal);
#endif
      robotInterface->rotVel(intVal);
      break;

    case ArCommands::LATVEL: //110
      if(session->eStopInProgress) break;
      intVal = getIntFromPacket(pkt);
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>  LATVEL =  %4d mm/s", intVal);
#endif
      robotInterface->latVel(intVal);

        break;

    case ArCommands::MOVE:
      if(session->eStopInProgress) break;
      intVal = getIntFromPacket(pkt);
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>   MOVE  =   %4d mm", intVal);
#endif
      robotInterface->move(intVal);
      break;


    case ArCommands::HEAD:  // absolute heading
       if(session->eStopInProgress) break;
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(argType == ArgTypes::NEGINT)
         intVal = -intVal;
#ifdef DEBUG_COMMANDS_RECIEVED
       print_debug("Pioneer emulation: <command>   HEAD  =  %4d deg      speed = %d deg/s", intVal, settings.RotVelMax);

#endif
       robotInterface->heading(intVal);
       break;

    case ArCommands::DHEAD: // relative heading
       if(session->eStopInProgress) break;
       intVal = getIntFromPacket(pkt);
#ifdef DEBUG_COMMANDS_RECIEVED
       print_debug("Pioneer emulation: <command>   DHEAD =  %4d deg      speed = %d deg/s", intVal, settings.RotVelMax);

#endif
       robotInterface->deltaHeading(intVal);
       break;

    case ArCommands::STOP:
      if(session->eStopInProgress) break;
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>   STOP");

#endif
      robotInterface->stop();
      break;

    case ArCommands::ESTOP:
    case 57:
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>   ESTOP (%d)", pkt->getID());

#endif
      if(myVerbose)
        inform("Received ESTOP/QSTOP command, ESTOPping...");
      robotInterface->stop(FAKE_TRANSACCTOP, FAKE_ROTACCTOP);
      session->eStopInProgress = true;
      break;


    case ArCommands::SETA:
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      if(argType == ArgTypes::NEGINT)
      {
        session->settings.TransDecel = intVal;
#ifdef DEBUG_COMMANDS_RECIEVED
        print_debug("Pioneer emulation: <command>   SETA  = -%d mm/s2. Setting robot deceleration to %d.", intVal, session->settings.TransDecel);

#endif
        robotInterface->setDecel(session->settings.TransDecel);
      }
      else
      {
        session->settings.TransAccel = intVal;
#ifdef DEBUG_COMMANDS_RECIEVED
        print_debug("Pioneer emulation: <command>   SETA  = %d mm/s2. Setting robot acceleration to %d.", intVal, session->settings.TransAccel);

#endif
        robotInterface->setAccel(session->settings.TransAccel);
      }
      break;

    case ArCommands::SETRA:
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      if(argType == ArgTypes::NEGINT)
      {
        session->settings.RotDecel = intVal;
#ifdef DEBUG_COMMANDS_RECIEVED
        print_debug("Pioneer emulation: <command>   SETRA = -%3d mm/s2", intVal);

#endif
        robotInterface->setRotDecel(session->settings.RotDecel);
      }
      else
      {
        session->settings.RotAccel = intVal;
#ifdef DEBUG_COMMANDS_RECIEVED
        print_debug("Pioneer emulation: <command>   SETRA = %4d mm/s2", intVal);

#endif
        robotInterface->setRotAccel(session->settings.RotAccel);
      }
      break;

    case ArCommands::LATACCEL:   // 113
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      if(argType == ArgTypes::NEGINT)
      {
        session->settings.LatDecel = intVal;
        robotInterface->setLatDecel(session->settings.LatDecel);
      }
      else
      {
        session->settings.LatAccel = intVal;
        robotInterface->setLatAccel(session->settings.LatAccel);
      }
      break;

    /* TODO
    case ArCommand::SETLATV:
      break;
    */

    case ArCommands::SETV:
      argType = pkt->bufToByte();
      session->settings.TransVelMax = pkt->bufToByte2();
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>   SETV  = %4d mm/s", session->settings.TransVelMax);

#endif
      robotInterface->setDefaultVel(session->settings.TransVelMax);
      break;

    case ArCommands::SETRV:
      argType = pkt->bufToByte();
      session->settings.RotVelMax = pkt->bufToByte2();
#ifdef DEBUG_COMMANDS_RECIEVED
      print_debug("Pioneer emulation: <command>   SETRV = %4d mm/s", session->settings.RotVelMax);

#endif
      robotInterface->setDefaultRotVel(session->settings.RotVelMax);
      break;

    case ArCommands::BUMPSTALL:
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      session->settings.BumpStallFront = (intVal == 1 || intVal == 3);
      session->settings.BumpStallRear = (intVal == 2 || intVal == 3);
      break;

    case ArCommands::CONFIG:
       if(logCommandsReceived) robotInterface->log("\tCONFIG command");
       replyPkt.empty();
       replyPkt.setID(0x20);

       replyPkt.strToBuf(params.RobotClass);
       replyPkt.strToBuf(params.RobotSubclass);
       replyPkt.strToBuf("SIM");  // Serial number
       replyPkt.byteToBuf(0);  // nothing (used to indicate that robot is an AT with P2OS)
       replyPkt.byte2ToBuf(FAKE_ROTVELTOP);
       replyPkt.byte2ToBuf(FAKE_TRANSVELTOP);
       replyPkt.byte2ToBuf(FAKE_ROTACCTOP);
       replyPkt.byte2ToBuf(FAKE_TRANSACCTOP);
       replyPkt.byte2ToBuf(400); // max pwm const.
       replyPkt.strToBuf("AMRISim"); // Robot name, always AMRISim
       replyPkt.byteToBuf((char)(params.SIPFreq));
       replyPkt.byteToBuf(0);  // host baud, irrelevant for TCP
       replyPkt.byteToBuf(0);  // baud rate for AUX1
       replyPkt.byte2ToBuf((ArTypes::Byte2)(robotInterface->haveGripper()?1:0)); // have gripper // StageInterface::haveGripper() { return false; }
       replyPkt.byte2ToBuf((ArTypes::Byte2)(robotInterface->haveFrontSonar()?1:0)); // have front sonar
       replyPkt.byteToBuf((ArTypes::Byte)(robotInterface->haveRearSonar()?1:0));    // have rear sonar (note byte not byte2, this is correct)
       replyPkt.byte2ToBuf(0); // low battery alarm (decivolts)
       replyPkt.byte2ToBuf(0); // revcount const.
       replyPkt.byte2ToBuf((ArTypes::Byte2)(params.WatchdogTime));
       replyPkt.byteToBuf((char)0);  // nothing
       replyPkt.byte2ToBuf(0); // stall PWM limit const.
       replyPkt.byte2ToBuf(0); // post-stall idle time, ms
       replyPkt.byte2ToBuf(0); // joyvel const.
       replyPkt.byte2ToBuf(0); // joyrvel const.
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.RotVelMax)); // current max rot vel
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.TransVelMax)); // current max trans vel
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.RotAccel)); // current rot acc
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.RotDecel)); // current rot decel
       replyPkt.byte2ToBuf(0); // current rot kp
       replyPkt.byte2ToBuf(0); // current rot kv
       replyPkt.byte2ToBuf(0); // current rot ki
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.TransAccel)); // current trans acc
       replyPkt.byte2ToBuf((ArTypes::Byte2)(session->settings.TransDecel)); // current trans decel
       replyPkt.byte2ToBuf(0); // current trans kp
       replyPkt.byte2ToBuf(0); // current trans kv
       replyPkt.byte2ToBuf(0); // current trans ki
       replyPkt.byteToBuf(0); // number of front bumper segments TODO
       replyPkt.byteToBuf(0); // number of rear bumper segments TODO
       replyPkt.byteToBuf(0); // charger type
       replyPkt.byteToBuf(0); // sonar cycle
       replyPkt.byteToBuf(2); // autobaud
       replyPkt.byteToBuf(0); // has gyro
       replyPkt.byte2ToBuf(0); // driftfactor
       replyPkt.byteToBuf(0);  // AUX2 baud rate
       replyPkt.byteToBuf(0);  // AUX3 baud rate
       replyPkt.byte2ToBuf(0); // encoder ticks per mm
       replyPkt.byte2ToBuf(0); // voltage beneath which everything is shut down
       replyPkt.byteToBuf(FIRMWARE_VER_MAJ);
       replyPkt.byteToBuf('.');
       replyPkt.byteToBuf(FIRMWARE_VER_MIN);
       replyPkt.byteToBuf(0);  // ARCOS version number end string
       replyPkt.byte2ToBuf(0); // CW gyro error
       replyPkt.byte2ToBuf(0); // CCW gyro error
       replyPkt.byteToBuf(0);  // Kinematic delay TODO
       replyPkt.byte2ToBuf(FAKE_LATVELTOP);
       replyPkt.byte2ToBuf(FAKE_LATACCTOP);
       replyPkt.byte2ToBuf((ArTypes::Byte2)session->settings.LatVelMax);
       replyPkt.byte2ToBuf((ArTypes::Byte2)session->settings.LatAccel);
       replyPkt.byte2ToBuf((ArTypes::Byte2)session->settings.LatDecel);
       replyPkt.byte2ToBuf(0); // charge threshold (powerbot)

       replyPkt.finalizePacket();
       if(!session->connection.write(replyPkt.getBuf(), replyPkt.getLength()))
         warn("Error sending CONFIG packet to client. Continuing.");
       break;

    case ArCommands::SETO:
       argType = pkt->bufToByte();
       if(pkt->getDataLength() > 0) {
         x = pkt->bufToByte2();
         y = pkt->bufToByte2();
         th = pkt->bufToByte2();
         inform("Client requested odometry set to %d, %d, %d with SETO command.", x, y, th);
         robotInterface->setOdom(x, y, th);
       } else {
         inform("Client requested odometry reset to 0,0,0 with SETO command.");
         robotInterface->setOdom(0, 0, 0);
       }
       break;


    case ArCommands::SIM_SET_POSE:
      argType = pkt->bufToByte(); // not used

      // 4 bytes for each value
      x = pkt->bufToByte4();
      y = pkt->bufToByte4();
      th = pkt->bufToByte4();
      //printf("Got SIM_SET_POSE (%d, %d, %d)\n", x, y, th);

      // Set pose in sim:
      inform("Client requested move to true pose (%d mm, %d mm, %d deg).", x, y, th);
      robotInterface->setSimulatorPose(x, y, 0, th);
      session->gotSetSimPose(x, y, th);
      break;

    case ArCommands::BATTEST:
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      if(params.BatteryType == 2)  // battery uses state of change instead of voltage, set that
      { 
        robotInterface->inform("Setting battery state of change to %d%% (set by BATTEST command #%d)", intVal, ArCommands::BATTEST);
        robotInterface->setStateOfCharge((float) intVal);
      }
      else    // battery is just voltage, change that
      {
        if(intVal == 0) 
        {
          robotInterface->inform("Resetting battery voltage value to default value of %.2f volts (reset by BATTEST 0 command)", DEFAULT_BATTERY_VOLTAGE);
          robotInterface->setBatteryVoltage(DEFAULT_BATTERY_VOLTAGE);
        }
        else
        {
          robotInterface->inform("Setting battery voltage value to %.2f volts (set by BATTEST command #%d)", (float)intVal / 10.0f, ArCommands::BATTEST);
          robotInterface->setBatteryVoltage( (float)intVal / 10.0f );
        }
      }
      break;

    case 251: // DIGTEMPTEST
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      robotInterface->inform("Setting temperature warning flag %s (override from DIGTEMPTEST client command 251)", intVal?"on":"off");
      robotInterface->setTempWarning(intVal);
      break;

    case 252: // SPOOFANALOGTEMP
      intVal = getIntFromPacket(pkt);
      if(intVal == -128) 
      {
        robotInterface->inform("Resetting temperature to disabled (from SPOOFANALOGTEMP client command 252)");
        robotInterface->setHaveTemperature(false);
      }
      else 
      {
        robotInterface->inform("Setting temperature value %d (override from SPOOFANALOGTEMP client command 252)", intVal);
        robotInterface->setTemperature((double)intVal);
      }
      break;

    case OLD_SET_TRUE_X:
    {
       WARN_DEPRECATED("SET_X", pkt->getID(), "SIM_SET_POSE", ArCommands::SIM_SET_POSE);
       if(!SRISimCompat) break;
       argType = pkt->bufToByte();
       long tx, ty, tz;
       int tt;
       robotInterface->getSimulatorPose(tx, ty, tz, tt);
       tx = pkt->bufToByte2();
       robotInterface->setSimulatorPose(tx, ty, tz, tt);
       break;
    }

    case OLD_SET_TRUE_Y:
    {
       WARN_DEPRECATED("SET_Y", pkt->getID(), "SIM_SET_POSE", ArCommands::SIM_SET_POSE);
       if(!SRISimCompat) break;
       argType = pkt->bufToByte();
       long tx, ty, tz;
       int tt;
       robotInterface->getSimulatorPose(tx, ty, tz, tt);
       ty = pkt->bufToByte2();
       robotInterface->setSimulatorPose(tx, ty, tz, tt);
       break;
    }

    case OLD_RESET_TO_ORIGIN:
       WARN_DEPRECATED("OLDSIM_RESETTOORIGIN", pkt->getID(), "SIM_RESET", ArCommands::SIM_RESET);
       if(!SRISimCompat) break;
       [[fallthrough]];
    case ArCommands::SIM_RESET:
      robotInterface->inform_s("Client requested simulator reset. Moving robot back to its initial pose.");
      robotInterface->setOdom(0, 0, 0);
      robotInterface->resetSimulatorPose();
      break;
      
    case OLD_LRF_ENABLE:
       if(!SRISimLaserCompat) 
       {
          WARN_DEPRECATED("LRF_ENABLE", pkt->getID(), "SIM_LRF_ENABLE", ArCommands::SIM_LRF_ENABLE);
          warn("Ignoring old LRF_ENABLE command (enable SRISim laser compatability to accept");
          break;
       }             
       WARN_DEPRECATED_QUIETLY("LRF_ENABLE", pkt->getID(), "SIM_LRF_ENABLE", ArCommands::SIM_LRF_ENABLE);
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(logCommandsReceived) robotInterface->log("OLD_LRF_ENABLE %d", intVal);
       if(intVal)
       {
         inform("Laser on");
         session->laserGen.start(intVal == 2);
       }
       else
       {
         inform("Laser off");
         session->laserGen.stop();
       }
       break;

    case ArCommands::SIM_LRF_ENABLE:
       intVal = getIntFromPacket(pkt);
       index = pkt->bufToUByte();
       if(logCommandsReceived) robotInterface->log("SIM_LRF_ENABLE %d for laser#%d", intVal, index);
//log("XXX LRF_ENABLE for laser #%d with %d\n", index, intVal);
       if(intVal)
       {
         inform("Laser %d on", index);
         session->laserGen.start(intVal == 2);
       }
       else
       {
         inform("Laser %d off", index);
         session->laserGen.stop();
       }
       break;

    case OLD_LRF_CFG_START:
       if(!SRISimLaserCompat) 
       {
          WARN_DEPRECATED("LRF_CFG_START", pkt->getID(), "SIM_LRF_SET_FOV_START", ArCommands::SIM_LRF_SET_FOV_START);
          warn("Ignoring old LRF_CFG_START command (enable SRISim laser compatability to accept");
          break;
       }             
       WARN_DEPRECATED_QUIETLY("LRF_CFG_START", pkt->getID(), "SIM_LRF_SET_FOV_START", ArCommands::SIM_LRF_SET_FOV_START);
       intVal = getIntFromPacket(pkt);
       if(logCommandsReceived) robotInterface->log("OLD_LRF_CFG_START %d", intVal);
       robotInterface->setLaserAngles(0, intVal, robotInterface->getLaserEndAngle(0));
       break;

    case ArCommands::SIM_LRF_SET_FOV_START:
       intVal = getIntFromPacket(pkt);
       index = pkt->bufToUByte();
       if(logCommandsReceived) robotInterface->log("SIM_LRF_SET_FOV_START %d for laser #%d", intVal, index);
       robotInterface->setLaserAngles(0, intVal, robotInterface->getLaserEndAngle(0));
       break;

    case OLD_LRF_CFG_END:
       if(!SRISimLaserCompat) 
       {
          WARN_DEPRECATED("LRF_CFG_END", pkt->getID(), "SIM_LRF_SET_FOV_END", ArCommands::SIM_LRF_SET_FOV_END);
          warn("Ignoring old LRF_CFG_END command (enable SRISim laser compatability to accept");
          break;
       }             
       WARN_DEPRECATED_QUIETLY("LRF_CFG_END", pkt->getID(), "SIM_LRF_SET_FOV_END", ArCommands::SIM_LRF_SET_FOV_END);
       intVal = getIntFromPacket(pkt);
       if(logCommandsReceived) robotInterface->log("OLD_LRF_CFG_END %d", intVal);
       robotInterface->setLaserAngles(0, robotInterface->getLaserStartAngle(0), intVal);
       break;

    case ArCommands::SIM_LRF_SET_FOV_END:
       intVal = getIntFromPacket(pkt);
       index = pkt->bufToUByte();
       if(logCommandsReceived) robotInterface->log("SIM_LRF_SET_FOV_END %d for laser#%d", intVal, index);
       robotInterface->setLaserAngles(0, robotInterface->getLaserStartAngle(0), intVal);
       break;

    case OLD_LRF_CFG_INC:
       if(!SRISimLaserCompat) 
       {
          WARN_DEPRECATED("LRF_CFG_INC", pkt->getID(), "SIM_LRF_SET_FOV_RES", ArCommands::SIM_LRF_SET_RES);
          warn("Ignoring old LRF_CFG_INC command (enable SRISim laser compatability to accept");
          break;
       }             
       WARN_DEPRECATED_QUIETLY("LRF_CFG_INC", pkt->getID(), "SIM_LRF_SET_RES", ArCommands::SIM_LRF_SET_RES);
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(logCommandsReceived) robotInterface->log("OLD_LRF_CFG_INC %d", intVal);
       robotInterface->setLaserResolution(0, (double)intVal/100.0);
       break;

    case ArCommands::SIM_LRF_SET_RES:
       intVal = getIntFromPacket(pkt);
       index = pkt->bufToUByte();
       if(logCommandsReceived) robotInterface->log("SIM_LRF_SET_RES %d for laser #d", intVal, index);
       // ignore arg type, it can only be positive
       robotInterface->setLaserResolution(0, (double)intVal/100.0);
       break;

    case 59: // MESSAGE
       argType = pkt->bufToByte();
       len = pkt->bufToUByte();
       // first three chars are special codes.
       byte3 = pkt->bufToByte();  
       byte2 = pkt->bufToByte();
       byte1 = pkt->bufToByte();
       len -= 3;
       goto msg_getchars;
    case ArCommands::SIM_MESSAGE:
       argType = pkt->bufToByte();
       len = pkt->bufToUByte();
    msg_getchars:
       memset(charbuf, 0, charbuf_maxlen);
       for(int i = 0; i < charbuf_maxlen && i < len && i < (MAX_PACKET_PAYLOAD_SIZE-1) &&  (c = pkt->bufToByte()); ++i)
         charbuf[i] = c;
#ifdef DEBUG_COMMANDS_RECIEVED
       print_debug(stderr, "Pioneer emulation (%s): Message from client (%d bytes): %s\n", params.RobotName, len, charbuf);
#endif
       if(logCommandsReceived) robotInterface->log("MESSAGE %s", charbuf);
       inform_s(charbuf);
        // hack to make sure its displayed in gui (fix when gui display
        // problem is fixed)
       //ArUtil::sleep(100);
       break;


    case ArCommands::SIM_EXIT:
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       inform("Client requested exit with code %d (SIM_EXIT command).", intVal);
       // Wait a few seconds to let the user see the message before exiting.
       ArUtil::sleep(1500);
       robotInterface->shutdown(intVal);
       break;

    case ArCommands::SIM_STAT:
    {
       if(logCommandsReceived) robotInterface->log("\tSIM_STAT command");
       argType = pkt->bufToByte();
       intVal = pkt->bufToByte2();
       if(intVal == 0)
        session->sendingSimstats = false;
       else if(intVal == 1)
       {
        if(!sendSIMSTAT(&session->connection))
         warn("Error sending SIMSTAT packet to a client.");
        else
         session->sentMiscPacket();
       }
       else if(intVal == 2)
        session->sendingSimstats = true;
       else
        warn("Received unrecognized argument %d to SIM_STAT (expect 0, 1 or 2)", intVal);
      break;
    }

    case OLD_END_SIM:
       // NOTE this is 62, the same command as new BATT_REQUEST command, which ARIA always sends.
       // So ignore it without warning, unless SRISim compatability is on
       //WARN_DEPRECATED("OLDSIM_EXIT", pkt->getID(), "SIM_EXIT", ArCommands::SIM_EXIT);
       if(!SRISimCompat) {
          if(warn_unsupported_commands)
            robotInterface->warn("Ignoring unsupported command: BATT_REQUEST (or old sim exit) #%d", pkt->getID());
          break;
        }
       robotInterface->inform("Client requested exit (OLDSIM_EXIT command #%d).", pkt->getID());
       // Wait a few seconds to let the user see the message before exiting.
       ArUtil::sleep(1500);
       robotInterface->shutdown();
       break;

    case ArCommands::SIM_CTRL:
    {
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      if(intVal == SIM_CTRL_LOAD_MAP || intVal == SIM_CTRL_MASTER_LOAD_MAP)
      {
        if(mapMasterEnabled && intVal != SIM_CTRL_MASTER_LOAD_MAP)
        {
            if(myVerbose) inform("Client requested new map file, but map master has already set it. Ignoring.");
            break;
        }
        char mapfile[256];
        memset(mapfile, 0, 256);
        size_t filenamelen = pkt->bufToUByte2();
        if(filenamelen > 255) filenamelen = 255;
        pkt->bufToStr((char*)mapfile, (int)filenamelen);
        if(intVal == SIM_CTRL_MASTER_LOAD_MAP)
        {
            mapMasterEnabled = true;
            inform("Map master client requested new map file: \"%s\".", mapfile);
        } else {
            if(myVerbose) inform("Client requested new map file: \"%s\".", mapfile);
        }
        //ArLog::log(ArLog::Normal, "EmulatePioneer::handleCommand(): sending robotInterface (%u) and &newMapLoadedCB (%u) to mapLoader.newMap()", (unsigned int)robotInterface, (unsigned int)&newMapLoadedCB);
        mapLoader.newMap(mapfile, robotInterface, &newMapLoadedCB);
        break;
      }
      else if(intVal == SIM_CTRL_CLEAR_MASTER_MAP)
      {
        mapMasterEnabled = false;
        inform("Canceled map master mode, now any client can change the map.");
        break;
      }
      else if(intVal == SIM_CTRL_ROTATE_LOGFILES)
      {
        if(!options.log_file)
        {
          warn("Cannot rotate log files, because we're not logging to a file.");
          break;
        }
        if(mobilesim_rotate_log_files(NULL))
        {
          if(!mobilesim_reopen_log_file())
          {
            warn("Error opening new log file \"%s\" after saving old files.", options.log_file);
            break;
          }
          robotInterface->inform("Saved copies of old log files and started a new log in \"%s\".", options.log_file);
        }
        else
        {
          robotInterface->warn("Error rotating log files.");
        }
        break;
      }
      else if(intVal == SIM_CTRL_LOG_STATE)
      {
        char message[512];
        memset(message, 0, 512);
        pkt->bufToStr(message, 511);
        log("-- Begin state log --");
        log("Requested by client %s (packet received at time %lld.%lld)", myClientSocket?myClientSocket->getIPString():"(no socket)", pkt->getTimeReceived().getSec(), pkt->getTimeReceived().getMSec());
        if(strlen(message) > 0)
          log(message);
        log("Current Settings: Robot type=%s, subtype=%s, name=%s.", params.RobotClass, params.RobotSubclass, params.RobotName);
        log("  SIPFreq=%d, WatchdogTime=%d", params.SIPFreq, params.WatchdogTime);
        log("  RotVelMax=%d (default %d), TransVelMax=%d (%d), LatVelMax=%d (%d)",
          session->settings.RotVelMax, params.RotVelMax,
          session->settings.TransVelMax, params.TransVelMax,
          session->settings.LatVelMax, params.LatVelMax);
        log("  RotAccel=%d (%d), RotDecel=%d (%d), TransAccel=%d (%d), TransDecel=%d, (%d), LatAccel=%d (%d), LatDecel=%d (%d)",
          session->settings.RotAccel, params.RotAccel,
          session->settings.RotDecel, params.RotDecel,
          session->settings.TransAccel, params.TransAccel,
          session->settings.TransDecel, params.TransDecel,
          session->settings.LatAccel, params.LatAccel,
          session->settings.LatDecel, params.LatDecel);
        log("  BumpStallFront=%d, BumpStallRear=%d", session->settings.BumpStallFront, session->settings.BumpStallRear);
        log("In watchdog disable state? %s, Estopping? %s",
            session->inWatchdogState?"yes":"no", session->eStopInProgress?"yes":"no");
        if(mapMasterEnabled)
          log("Map master mode is enabled in AMRISim.");
        log("Currently have %d RobotInterface objects and %d EmulatePioneer objects.", robotInterfaces.size(), ourActiveInstances.size());
        std::string commandsIgnoredStr;
        char s[4];
        for(std::set<int>::const_iterator ig = myCommandsToIgnore.begin(); ig != myCommandsToIgnore.end(); ++ig)
        {
          snprintf(s, 4, "%d ", *ig);
          commandsIgnoredStr += s;
        }
        log("Ignored commands (as requested by user): %s", commandsIgnoredStr.c_str());
        session->logStats(this);
        robotInterface->logState();
        if(session->gotSimSetPose)
          log("Last SIM_SET_POSE command during this client session was to (%f, %f, %f) at time %lld.%lld (%d msec ago).", 
            session->lastSimSetPose.getX(), session->lastSimSetPose.getY(),
            session->lastSimSetPose.getTh(), session->lastSimSetPoseTime.getSec(),
            session->lastSimSetPoseTime.getMSec(), session->lastSimSetPoseTime.mSecSince());
        log("-- End state log --");
        break;
      }
      else if(intVal == SIM_CTRL_SIM_INFO)
      {
          // Send SIMINFO packet
         replyPkt.empty();
         replyPkt.setID(0x63);

         replyPkt.strToBuf(myApplicationName.c_str());
         replyPkt.strToBuf(myApplicationVersion.c_str());

         // feature flags:
         ArTypes::UByte4 flags = 0;
         if(!options.NonInteractive)
          flags |= ArUtil::BIT0; // have GUI
         if(options.RestartedAfterCrash)
          flags |= ArUtil::BIT1;
         replyPkt.uByte4ToBuf(flags);

         // devices:
         std::vector< RobotInterface::DeviceInfo > devs = robotInterface->getDeviceInfo();
         //printf("SIMINFO: %lu devices.\n", devs.size());
         assert(devs.size() <= std::numeric_limits<ArTypes::UByte4>::max());
         replyPkt.uByte4ToBuf((ArTypes::UByte4) devs.size());
         for(std::vector< RobotInterface::DeviceInfo >::const_iterator i = devs.begin(); i != devs.end(); ++i)
         {
           //printf("SIMINFO: adding device %s (%s:%d)\n", i->name.c_str(), i->type.c_str(), i->which);
            replyPkt.strToBuf(i->name.c_str());
            replyPkt.strToBuf(i->type.c_str());
            replyPkt.uByteToBuf((unsigned char) i->which);
            replyPkt.uByte4ToBuf(i->status); // status
         }
         replyPkt.finalizePacket();
         if(!session->connection.write(replyPkt.getBuf(), replyPkt.getLength()))
           warn("Error sending SIMINFO packet to a client.");
         break;
      }
      else if(intVal == SIM_CTRL_SET_GHOST)
      {
        log("SIM_CTRL Ghost (7) command received, making this robot invisible and ephemeral...");
        robotInterface->setInvisible(true);
        robotInterface->setEphemeral(true);
      }
      else
      {
        warn("Unknown SIM_CTRL code %d.", intVal);
      }
      break;
    }

    case ArCommands::DIGOUT:
      //argType = pkt->bufToByte();
       // The byte parameters are signed, but the argument type still
       // applies to each, I guess. Consistent, but confusing.
      intVal = getIntFromPacket(pkt);
      ubyte2 = (unsigned char)intVal;
      ubyte1 = (unsigned char)(intVal >> 8);
      changeDigout(ubyte1, ubyte2);
      break;

    case ArCommands::TTY2:
    //Can't do anything with TTY3 which is the same as OLDSIM_SETORIGIN_X.
    case ArCommands::TTY4:
    {
      if(!myVerbose) break;

      argType = pkt->bufToByte();
      size_t strlen = (size_t)(pkt->bufToUByte());
      std::string str;
      for(unsigned int i = 0; i < strlen; ++i)
      {
        str += toPrintable(pkt->bufToUByte());
      }
      inform("For aux tty %d: %s (%u bytes)", (pkt->getID() == ArCommands::TTY2)?1:3, str.c_str(), strlen);
      break;
    }


    case 31: // SETPBIOPORT
    {
      if(!warn_unsupported_commands) break;

      argType = pkt->bufToByte();
      ubyte1 = pkt->bufToUByte();  // On/Off (this was the LSB of the 2-byte int)
      ubyte2 = pkt->bufToUByte();  // Num    (this was the MSB of the 2-byte int)
      warn("Ignoring unsupported command: SETPBIOPORT %u %s.", ubyte2, ubyte1?"ON":"OFF");
      break;
    }


    case ArCommands::MOVINGBLINK:
    {
      if(!warn_unsupported_commands) break;

      argType = pkt->bufToByte();
      intVal = pkt->bufToUByte2();
      warn("Ignoring command: MOVINGBLINK %s.", intVal?"ON":"OFF");
      break;
    }

    case ArCommands::JOYINFO:
    {
      if(!warn_unsupported_commands) break;
      argType = pkt->bufToByte();
      intVal = pkt->bufToUByte2();
      warn("Ignoring command: JOYPAC %u.", intVal);
      break;
    }

    case ArCommands::JOYDRIVE:
    {
      if(!warn_unsupported_commands) break;
      argType = pkt->bufToByte();
      intVal = pkt->bufToByte2();
      warn("Ignoring command: JOYDRIVE %d.", intVal);
      break;
    }

    case ArCommands::IOREQUEST:
    {
      if(!warn_unsupported_commands) break;
      argType = pkt->bufToByte();
      intVal = pkt->bufToUByte2();
      warn("Ignoring unimplemented command: IOREQ with argument %u.", intVal);
      break;
    }


    // "Known unsupported", but less common or maybe important to know, so warn in the gui
    //case ArCommands::SOUNDTOG:
    //  if(myVerbose) warn("Received unsupported command %d (Amigo-H8 SOUNDTOG). Ignoring.", pkt->getID());
    //  break;
    case ArCommands::DCHEAD:
      if(warn_unsupported_commands) warn("Received unsupported command %d (DCHEAD). Ignoring.", pkt->getID());
      break;
    case ArCommands::SOUND:
      if(warn_unsupported_commands) warn("Received unsupported command %d (SOUND). Ignoring.", pkt->getID());
      break;
    case ArCommands::PLAYLIST:
      if(warn_unsupported_commands) warn("Received unsupported command %d (Amigo-H8 PLAYLIST). Ignoring.", pkt->getID());
      break;
    case ArCommands::TCM2:
      if(warn_unsupported_commands) warn("Received unsupported command %d (TCM2). Ignoring.", pkt->getID());
      break;


    case 253:
    case 255:
        // intentionally raise a SIGABRT signal to trigger crash handler
        if(options.NonInteractive)
            log_error("Received maintainence command 255, aborting program. ABORT signal may trigger crash/debug handler if supported in this platform and installation (in noninteractive mode).");
        else
            log_error("Received maintainence command 255, triggering ABORT signal.");
        abort();
        //raise(SIGABRT);
        break;

    // Unrecognized command, ignore
    default:
      if(warn_unsupported_commands) warn("Received unsupported command %d. (Ignored)", pkt->getID());
      break;
  } // switch

  return true;
} // handleCommand()

void EmulatePioneer::endSession() 
{
  if(!sessionActive)
  {
    print_debug("note, endSession() called on an inactive session");
    return;
  }
  inform("Client disconnected.");
  if(robotInterface) robotInterface->disconnect();
  sessionActive = false;
  status |= DISCONNECTED;
  if(myClientSocket)
  {
    log("Closing connection.");
    //print_debug("Closing connection for ArSocket %p, fd=%d", myClientSocket, myClientSocket->getFD());
    AMRISim::Sockets::removeSocketCallback(myClientSocket);
    myClientSocket->close();
    if(myDeleteClientSocketOnDisconnect)
    {
      print_debug("Deleting clientSocket %p...", myClientSocket);
      delete myClientSocket;
    }
  }
  myClientSocket = nullptr;
  delete session;
  session = nullptr;

  //print_debug("EmulatePioneer::endSession: myDeleteOnDisconnect=%d", myDeleteOnDisconnect);
  if(myDeleteOnDisconnect)
  {
    status |= DELETEME;
    //print_debug("EmulatePioneer::endsession: throwing deletion request exception with status=0x%x", status);
    throw DeletionRequest(this);
  }
  else
  {
    // we added DISCONNECTED to status above, so processAll can do whatever it needs to do to deal with this EP becoming inactive

    // re-open listen socket if last session was created from accepting a socket on listensocket
    if(useListenSocket)
      openSocket();

    throw Disconnected(); // causes caller to remove this EP from active list
  }
}


ArRobotPacket* SIPGenerator::getPacket()
{
  if(!started || !robotInterface || !params) return 0;

#if 0
    // debugging:
    if(positionData)
      printf("DEBUG: position x=%d y=%d th=%d vel=%d rotvel=%d\n",
          (int16_t) revByteOrder32(positionData->xpos), 
          (int16_t) revByteOrder32(positionData->ypos), 
          (int16_t) revByteOrder32(positionData->yaw), 
          (int16_t) revByteOrder32(positionData->xspeed), 
          (int16_t) revByteOrder32(positionData->yawspeed));
    if(powerData)
      printf("DEBUG: power charge=%d\n", revByteOrder32(powerData->charge));
#endif

  // TODO, keep a data buffer ourselves so it's not reallocated so much
  // (same with config pakcets!)
 
  /* Note, Pioneer comms is always little endian (Intel byte order) */

  pkt.empty();
  pkt.setID(0x32);



  // Get all robot motion state at once, saves us several function calls that
  // might lock:
  int x, y, theta, transVel, rotVel;
  bool stalled, motorsEnabled;
  robotInterface->getMotionState(x, y, theta, transVel, rotVel, stalled, motorsEnabled);

  if(robotInterface->havePositionData())
  {

    // different packet ID if robot is moving or stopped: (or should it be for
    // enabled vs. disabled?)  Does this even matter? What's the point?
    //if (positionData->xspeed > 0 || positionData->yawspeed > 0 ||
    //  positionData->yspeed > 0)
    //{
    //  pkt->setID(0x33);
    //} else {
    //  pkt->setID(0x32);
    //}

    // By adding the difference from the last time we checked to the 16 bit 
    // variable, we assure that
    // the pos returned wraps at the 16 bit limit just like ARIA checks for,
    // with a minimum of fuss.

    // Note: Compilers and analyzers will warn about conversion from int32_t to ArTypes::Byte2 (int16_t). 
    // We want to force the conversion however. Someday this may be fixed to express this better.

    assert(params->DistConvFactor > 0.0001);

    xPosAccum += (int32_t)( (double)x/params->DistConvFactor - xPosAccum );
    pkt.byte2ToBuf(xPosAccum);

    yPosAccum += (int32_t)( (double)y/params->DistConvFactor - yPosAccum );
    pkt.byte2ToBuf(yPosAccum);


    // theta, just truncated from 32 bits, stage should keep this between
    // -180,180 for us.
    assert(params->AngleConvFactor > 0.0001);
    const ArTypes::Byte2 siptheta = (ArTypes::Byte2) (ArMath::degToRad(theta) / params->AngleConvFactor);
    pkt.byte2ToBuf(siptheta);  

    // wheel velocities (left, right):
    int16_t rightVel, leftVel;
    if (params->DiffConvFactor > 0.0)
    {
      rightVel = (int16_t) (transVel + ArMath::degToRad((double)rotVel / params->DiffConvFactor));
      leftVel =  (int16_t) (transVel - ArMath::degToRad((double)rotVel / params->DiffConvFactor));
    }
    else
    {
      rightVel = (int16_t) transVel;
      leftVel =  (int16_t) transVel;
    }

      
    pkt.byte2ToBuf((ArTypes::Byte2)leftVel);
    pkt.byte2ToBuf((ArTypes::Byte2)rightVel);

    if(logDataSent)
    {
      robotInterface->log("Sending SIP with: X=%d, Y=%d, Th=%d (converted from (%d,%d,%d) with DistConv=%f, AngleConv=%f), LeftVel=%d, RightVel=%d (from %d mm/s trans vel with DiffConv=%f)", xPosAccum, yPosAccum, siptheta, x, y, theta, params->DistConvFactor, params->AngleConvFactor, leftVel, rightVel, transVel, params->DiffConvFactor);
    }

  }
  else
  {
    // Have no position data.
    //pkt->setID(0x32);
    pkt.byte2ToBuf(0);   // xpos
    pkt.byte2ToBuf(0);   // ypos
    pkt.byte2ToBuf(0);   // theta
    pkt.byte2ToBuf(0);   // lvel
    pkt.byte2ToBuf(0);   // rvel

    if(logDataSent)
    {
      robotInterface->log("Sending SIP with: XPOS=0, YPOS=0, THETA=0, LeftVel=0, RightVel=0 (Have no position data yet!)");
    }

  }
  if(params->BatteryType == 2)
    pkt.byteToBuf(0);  // battery reports state of charge later in sip instead of voltage
  else
    pkt.byteToBuf( (ArTypes::Byte) (robotInterface->getBatteryVoltage() * 10.0) ); // battery voltage, decivolts, one byte

  // Stall and bumper flags. bumpers are not implemented yet.
  // Simulation will stall if we hit something (or are extremely close to
  // hitting.)
  // Stall only if trying to translate, not rotate. This allows software to have
  // some hope of getting out of the stall by itself.
  ArTypes::UByte2 bumpStallFlags = 0;
  if(stalled)
    bumpStallFlags |= ArUtil::BIT0 | ArUtil::BIT8;
  pkt.uByte2ToBuf(bumpStallFlags); 

  pkt.byte2ToBuf(0);  // "control", not used anymore.

  // Misc status flags:
  ArTypes::UByte2 flags = 0;
  
  // sonar status flags on: 
  if(robotInterface->sonarOpenRequested())
  {
    flags |= ArUtil::BIT1 | ArUtil::BIT2 
      | ArUtil::BIT3 | ArUtil::BIT4;
  }

  // Motor status
  if(motorsEnabled) flags |= ArUtil::BIT0;
  pkt.uByte2ToBuf(flags); 

  pkt.uByteToBuf(0);   // compass, not used.

  // Sonar values:
  if(robotInterface->sonarOpen() && robotInterface->haveSonar())
  {
    const int n = (int) robotInterface->numSonarReadings(); 
#ifdef DEBUG_SIP_SONAR_DATA
    print_debug("SIP: sonar has %d readings. Max per packet is %d.", n, params->Sim_MaxSonarReadingsPerSIP);
#endif
    pkt.byteToBuf((ArTypes::Byte)n);

    class PackSonarReadingFunc : public virtual RobotInterface::SonarReadingFunc {
    public:
      ArRobotPacket &pkt;
      const double conv;
      const size_t max;
      size_t i;
      size_t count;
      PackSonarReadingFunc(ArRobotPacket &init_pkt, const double &init_conv, const size_t &init_max, const size_t &init_i = 0) 
        : pkt(init_pkt), conv(init_conv), max(init_max), i(init_i), count(0)
        {}
      virtual bool operator()(unsigned int r) override {
        pkt.byteToBuf((ArTypes::Byte)i);
        const ArTypes::Byte2 convr = (ArTypes::Byte2) ArMath::roundInt(r / conv);
        pkt.byte2ToBuf(convr);

#ifdef DEBUG_SIP_SONAR_DATA
        print_debug("SIP: Sonar #%lu is %d (converted from %d) conv=%0.2f count=%lu max=%lu", i, convr, r, conv, count, max);
#endif

        ++i;
        if(++count > max) return false;
        return true;
      }
    };
  
    PackSonarReadingFunc func(pkt, params->RangeConvFactor, (size_t) params->Sim_MaxSonarReadingsPerSIP, (size_t) firstSonarReadingToSend);

    const int numPacked = (int) robotInterface->forEachSonarReading(func, (size_t) firstSonarReadingToSend);
#ifdef DEBUG_SIP_SONAR_DATA
    print_debug("SIP: Sent %d sonar readings", numPacked);
#endif

    if(numPacked >= n)
      firstSonarReadingToSend = 0; // start from beginning next time
    else
      firstSonarReadingToSend += numPacked;
#ifdef DEBUG_SIP_SONAR_DATA
    print_debug("SIP: Will start with sonar reading #%d in next SIP.", firstSonarReadingToSend);
#endif
  }
  else
  {
#ifdef DEBUG_SIP_SONAR_DATA
    print_debug("SIP: sonar not enabled.");
#endif
    pkt.byteToBuf(0);   
  }

  // More:
  pkt.byteToBuf(robotInterface->gripperState()); 
  pkt.byteToBuf(0);   // analog select
  pkt.byteToBuf(0);   // analog data
  pkt.byteToBuf((ArTypes::Byte) robotInterface->getDiginState()); //(char)0xFF);   // digital in, also used for IR on peoplebot (0 means triggered, 1 means not triggered)
  pkt.byteToBuf((ArTypes::Byte) robotInterface->getDigoutState()); //(char)0xFF);   // digital out (0 means triggered, 1 means not triggered)
  if(params->BatteryType == 2)
    pkt.byte2ToBuf(0);  // battery reports state of charge later in sip instead of voltage
  else
    pkt.byte2ToBuf( (ArTypes::Byte2) (robotInterface->getBatteryVoltage()*10.0) );  // battery decivolts in two bytes, default of 13V 

  pkt.uByteToBuf(0);  // recharge/dock status (0=not recharging,1=bulk,2=over,3=float)

  pkt.byte2ToBuf( (ArTypes::Byte2) (rotVel*10.0) );  // current rot. vel

  if(logDataSent)
  {
    robotInterface->log("Sending SIP with: RotVel=%d (%d deg/s)", (int)(rotVel*10.0), rotVel);
  }

  if(robotInterface->getTempWarning())
    pkt.byte2ToBuf((ArTypes::Byte2)ArUtil::BIT1); // fault flags
  else
    pkt.byte2ToBuf(0); // fault flags
  pkt.byte2ToBuf((ArTypes::Byte2) robotInterface->yspeed());
  if(robotInterface->haveTemperature())
    pkt.byteToBuf((ArTypes::Byte) robotInterface->getTemperature());
  else
    pkt.byteToBuf((ArTypes::Byte) -127);

  //if(params->BatteryType == 2)  // should provide state of charge with battery type 2
  if(robotInterface->haveStateOfCharge())  // should provide state of charge with battery type 2
  {
    robotInterface->updateStateOfCharge();
    //ArLog::log(ArLog::Normal, "SIPGenerator::getPacket(): Robot: %s, Packing SoC into SIP: %f\n", robotInterface->getRobotName().c_str(), robotInterface->getStateOfCharge());
    pkt.byteToBuf((ArTypes::Byte) robotInterface->getStateOfCharge());
  }
  else  // we have no state of charge with other battery types
    pkt.byteToBuf((ArTypes::Byte) 0);
  pkt.finalizePacket();

#ifdef DEBUG_SIP_PACKET_CONTENTS
  printf("Pioneer emulation: ------- sending SIP: ----\n");
  pkt.printHex();
  printf("--------------------------------------------\n");
#endif 


  return &pkt;
}

ArRobotPacket* LaserPacketGenerator::getPacket()
{
  if(!started || !robotInterface || !robotInterface->haveLaser(0) || !params) return 0;

  // max. number of readings per packet.
  const int MaxReadingsPerPacket = 32;

  // Figure out how many readings to put in this packet. 

  //ArTypes::Byte2 totalReadings = robotInterface->numLaserReadings();
  const size_t totalReadings = robotInterface->numLaserReadings(0);
  if(currentReading >= totalReadings) 
  {
    currentReading = 0;
    return NULL;
  }


  pkt.empty(); // TODO just replace the fields in the packet buffer which have changed instead of emptying and repacking

  if(extendedInfoFormat) 
  {
    pkt.setID(0x61);
  } 
  else 
  {
    pkt.setID(0x60);
    int x, y, th;
    robotInterface->getPosition(x, y, th);
    pkt.byte2ToBuf((ArTypes::Byte2)x);
    pkt.byte2ToBuf((ArTypes::Byte2)y);
    pkt.byte2ToBuf((ArTypes::Byte2)th);
  }
  assert(totalReadings <= std::numeric_limits<ArTypes::Byte2>::max());
  pkt.byte2ToBuf((ArTypes::Byte2)totalReadings);   // total range reading count the device has
  pkt.byte2ToBuf((ArTypes::Byte2) currentReading); // which reading is the first one in this packet
  const int numReadingsThisPacket = AMRISim::min((int)MaxReadingsPerPacket, (int)(totalReadings - currentReading));  // num. readings that follow
  pkt.uByteToBuf((ArTypes::UByte) numReadingsThisPacket);

  class PackLaserReadingFunc_OldFormat : public virtual RobotInterface::LaserReadingFunc {
  protected:
    ArRobotPacket &pkt;
    size_t i;
    const size_t max;
    size_t count;
  public:
    PackLaserReadingFunc_OldFormat(ArRobotPacket &set_pkt, const size_t &set_max, size_t init_i = 0) 
      : pkt(set_pkt), i(init_i), max(set_max), count(0)
      {}
    virtual ~PackLaserReadingFunc_OldFormat() {}
    virtual bool operator()(unsigned int range, int /*ref*/) override {
      pkt.uByte2ToBuf((ArTypes::UByte2)range);
      ++i;
      if(++count >= max) return false;
      return true;
    }
  };

  class PackLaserReadingFunc_ExtFormat : public virtual PackLaserReadingFunc_OldFormat {
  public:
    PackLaserReadingFunc_ExtFormat(ArRobotPacket &set_pkt, const size_t &set_max, size_t init_i = 0) 
      : PackLaserReadingFunc_OldFormat(set_pkt, set_max, init_i)
      {}
    virtual ~PackLaserReadingFunc_ExtFormat() {}
    virtual bool operator()(unsigned int range, int ref) override {
      const bool r = PackLaserReadingFunc_OldFormat::operator()(range, ref);
      pkt.uByteToBuf((ArTypes::UByte)(ref));
      pkt.uByteToBuf(0); // reserved for future flags
      pkt.uByteToBuf(0); // reserved for future flags
      //if(!r) printf("PackLaserReadingFunc Hit max readings (count=%d, max=%d)\n", count, max);
      return r;
    }
  };

  size_t numPacked = 0;

  if(extendedInfoFormat) {
    PackLaserReadingFunc_ExtFormat func(pkt, MaxReadingsPerPacket, currentReading);
    numPacked = robotInterface->forEachLaserReading(0, func, currentReading);
    pkt.uByteToBuf(0); // laser device index (multiple lasers)
    unsigned char flags = 0;
    if(robotMoved)
    {
      flags |= 1;
      robotMoved = false;
    }
    pkt.uByteToBuf(flags);
  } else {
    PackLaserReadingFunc_OldFormat func(pkt, MaxReadingsPerPacket, currentReading);
    numPacked = robotInterface->forEachLaserReading(0, func, currentReading);
  }
  //printf("num laser readings packed: %d (should be %d)\n", numPacked, numReadingsThisPacket);
  assert(numPacked == (size_t) numReadingsThisPacket);

//  pkt.uByteToBuf(1); // fake laser index for testing


  if(numPacked > totalReadings)
    currentReading = 0;
  else
    currentReading += numPacked;

  pkt.finalizePacket();
  return &pkt;
}
  
/* Debugging tool */
void LaserPacketGenerator::printLaserPacket(ArRobotPacket* pkt) const
{
  assert(pkt->verifyCheckSum());
  assert(pkt->getID() == 0x60 || pkt->getID() == 0x61);
  printf("Laser packet length=%d: ", pkt->getLength());
  if(pkt->getID() == 0x60)
  {
    pkt->bufToByte2();
    pkt->bufToByte2();
    pkt->bufToByte2();
  }
  int total = pkt->bufToByte2();
  int first = pkt->bufToByte2();
  int here = pkt->bufToUByte();
  printf("total=%d this=%d curr=%d  readings: ", total, here, first );
  for(int i = 0; i < here; ++i)
  {
    printf("%d ", pkt->bufToUByte2());
    fflush(stdout);
  }
  if(pkt->getID() == 0x61)
  {
    printf("(R=%d) ", pkt->bufToUByte());
    fflush(stdout);
    pkt->bufToUByte();
    pkt->bufToUByte();
  }
  if(robotMoved) printf(" Robot moved.\n");
  printf("\n");
}

int EmulatePioneer::getIntFromPacket(ArRobotPacket* pkt)
{
   if(pkt->bufToByte() == ArgTypes::NEGINT)
     return (-1 * pkt->bufToByte2());
   else
     return pkt->bufToByte2();
}

bool EmulatePioneer::sendSIMSTAT(ArDeviceConnection *con)
{
  if(!con) return false;

  ArRobotPacket replyPkt;
  replyPkt.empty();
  replyPkt.setID(0x62);

  // compatability with older AMRISim versions:
  replyPkt.strToBuf("");
  replyPkt.strToBuf("");

  // status flags
  ArTypes::UByte4 flags = 0;
  if(mapLoader.haveMapOriginLLA)
    flags |= ArUtil::BIT1;
  if(robotInterface->haveSimulatorOdomError())
    flags |= ArUtil::BIT2;
  replyPkt.uByte4ToBuf(flags);

  // time statistics:
  replyPkt.uByte2ToBuf((unsigned short)robotInterface->getSimInterval());
  replyPkt.uByte2ToBuf((unsigned short)robotInterface->getRealInterval());
  replyPkt.uByte2ToBuf((unsigned short)robotInterface->getLastInterval());

  // true pose:
  long x, y, z;
  int th;
  robotInterface->getSimulatorPose(x, y, z, th);
  replyPkt.byte4ToBuf((int)x);
  replyPkt.byte4ToBuf((int)y);
  replyPkt.byte4ToBuf((int)z);
  replyPkt.byte4ToBuf(th);

  // true pose, converted into latitude and longitude
  if(mapLoader.haveMapOriginLLA)
  {
    ArLLACoords mapOrigin = mapLoader.getMapOriginLLA();
    //double mapOriginAlt = map->getOriginAltitude();
    ArMapGPSCoords mapCoords(mapOrigin); //ArLLACoords(mapOrigin.getX(), mapOrigin.getY(), mapOriginAlt));
    const float gpspx = (float) params.GPSPosX;
    const float gpspy = (float) params.GPSPosY;
    const double sinth = ArMath::sin(th);
    const double costh = ArMath::cos(th);
    // find offset GPS receiver position in global coords
    const double xoffset = gpspx*costh - gpspy*sinth;
    const double yoffset = gpspx*sinth + gpspy*costh;
    //print_debug("Pioneer emulation: Sending GPS position with GPS position offset %f, %f mm", xoffset, yoffset);
    double lat, lon, alt;
    mapCoords.convertMap2LLACoords((double)x+xoffset, (double)y+yoffset, mapOrigin.getAltitude(), lat, lon, alt);
    replyPkt.byte4ToBuf((int)(lat*10e6));
    replyPkt.byte4ToBuf((int)(lon*10e6));
    replyPkt.byte4ToBuf((int)(alt*100));
    if(insideBadGPSSector(ArPose((double)x+xoffset, (double)y+yoffset, th)))
    {
      replyPkt.byte4ToBuf(0);
    }
    else
    {
      replyPkt.byte4ToBuf((ArTypes::Byte4)(robotInterface->getSimGPSDOP() * 100.0));
      //replyPkt.byte4ToBuf(100);
    }
  }
  else
  {
    replyPkt.byte4ToBuf(0);
    replyPkt.byte4ToBuf(0);
    replyPkt.byte4ToBuf(0);
    replyPkt.byte4ToBuf(-1);
  }

    // odometry error
  if(robotInterface->haveSimulatorOdomError())
  {
    replyPkt.byte4ToBuf((ArTypes::Byte4)(robotInterface->getSimulatorOdomErrorX() * 10e6));
    replyPkt.byte4ToBuf((ArTypes::Byte4)(robotInterface->getSimulatorOdomErrorY() * 10e6));
    replyPkt.byte4ToBuf((ArTypes::Byte4)(robotInterface->getSimulatorOdomErrorTh() * 10e6));
  }
  else
  {
    replyPkt.byte4ToBuf(0);
    replyPkt.byte4ToBuf(0);
    replyPkt.byte4ToBuf(0);
  }

  replyPkt.finalizePacket();
  return con->write(replyPkt.getBuf(), replyPkt.getLength());
}

bool EmulatePioneer::insideBadGPSSector(const ArPose& p)
{
  for(std::list< std::vector<ArPose> >::const_iterator i = badGPSSectorVertices.begin(); i != badGPSSectorVertices.end(); ++i)
  {
     if(p.isInsidePolygon(*i))
      return true;
  }
  return false;
}

bool EmulatePioneer::sendMapChanged(std::string mapname, bool user, ArTypes::Byte status)
{
  ArRobotPacket pkt;
  pkt.empty();
  pkt.setID(102);
  pkt.uByteToBuf(user);
  pkt.byteToBuf(status);
  pkt.strToBuf(mapname.c_str());
  pkt.finalizePacket();
  return session->connection.write(pkt.getBuf(), pkt.getLength());
}

void EmulatePioneer::newMapLoaded(MapLoadedInfo info)
{
  //ArLog::log(ArLog::Normal, "EmulatePioneer::newMapLoaded(): %s", robotInterface->getRobotName().c_str());

  if(!sendMapChanged(info.filename, false, (ArTypes::Byte)info.status))
  {
    warn("Could not send SIM_MAP_CHANGED notification packet!");
  }

  // If we have map data, and the map has actually changed (status != 0), 
  // then get info about new map objects.
  if(info.status != 0 && info.map)
    loadMapObjects(info.map);
}

void EmulatePioneer::loadMapObjects(ArMap *map)
{
  // Copy locations of BadGPSSector objects:
  badGPSSectorVertices.clear(); 
  if(!map) return;
  std::list<ArMapObject*> badGPSSectors = map->findMapObjectsOfType("BadGPSSector", true);
  std::list<ArMapObject*> simBadGPSSectors = map->findMapObjectsOfType("SimBadGPSSector", true);
  badGPSSectors.splice(badGPSSectors.end(), simBadGPSSectors);
  log("Found %d BadGPSSector and SimBadGPSSector objects while loading new map.", badGPSSectors.size());
  for(std::list<ArMapObject*>::const_iterator i = badGPSSectors.begin(); i != badGPSSectors.end(); ++i)
  {
    if (!(*i))
      continue;
    const std::vector<ArPose> vertices = (*i)->getRegionVertices();
    if(vertices.size() > 2)
      badGPSSectorVertices.push_back(vertices);
  }
}


void Session::checkLogStats(LogInterface* l)
{
  if((unsigned long) loggedStats.mSecSince() >= AMRISim::log_stats_freq) {
    logStats(l);
  }
}

void Session::logStats(LogInterface *l)
{
  const float tsec = ((float)loggedStats.mSecSince())/1000.0f;
  l->log("Sent %.1f SIP packets/sec (%lu SIP packets), received %.1f packets/sec (%lu packets) in last %.0f sec.", (double)packetsSent/tsec, packetsSent, (double)packetsReceived/tsec, packetsReceived, tsec);
  loggedStats.setToNow();
  packetsSent = 0;
  packetsReceived = 0;
  laserPacketsSent = 0;
  miscPacketsSent = 0;
}

void Session::gotPacket()
{
  ++packetsReceived;
  gotLastCommand.setToNow();
  gotLastValidCommand.setToNow();
}

void Session::sentPacket()
{
  ++packetsSent;
}

void Session::sentLaserPacket()
{
  ++laserPacketsSent;
}

void Session::sentMiscPacket()
{
  ++miscPacketsSent;
}
