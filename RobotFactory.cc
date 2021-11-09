
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

#include <set>
#include <errno.h>

#include "RobotFactory.hh"
#include "RobotInterface.hh"
#include "EmulatePioneer.hh"

#include "AMRISim.hh"
#include "Socket.hh"

#include "Aria/ArASyncTask.h"

#include "ListeningSocket.hh"

ArMutex RobotFactory::myClientSocketsMutex;


//RobotFactory::RobotFactory(const std::string& modelName, bool verbose, const char *listenAddress, const AMRISim::Options *userOpts) :
RobotFactory::RobotFactory(const std::string& modelName, const AMRISim::Options *userOpts, const char *listenAddress) :
  myModelName(modelName),
  //myPort(8101),
  //myVerbose(verbose),
  myListenAddress(listenAddress),
  //mySRISimCompat(false),
  //mySRISimLaserCompat(true),
  //myLogPacketsReceived(false),
  acceptClientCB(this, &RobotFactory::acceptNewClient),
  //myWarnUnsupportedCommands(false)
  myUserOptions(userOpts),
  myListeningSocket(nullptr)
{
  //ArLog::log(ArLog::Normal, "RobotFactory::ctor: modelName = %s, verbose = %d, listenAddress = %s, userOpts = %d\n", modelName.c_str(), (int)verbose, listenAddress, (int)userOpts);

  // For debugging only
  //myCRCumulateiveMSec = 0;
  //myCRNumCreated = 0;
  //myCreateRobotLastXMSec.clear();
  //myCreatedRobotLastXToTrack = 10;
}

ArSocket* RobotFactory::open(int port, const char *listenAddress)
{
  if(listenAddress)
    log("RobotFactory: opening port %d for new connections to address %s", port, listenAddress);
  else if(myListenAddress)
    log("RobotFactory: opening port %d for new connections to address %s", port, myListenAddress);
  else
    log("RobotFactory: opening port %d for new connections", port);

  // Launch ListeningSocket async task
  myListeningSocket = new ListeningSocket;
  ArSocket *facsock = myListeningSocket->init(port, this, listenAddress?listenAddress:myListenAddress);
  if(!facsock)
  {
    delete myListeningSocket;
    return NULL;
  }
  else
  {
    myListeningSocket->runAsync();
    return facsock;
  }


/* - Obsolete code, from when the listening socket was just one client in the RobotFactory's client list, made to wait while other AMRISim routines were running
  if(myListenSocket.open(port, ArSocket::TCP, listenAddress?listenAddress:myListenAddress)) // == true || mySocket.getError() == ArSocket::NoErr)
  {
    myListenSocket.setNonBlock();
    myListenSocket.setReuseAddress();
    // TODO: removing this temporarily so that testing new ListeningSocket doesn't conflict with this socket
    AMRISim::Sockets::addSocketCallback(&myListenSocket, &acceptClientCB, "RobotFactory listening socket (callback to accept clients and create EP objects)");
    print_debug("RobotFactory socket callback functor is 0x%x", &acceptClientCB);
    return &myListenSocket;
  }
  else  
  {
    fprintf(stderr, "Error %d opening socket on port %d for robot factory: %s\n", myListenSocket.getError(), port, myListenSocket.getErrorStr().c_str());
    log("RobotFactory: error opening socket:");
    log(myListenSocket.getErrorStr().c_str());
    return NULL;
  }
*/
}

/*
RobotFactory::~RobotFactory()
{
}
*/

void RobotFactory::acceptNewClient(unsigned int /*maxTime*/)
{
  ArSocket *clientSocket = new ArSocket;
  ArTime timer;

  if(!myListenSocket.accept(clientSocket))
  {
    log("RobotFactory: error accepting client:");
    log(myListenSocket.getErrorStr().c_str());
    delete clientSocket;
    return;
  }

  if(!clientSocket->isOpen())
  {
    log("RobotFactory: error accepting client: new client socket not open.");
    delete clientSocket;
    return;
  }

  log("Accepted client. Creating robot...");

  RobotInterface *ri = createRobot(myModelName, clientSocket->getIPString());
  if(!ri)
  {
    log("Robot factory: Error creating new robot. Closing client socket.");
    clientSocket->close();
    delete clientSocket;
    return;
  }

  //print_debug("RobotFactory: new RobotInterface is 0x%x", ri);
  //log(("RobotFactory: created robot interface "));
  // Note, assumes that EmulatePioneer properly deletes client sockets
  // when told to do so (otherwise leaks memory)
  clientSocket->setNonBlock();
  EmulatePioneer *ep = new EmulatePioneer(ri, myModelName, clientSocket, /*deleteOnDisconnect=*/true, /*deleteClientSocketOnDisconnect=*/true, myUserOptions);
  ep->setSimulatorIdentification("AMRISim", AMRISIM_VERSION);
  //ep->setCommandsToIgnore(myCommandsToIgnore);
  //ep->setVerbose(myVerbose);
  //ep->setSRISimCompat(mySRISimCompat, mySRISimLaserCompat);
  //ep->setLogPacketsReceived(myLogPacketsReceived);
  //ep->setWarnUnsupportedCommands(myWarnUnsupportedCommands);
} 

void RobotFactory::acceptNewClientFromListenerThread(ArSocket *clientSocket, [[maybe_unused]] unsigned int maxTime/*=0*/)
{
  myClientSocketsMutex.lock();
  //ArLog::log(ArLog::Normal, "RobotFactory::acceptNewClientFromListenerThread(): %s factory accepted new client. Adding to myNewClientSockets...", myModelName.c_str());
  myNewClientSockets.push(clientSocket);
  myClientSocketsMutex.unlock();
}

void RobotFactory::createNewRobotsFromClientsList()
{
  // Currently, this method creates one robot if there is at least one client socket
  //  in the pending queue.

  // Testing has shown this process takes ~1.5msec (for a full robot model, probably much less for modelInitByTCP's stub model)
  //   One robot per major update loop results in ~25 robots/sec created. If a higher
  //   creation rate is necessary for mass connections, this method should be looped
  //   with a max time, so more than one robot can be created per major update loop.

  while (true)  // This loop will iterate until the queue is empty. TODO: If this ever starts to take too long (such as during a mass connection event), a time-based exit condition should be added.
  {
    myClientSocketsMutex.lock();

    if (myNewClientSockets.empty())
    {
      //ArLog::log(ArLog::Normal, "RobotFactory::createNewRobotsFromClientsList(): %s factory's myNewClientSockets is empty. Returning...", myModelName.c_str());
      myClientSocketsMutex.unlock();
      return;
    }

    //ArLog::log(ArLog::Normal, "RobotFactory::createNewRobotsFromClientsList(): %s factory's myNewClientSockets has %d clients. Fetching one...", myModelName.c_str(), myNewClientSockets.size());
    ArSocket *clientSocket = myNewClientSockets.front();
    myNewClientSockets.pop();

    myClientSocketsMutex.unlock();

    //ArLog::log(ArLog::Normal, "RobotFactory::createNewRobotsFromClientsList(): %s factory. Creating one robot...", myModelName.c_str());
    RobotInterface *ri = createRobot(myModelName, clientSocket->getIPString());
    if(!ri)
    {
      log("Robot factory: Error creating new robot. Closing client socket.");
      clientSocket->close();
      delete clientSocket;
      return;
    }

    //print_debug("RobotFactory: new RobotInterface is 0x%x", ri);
    //log(("RobotFactory: created robot interface "));
    // Note, assumes that EmulatePioneer properly deletes client sockets
    // when told to do so (otherwise leaks memory)
    clientSocket->setNonBlock();
    EmulatePioneer *ep = new EmulatePioneer(ri, myModelName, clientSocket, /*deleteOnDisconnect=*/true, /*deleteClientSocketOnDisconnect=*/true, myUserOptions);
    ep->setSimulatorIdentification("AMRISim", AMRISIM_VERSION);
    //ep->setCommandsToIgnore(myCommandsToIgnore);
    //ep->setVerbose(myVerbose);
    //ep->setSRISimCompat(mySRISimCompat, mySRISimLaserCompat);
    //ep->setLogPacketsReceived(myLogPacketsReceived);
    //ep->setWarnUnsupportedCommands(myWarnUnsupportedCommands);
  }
}


