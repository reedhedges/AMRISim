
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

#include "ListeningSocket.hh"
#include "AMRISim.hh"

#include "RobotFactory.hh"

/*
ListeningSocket::ListeningSocket()
{
  //ArLog::log(ArLog::Normal, "ListeningSocket ctor");
}

ListeningSocket::~ListeningSocket()
{
  //ArLog::log(ArLog::Normal, "ListeningSocket dtor");
}
*/

ArSocket *ListeningSocket::init(int port, RobotFactory *parentFactory, const char *listenAddress/* = NULL*/)
{
  myPort = port;
  myFactory = parentFactory;
  if(mySocket.open(myPort, ArSocket::TCP, listenAddress))
  {
    mySocket.setNonBlock();
    mySocket.setReuseAddress();

    //ArLog::log(ArLog::Normal, "ListeningSocket: Opened socket for RobotFactory(%s)", myFactory->getModelName().c_str());
    return &mySocket;
  }
  else
  {
    ArLog::log(ArLog::Normal, "ListeningSocket: Error %d opening socket on port %d for RobotFactory(%s): %s\n",
        mySocket.getError(), myPort, myFactory->getModelName().c_str(), mySocket.getErrorStr().c_str());
    return NULL;
  }
}

void * ListeningSocket::runThread(void *)
{
  threadStarted();

  while (myRunning) {

    runLoop();

    //ArUtil::sleep(1); // TODO: For how long?

  }
  threadFinished();
  return NULL;

} // end method runThread

void ListeningSocket::runLoop()
{
  //ArLog::log(ArLog::Normal, "ListeningSocket::runLoop()");

  //ArLog::log(ArLog::Normal, "ListeningSocket: listening on port (%d), fd = %d", myPort, mySocket.getFD());

  //ArLog::log(ArLog::Normal, "ListeningSocket: about to 'select', blocking");

  fd_set readfds;

  FD_SET( mySocket.getFD(), &readfds );

  int maxfd = mySocket.getFD();

  int activity = select( maxfd + 1, &readfds, NULL, NULL, NULL );

  if (activity < 0)
    ArLog::log(ArLog::Normal, "ListeningSocket: select error");
  if (activity == 0)
    return;

  if( FD_ISSET( mySocket.getFD(), &readfds ) )
  {
    //ArLog::log(ArLog::Normal, "ListeningSocket: Activity on ListeningSocket. Creating new connection", mySocket.getFD());


    ArSocket *clientSocket = new ArSocket;
    if(!mySocket.accept(clientSocket))
    {
      ArLog::log(ArLog::Normal, "ListeningSocket: error accepting client:");
      ArLog::log(ArLog::Normal, mySocket.getErrorStr().c_str());
      delete clientSocket;
      return;
    }
    if(!clientSocket->isOpen())
    {
      ArLog::log(ArLog::Normal, "ListeningSocket: error accepting client: new client socket not open.");
      delete clientSocket;
      return;
    }

    //ArLog::log(ArLog::Normal, "ListeningSocket: Accepted client. Sending to RobotFactory...");

    myFactory->acceptNewClientFromListenerThread(clientSocket);

  }
  else
  {
    //ArLog::log(ArLog::Normal, "ListeningSocket: No activity on ListeningSocket");
  }
}
