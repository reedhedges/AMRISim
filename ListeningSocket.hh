
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


#ifndef LISTENING_SOCKET_HH_
#define LISTENING_SOCKET_HH_

#include "Aria/ArASyncTask.h"
#include "Aria/ArSocket.h"

class RobotFactory;  // Forward declare the class, so that it can be linked

/** Interface for a class to listen on a port, creating a new robot in the simulator with an attached
 *  EmulatePioneer object for each client that connects.
 */
class ListeningSocket : public ArASyncTask
{
public:
  ListeningSocket() = default;

  // disallow  copying 
  ListeningSocket(const ListeningSocket& other) = delete;
  ListeningSocket& operator=(const ListeningSocket& other) = delete;

  // TODO move could be ok if ArSocket and ArASyncTask components can be moved
  ListeningSocket(ListeningSocket &&old) = delete;
  ListeningSocket &operator=(ListeningSocket &&old) = delete;

  virtual ArSocket *init(int port, RobotFactory *parentFactory, const char *listenAddress = NULL);
  virtual void * runThread(void *arg);

protected:

  /// Performs one cycle of the thread's execution loop.
  virtual void runLoop();

  int myPort;
  ArSocket mySocket;
  RobotFactory *myFactory;
};



#endif /* LISTENING_SOCKET_HH_ */
