#ifndef _NETWORK_DISCOVERY_HH_
#define _NETWORK_DISCOVERY_HH_


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


#include "ariaUtil.h"
#include "ArSocket.h"
#include "ArASyncTask.h"

/**  This object listens for UDP messages on port 30718, and responds
   with some identifiying information to certain requests.  This allows
   clients to search the local network for instances of AMRISim by 
   sending broadcast messages to the network.  For client-side implementation
   see DiscoverWiBox.py in ARIA Python code.

   Discovery requests must consist of four bytes with value 0xf6 (0, 0, 0, 0xf6).
  
   The response packet will be 30 null bytes except byte 3 will be 0xF7, byte
   8 will be 0xFA and byte 9 will be OxFB.  Byte 3 being 0xF7 identifies this
   as a valid response to a request of 0xF6, and 0xFA,0xFB identifies it as
   AMRISim responding.
     
   See <http://wiki.lantronix.com/developer/Lantronix_Discovery_Protocol>.
*/


class NetworkDiscoveryResponder : public virtual ArASyncTask
{
public:
  NetworkDiscoveryResponder(int _port = 30718) : port (_port)
  {
  }

  int getPort() const { return port; }

private:

  int port;
  ArSocket socket;

  void check()
  {
    char request[4];
    sockaddr_in sin;
    while(socket.recvFrom(request, 4, &sin) >= 4)
    {
      //printf("got data %x %x %x %x, want 3 to be %x\n", request[0], request[1], request[2], request[3], (int)0xf6);
      if(request[3] == '\xf6')
      {
        //puts("got 0xf6, sending response");
        char response[30];
        memset(response, 0, 30);
        response[3] = '\xF7';
        response[8] = '\xFA';
        response[9] = '\xFB';
        socket.sendTo(response, 30, &sin);
      }
      ArUtil::sleep(300);
    }
  }


  virtual void *runThread(void *)
  {
    socket.open(port, ArSocket::UDP);
    socket.setNonBlock();
    while(getRunning())
    {
      check();
      ArUtil::sleep(2000);
    }
    socket.close();
    return 0;
  }
};

#endif
