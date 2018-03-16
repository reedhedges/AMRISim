/*  
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

#ifndef MOBILESIM_SOCKETS_HH
#define MOBILESIM_SOCKETS_HH

#include <ArSocket.h>
#include <ArFunctor.h>
#include <map>
#include <string>

namespace MobileSim
{


/** TODO replace with an ArSocket subclass that takes the callback as an argument, and overrides ctor and dtor to add/remove self from list.??*/
class Sockets
{
public:
  typedef ArFunctor1<unsigned int> Callback;

  class SocketInfo {
  public:
    ArSocket *socket;
    Callback *callback;
    int fd;
    std::string name;
    SocketInfo(ArSocket* _socket, Callback *_cb, const std::string& _name = "") :
        socket(_socket), callback(_cb), fd(_socket->getFD()), name(_name)
    {}
  };


  typedef std::map<ArSocket*, SocketInfo> SocketsList;
  static SocketsList sockets;
//  static std::map<int, SocketInfo> socketsByFD;
  //static fd_set readFDSet;
  //static bool readFDSetInitialized;
  //static int maxFD;
  static void addSocketCallback(ArSocket *s, Callback* callback, const std::string& name = "");
  static void removeSocketCallback(ArSocket *s);
  static ArSocket* newSocket(Callback *callback, const std::string& name = "");
  static void deleteSocket(ArSocket *socket);
  //static void initreadFDSet();
  static int processInput(unsigned int maxTime = 0);
};

} // namespace MobileSim

#endif
