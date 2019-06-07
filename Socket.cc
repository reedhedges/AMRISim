

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
#include "Socket.hh"
#include <sys/stat.h>
#include <unistd.h>

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/select.h>
#include <sys/types.h>
#endif


using namespace AMRISim;

Sockets::SocketsList Sockets::sockets;

void Sockets::addSocketCallback(ArSocket *s, Callback* callback, const std::string& name)
{
  //socketsByFD[s->getFD()] = info;
  //sockets[s] = SocketInfo(s, callback, name);

  sockets.insert(SocketsList::value_type(s, SocketInfo(s, callback, name)));
}

void Sockets::removeSocketCallback(ArSocket *s)
{
 //std::map<int, SocketInfo>::const_iterator fi = socketsByFD.find(s->getFD());
  //if(fi != socketsByFD.end())
  //  socketsByFD.erase(fi);
  std::map<ArSocket*, SocketInfo>::iterator si = sockets.find(s);
  if(si != sockets.end())
  {
    sockets.erase(si);
  }
}


int Sockets::processInput(unsigned int maxTime)
{
  if(sockets.size() == 0)
    return 1;

  struct timeval tv;
  //printf("maxtime=%u\n", maxTime);
  //assert(maxTime < 99999);
  long usec = (long)(((double)maxTime)*1000.0);
  //printf("maxtime usec=%ld\n", usec);
  if(maxTime > 1000)
    tv.tv_sec = (long)floor((double)maxTime/1000.0);
  else
    tv.tv_sec = 0;
  tv.tv_usec = (long)floor((double)(maxTime-tv.tv_sec)*1000.0);
  fd_set rfds;
  int maxfd = 0;
  FD_ZERO(&rfds);
  //print_debug("Sockets::process: Select on sockets:\n");

  for(Sockets::SocketsList::const_iterator i = sockets.begin(); i != sockets.end(); ++i)
  {
    int fd = (*i).second.fd;
#ifndef WIN32
    struct stat sb;
    int st = fstat(fd, &sb);
    //print_debug("\t%d (%s) (fstat=%d)\n", fd, (*i).second.name.c_str(), st);
    if(st == -1)
    {
      print_debug("Skipping invalid socket %d in Sockets::processInput", fd);
    }
    else
#endif
    {
      FD_SET(fd, &rfds);
      maxfd = max(fd, maxfd);
    }
  }

  if(maxfd == 0)
    return 1;   // no valid sockets in sockets list

  //printf(" (maxFD=%d, tv_sec=%ld, tv_usec=%ld)\n", maxfd, tv.tv_sec, tv.tv_usec);
  //printf("select... maxFD=%d, maxTime=%d, sockets.size=%d\n", maxfd, maxTime, sockets.size()   ); fflush(stdout);
  ArTime t;
  int n = select(maxfd+1, &rfds, NULL, NULL, &tv); //NULL);// &tv);
  //print_debug("...select returned %d after %ld msec\n\n", n, t.mSecSince()); fflush(stdout);
  if(t.mSecSince() < (long)maxTime)
    maxTime -= t.mSecSince();
  else
    maxTime = 0;

  if(n == -1)
  {
    if(errno == EBADF)
    {
      // XXX TEMP -- debugging --
      print_debug("bad file descriptor error from select()...");
      for(Sockets::SocketsList::const_iterator i = sockets.begin(); i != sockets.end(); ++i)
      {
        int fd = (*i).second.fd;
        struct stat sb;
        int s = fstat(fd, &sb);
        print_debug("...%d is invalid (%d)", fd, s);
      }
      // XXX TEMP -- end debugging --

    }
    else if(errno == EINVAL)
      puts("Error: bad nfds or timeout argument to select()");
    else if(errno == EINTR)
      puts("Error: Signal during select()");
    else if(errno == ENOMEM)
      puts("Error: out of memory during select() call");
  }
  if(n <= 0)
    return n;

  // Build a separate list of callbacks to invoke for ready sockets
  // Note these callbacks may modify the main 'sockets' list.
  std::list<Callback*> readycbs;
  for(std::map<ArSocket*, SocketInfo>::const_iterator i = sockets.begin(); n > 0 && i != sockets.end(); ++i)
  {
    if(FD_ISSET((*i).second.fd, &rfds))
    {
      readycbs.push_back((*i).second.callback);
      --n;
    }
  }

  for(std::list<Callback*>::const_iterator i = readycbs.begin(); i != readycbs.end(); ++i)
  {
    Callback *c = (*i);
    //print_debug("invoking socket callback 0x%x", c);
    try
    {
      c->invoke(maxTime/readycbs.size());
    }
    catch(AMRISim::DeletionRequest &r)
    {
      // a callback into some instance (class unknown here) wants that instance to be deleted after processing it
      //print_debug("sockets: got deletion request, handling...");
      r.doDelete();
    }
  }

  return 1;
}

ArSocket* Sockets::newSocket(Callback *callback, const std::string& name)
{
  ArSocket *s = new ArSocket;
  addSocketCallback(s, callback, name);
  return s;
}


void Sockets::deleteSocket(ArSocket *socket)
{
  removeSocketCallback(socket);
  delete socket;
}
