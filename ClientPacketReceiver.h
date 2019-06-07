

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

#ifndef NLNETPACKETRECEIVERTCP_H
#define NLNETPACKETRECEIVERTCP_H

//#include "Aria.h"
#include "ArRobotPacket.h"
#include "ArSocket.h"

/**
   This class receives packets from a TCP socket, you need to have an
   open socket and give it to the socket with setSocket, then you need
   to set up a callback to process packets with setProcessPacketCB,
   finally call readData which will read in all the data and call the
   processPacketCB.
**/
class ClientPacketReceiver
{
public:
  /// Constructor
  AREXPORT ClientPacketReceiver();
  /// Destructor
  AREXPORT ~ClientPacketReceiver();
  
  /// Sets the socket this receiver uses
  AREXPORT void setSocket(ArSocket *socket);
  /// Gets the socket this receiver uses
  AREXPORT ArSocket *getSocket(void);

  /// Sets the callback for use when a packet is received
  AREXPORT void setProcessPacketCB(ArFunctor1<ArRobotPacket *> *functor);

  /// Gets the callback used when a packet is received
  AREXPORT ArFunctor1<ArRobotPacket *> *getProcessPacketCB(void);

  /// Sets the logging prefix
  AREXPORT void setLoggingPrefix(const char *loggingPrefix);

  /// Reads in all the data available calling the processPacketCB
  AREXPORT bool readData(int msWait = 0);

  /// Sets whether we're quiet about errors or not
  void setQuiet(bool quiet) { myQuiet = quiet; }
  /// Gets whether we're quiet about errors or not
  bool getQuiet(void) { return myQuiet; }
protected:
  enum Ret { 
    RET_CONN_CLOSED, // the connection was closed
    RET_GOT_PACKET, // we got a good packet
    RET_BAD_PACKET, // we got a bad packet (checksum wrong)
    RET_FAILED_READ, // our read failed (no data)
    RET_TIMED_OUT}; // we were reading and timed out
  /// Finds and reads in a single packet from data available to read from socket, returns NULL if not one
  //Ret readPacket(int msWait);

  enum State { STATE_SYNC1, STATE_SYNC2, STATE_LENGTH, STATE_DATA };

  enum {
    MAX_PACKET_DATA_SIZE = 202,
    PACKET_HEADER_SIZE = 3, // 2-byte sync header, one length byte
    PACKET_FOOTER_SIZE = 2, // checksum
    MAX_PACKET_SIZE = MAX_PACKET_DATA_SIZE+PACKET_HEADER_SIZE+PACKET_FOOTER_SIZE,
    BUFFER_SIZE = MAX_PACKET_SIZE*2
  };

  State myState;
  ArFunctor1<ArRobotPacket *> *myProcessPacketCB;
  bool myQuiet;
  ArSocket *mySocket;
  ArTime myLastPacket;
  ArRobotPacket myPacket;


  char myReadBuff[BUFFER_SIZE];  ///< store input data read from socket for processing in readPacket
  int myReadDataCount; ///< How much of the packet data (payload) has been read so far.
  int myReadLength; ///< Length value read from input data
  int myReadCommand; ///< Command ID read from input data
  const unsigned char mySync1;
  const unsigned char mySync2;
  std::string myLoggingPrefix;
};

#endif // NLNETPACKETRECEIVERTCP
