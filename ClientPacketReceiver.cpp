

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

//#include "Aria.h"
#include "ArExport.h"
#include "ClientPacketReceiver.h"
#ifndef WIN32
#include <errno.h>
#endif

AREXPORT ClientPacketReceiver::ClientPacketReceiver() :

  myState(STATE_SYNC1),
  myProcessPacketCB(NULL),
  myQuiet(false),
  mySocket(NULL),
  myReadDataCount(0),
  myReadLength(0),
  myReadCommand(0),
  mySync1(0xFA),
  mySync2(0xFB),
  myLoggingPrefix("AMRISim ClientPacketReceiver: ")
{
  memset(myReadBuff, 0, BUFFER_SIZE);
}

AREXPORT ClientPacketReceiver::~ClientPacketReceiver()
{

}

/**
   Sets the socket that this receiver will use, note that it does not
   transfer ownership of the socket.  

   @param socket the socket to use for receiving data
**/
AREXPORT void ClientPacketReceiver::setSocket(ArSocket *socket)
{
  mySocket = socket;
}

AREXPORT void ClientPacketReceiver::setLoggingPrefix(
	const char *loggingPrefix)
{
  if (loggingPrefix != NULL && loggingPrefix[0] != '\0')
    myLoggingPrefix = loggingPrefix;
  else
    myLoggingPrefix.clear();
}

/**
   Gets the socket that the receiver is using, note that it does not
   have ownership of this socket and that whatever created it should.
**/
AREXPORT ArSocket *ClientPacketReceiver::getSocket(void)
{
  return mySocket;
}

#if 0
/**
   This reads in the available data in the TCP port so that packets
   can be fetched with getPacket.  If an error occurs that is
   unrecoverable false will be returned which means that the socket
   owner who is calling this should close the socket and clean up.

   @return false on an error that should cause the socket to be closed,
   true with no errors
**/
AREXPORT bool ClientPacketReceiver::OLDreadData(int msWait)
{
  Ret ret;

  while (1)
  {
    ret = OLDreadPacket(msWait);
    if (ret == RET_TIMED_OUT)
    {
      if (!myQuiet)
	ArLog::log(ArLog::Terse, "%sReadTcp timed out",
		   myLoggingPrefix.c_str());
      return true;
    }
    else if (ret == RET_CONN_CLOSED)
    {
      if (!myQuiet)
	ArLog::log(ArLog::Terse, "%sConnection to %s closed",
		   myLoggingPrefix.c_str(), mySocket->getIPString());
      return false;
    }
    else if (ret == RET_FAILED_READ)
    {
      // if neither of these true's trap it the false is returned cause it was some bad error
#ifdef WIN32
      if (WSAGetLastError() == WSAEWOULDBLOCK)
	return true;
      else if (!myQuiet)
	ArLog::log(ArLog::Terse, "%sFailed on read TCP, error %d", myLoggingPrefix.c_str(), WSAGetLastError());
#endif
#ifndef WIN32
      if (errno == EAGAIN)
	return true;
      else if (!myQuiet)
	ArLog::log(ArLog::Terse, "%sFailed on read TCP, error %d", myLoggingPrefix.c_str(), errno);
#endif
      if (!myQuiet)
	ArLog::log(ArLog::Terse, "%sFailed on the tcp read", myLoggingPrefix.c_str());
      return false;
    }
    else if (ret == RET_BAD_PACKET)
    {
      // well... there was a bad checksum, keep going
    }
    else if (ret == RET_GOT_PACKET)
    {
      if (myProcessPacketCB != NULL)
      {
	//printf("Got in a packet of %d\n", myPacket.getCommand());
  //myPacket.setPacketSource(ArRobotPacket::TCP);
	myProcessPacketCB->invoke(&myPacket);
      }
    }
    else
    {
      if (!myQuiet)
  ArLog::log(ArLog::Terse, "%sClientPacketReceiver: bad Ret value %d", myLoggingPrefix.c_str(), ret);
      return false;
    }
  }
}
#endif

bool ClientPacketReceiver::readData(unsigned int msWait)
{
  int n = mySocket->read(this->myReadBuff, BUFFER_SIZE, msWait);
  //printf("ClientPacketReceiver: mySocket->read() returned %d. myState=%d\n", n, myState);
  if(n == -1)
  {
    // No data
    return true;
  }
  if(n == 0)
  {
    // Error or closed.
    return false;
  }

  // Scan myReadBuf for next expected value, storing it. If end of packet is reached, call callback.
  for(int i = 0; i < n; ++i)
  {
    const unsigned char c = myReadBuff[i]; // must be unsigned char
    switch(myState)
    {
      case STATE_SYNC1:
      {
        if(c == mySync1)
        {
          myPacket.setLength(0);
          myReadDataCount = 0;
          myPacket.uByteToBuf(c); // echo back SYNC1 byte
          myState = STATE_SYNC2;
        }
        //else
        //  printf("expected SYNC1 0x%x, but found 0x%x ('%c) instead.\n", mySync1, c, c);
        break;
      }
      case STATE_SYNC2:
      {
        if(c == mySync2)
        {
          myPacket.uByteToBuf(c); // echo back SYNC2 byte
          myState = STATE_LENGTH;
        }
        else
          myState = STATE_SYNC1; // SYNC2 did not follow SYNC1, start looking for SYNC1 again.
        break;
      }
      case STATE_LENGTH:
      {
        myReadLength = (unsigned int)c & 0xff;
        if(myReadLength > MAX_PACKET_SIZE)
        {
          ArLog::log(ArLog::Normal, "%sWarning: Illegal length in packet: %u (must be less than %u). Scanning for next packet...", myLoggingPrefix.c_str(), myReadLength, MAX_PACKET_SIZE+1);
          myState = STATE_SYNC1;
        }
        myPacket.uByteToBuf(c); // echo back byte
        myState = STATE_DATA;
        break;
      }
      case STATE_DATA:
      {
        // put rest of data <= myReadLength into packet.
        const int avail = n-i;
        const int rest = ArUtil::findMin(avail, myReadLength);
        //printf("STATE_ACQUIRE_DATA: myReadDataCount=%d, i=%d, n=%d, avail=n-i=%d, myReadLength=%d. will take %d bytes of data for this packet.\n", myReadDataCount, i, n, n-i, myReadLength, rest);
        myPacket.dataToBuf(myReadBuff+i, rest);
        i += rest-1; // i will be incremented in for loop
        myReadDataCount += rest;
        // if we read the whole packet, call callback and reset myPacket.
        if(myReadDataCount == myReadLength)
        {
          //puts("finished getting all packet data.");
          //myPacket.log();
          if(myPacket.verifyCheckSum())
          {
            myPacket.resetRead();
            if(myProcessPacketCB)
              myProcessPacketCB->invoke(&myPacket);
          }
          else
          {
            ArLog::log(ArLog::Normal, "%sWarning: received %d packet with bad checksum.", this->myLoggingPrefix.c_str(), myPacket.getID());
          }
          myPacket.empty();
          myState = STATE_SYNC1;
          myReadDataCount = 0;
          myReadLength = 0;
        }
        break;
      }
    }//switch state

  }//floor loop over data buffer
  //printf("finished reading buffer.\n");
  return true;
}

#if 0
ClientPacketReceiver::Ret ClientPacketReceiver::OLDreadPacket(int msWait)
{
  long timeToRunFor = -1;
  int numRead = 0;
  bool printing = true;
  int ret = 0;
  unsigned char c = 0;

  ArTime timeDone;
  timeDone.setToNow();
  timeDone.addMSec(msWait);

  //printf("Read packet!\n");
  do
  {
    timeToRunFor = timeDone.mSecTo();
    if (timeToRunFor < 0)
      timeToRunFor = 0;

    if (myState != STATE_DATA)
    {
      c = 0;
      if ((ret = mySocket->read((char *)&c, 1, 0)) == -1)
      {
        //if (myState == STATE_SYNC1)
        {
          puts("ClientPacketReceiver: no data or read error before packet sync, returning.");
          return RET_FAILED_READ;
        }
        //else
        //{
        //  printf("ClientPacketReceiver: no data or read error during packet sync, trying again.");
          //ArUtil::sleep(1);
        //  continue;
        //}
      }
      else if (ret == 0)
      {
        puts("ClientPacketReceiver: socket read returned 0 during packet sync, connection closed.");
        return RET_CONN_CLOSED;
      }
      printf("ClientPacketReader: socket read received byte %d (%c) during packet sync", c, c);
    }

    /*
    if (myState != STATE_ACQUIRE_DATA)
    {
    printf("%d", myState);
    printf(" %d\n", c);
    }
    */
    //else
    //{
    //printf("\n");
    //}
    switch (myState) {
    case STATE_SYNC1:
      if (c == mySync1) // move on, resetting packet
      {
        myState = STATE_SYNC2;
        myPacket.empty();
        myPacket.setLength(0);
        myPacket.uByteToBuf(c);
      }
      else
      {
        if (printing)
          ArLog::log(ArLog::Verbose, "%sBad char in sync1 %d", myLoggingPrefix.c_str(), c);
        return RET_BAD_PACKET;
      }
      break;
    case STATE_SYNC2:
      if (c == mySync2) // move on, adding this byte
      {
        myState = STATE_LENGTH1;
        myPacket.uByteToBuf(c);
      }
      else // go back to beginning, packet hosed
      {
        if (printing)
          ArLog::log(ArLog::Verbose, "%sBad char in sync2 %d, returning to sync1",  
            myLoggingPrefix.c_str(), c);
        myState = STATE_SYNC1;
        return RET_BAD_PACKET;
      }
      break;
    case STATE_LENGTH1:
      myState = STATE_LENGTH2;
      myReadLength = ((unsigned int)c & 0xff);
      myPacket.uByteToBuf(c);
      break;
      /* Only one length byte in a robot packet:
    case STATE_LENGTH2:
      myState = STATE_ACQUIRE_DATA;
      myReadLength += ((unsigned int)c & 0xff) << 8;
      myPacket.uByteToBuf(c);
      myReadDataCount = 0;
      break;
    */
    case STATE_ACQUIRE_DATA:
      if (myReadLength > MAX_PACKET_DATA_SIZE ||
        myReadLength < myPacket.getHeaderLength() + myPacket.getFooterLength())
      {
        if (!myQuiet)
          ArLog::log(ArLog::Normal, 
                     "%sClientPacketReceiver::readPacket: bad packet length, it is %d which is more than max length of %d bytes or less than the minimum %d",
                     myLoggingPrefix.c_str(), myReadLength, 
                     MAX_PACKET_DATA_SIZE,
                     myPacket.getHeaderLength() + myPacket.getFooterLength());
        myState = STATE_SYNC1;
        return RET_BAD_PACKET;
      }

      // here we read until we get as much as we want, OR until
      // we go 100 ms without data... its arbitrary but it doesn't happen often
      // and it'll mean a bad packet anyways
      if (myReadDataCount < myReadLength - 0)
      {
        numRead = mySocket->read(myReadBuff + myReadDataCount,
                                 myReadLength - myReadDataCount - 0);
        //printf("numRead %d myReadLength %d\n", numRead, myReadLength);
        // trap if it wasn't data
        if (numRead == 0)
          return RET_CONN_CLOSED;
        else if (numRead < 0)
          return RET_FAILED_READ;
        // if it was data add it to our number read
        myReadDataCount += numRead;
      }
      if (myReadDataCount > myReadLength - 0)
      {
        if (!myQuiet)
          ArLog::log(ArLog::Terse, 
          "%sRead is greater than it should be at %d > %d", 
          myLoggingPrefix.c_str(), myReadDataCount, myReadLength - 0);
      }
      if (myReadDataCount == myReadLength - 0)
      {
        myPacket.dataToBuf(myReadBuff, myReadDataCount);

        if (myPacket.verifyCheckSum()) 
        {
          myPacket.resetRead();
          // take off the footer from the packets length Variable
          /* put this in if you want to see the packets received
          //printf("Input ");
          myPacket.log();
          */
          // you can also do this next line if you only care about type
          //printf("Input %x\n", myPacket.getCommand());
          //myPacket.log();
          myState = STATE_SYNC1;
          return RET_GOT_PACKET;
        }
        else 
        {
          myPacket.resetRead();
          //if (!myQuiet)
            ArLog::log(ArLog::Normal, 
            "%sClientPacketReceiver::receivePacket: bad packet, bad checksum on packet %d", myLoggingPrefix.c_str(), myPacket.getID());
          myState = STATE_SYNC1;
          return RET_BAD_PACKET;
        }
      }
      break;
    default:
      break;
    }
  } while (timeDone.mSecTo() >= 0 || myState != STATE_SYNC1);

  return RET_TIMED_OUT;

}
#endif

/**
   @param functor the callback to use when a packet needs to be processed
**/
AREXPORT void ClientPacketReceiver::setProcessPacketCB(
  ArFunctor1<ArRobotPacket *> *functor)
{
  myProcessPacketCB = functor;
}

/**
   @return the callback used when a packet needs to be processed
**/
AREXPORT ArFunctor1<ArRobotPacket *> *ClientPacketReceiver::getProcessPacketCB(void)
{
  return myProcessPacketCB;
}


