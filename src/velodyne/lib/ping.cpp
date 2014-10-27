/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "ping.h"

#include <cmath>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>

#include <chrono>
#include <iostream>

#include "libvelodyne/exceptions/SystemException.h"
#include "libvelodyne/exceptions/InvalidOperationException.h"
#include "libvelodyne/exceptions/IOException.h"

#define MAXIPLEN 60
#define MAXICMPLEN 76
#define MAX_PACKET_SIZE (65536 - MAXIPLEN - ICMP_MINLEN)
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

namespace velodyne {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  int64_t ping(std::string hostIp, size_t dataLength, double timeout) {
    int s = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (s < 0)
      throw SystemException(errno, "velodyne::ping()::socket()");
    struct sockaddr_in to;
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = inet_addr(hostIp.c_str());
    uint8_t outPacket[MAX_PACKET_SIZE];
    struct icmp* icp = reinterpret_cast<struct icmp*>(outPacket);
    icp->icmp_type = ICMP_ECHO;
    icp->icmp_code = 0;
    icp->icmp_cksum = 0;
    icp->icmp_seq = 12345;
    icp->icmp_id = getpid();
    size_t packetLength = dataLength + ICMP_MINLEN;
    icp->icmp_cksum = checksum(reinterpret_cast<uint16_t*>(icp), packetLength);
    double intPart;
    double fracPart = std::modf(timeout, &intPart);
    struct timeval waitd;
    waitd.tv_sec = intPart;
    waitd.tv_usec = fracPart * 1e6;
    fd_set writeFlags;
    FD_ZERO(&writeFlags);
    FD_SET(s, &writeFlags);
    int res = select(s + 1, reinterpret_cast<fd_set*>(0), &writeFlags,
      reinterpret_cast<fd_set*>(0), &waitd);
    if(res < 0)
      throw SystemException(errno, "velodyne::ping()::select()");
    #if GCC_VERSION > 40604
      std::chrono::time_point<std::chrono::steady_clock> start, end;
    #else
      std::chrono::time_point<std::chrono::monotonic_clock> start, end;
    #endif
    if (FD_ISSET(s, &writeFlags)) {
      FD_CLR(s, &writeFlags);
    #if GCC_VERSION > 40604
      start = std::chrono::steady_clock::now();
    #else
      start = std::chrono::monotonic_clock::now();
    #endif
      res = sendto(s, outPacket, packetLength, 0,
        reinterpret_cast<const struct sockaddr*>(&to), static_cast<socklen_t>(
        sizeof(struct sockaddr_in)));
      if (res < 0 || res != packetLength)
        throw SystemException(errno, "velodyne::ping()::sendto()");
    }
    else
      throw IOException("velodyne::ping(): timeout occured");
    packetLength = dataLength + MAXIPLEN + MAXICMPLEN;
    uint8_t* inPacket = reinterpret_cast<uint8_t*>(malloc(packetLength));
    if (inPacket == 0)
      throw InvalidOperationException("velodyne::ping(): malloc failed");
    fd_set readFlags;
    FD_ZERO(&readFlags);
    FD_SET(s, &readFlags);
    res = select(s + 1, &readFlags, reinterpret_cast<fd_set*>(0),
      reinterpret_cast<fd_set*>(0), &waitd);
    if(res < 0)
      throw SystemException(errno, "velodyne::ping()::select()");
    if (FD_ISSET(s, &readFlags)) {
      FD_CLR(s, &readFlags);
      struct sockaddr_in from;
      socklen_t size;
      res = recvfrom(s, inPacket, packetLength, 0,
        reinterpret_cast<struct sockaddr*>(&from), &size);
      #if GCC_VERSION > 40604
        end = std::chrono::steady_clock::now();
      #else
        end = std::chrono::monotonic_clock::now();
      #endif
      if (res < 0)
        throw SystemException(errno, "velodyne::ping()::recvfrom()");
      struct ip* ip = reinterpret_cast<struct ip*>(inPacket);
      if (res < (sizeof(struct ip) + ICMP_MINLEN))
        throw IOException("velodyne::ping(): packet too short");
      icp = reinterpret_cast<struct icmp*>(inPacket + sizeof(struct ip));
      if (icp->icmp_type == ICMP_ECHOREPLY) {
        if (icp->icmp_seq != 12345)
          throw IOException("velodyne::ping(): wrong sequence number");
        if (icp->icmp_id != getpid())
          throw IOException("velodyne::ping(): wrong pid");
      }
      else
        throw IOException("velodyne::ping(): not an echo reply");
    }
    else
      throw IOException("velodyne::ping(): timeout occured");
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end -
      start).count();
  }

  uint16_t checksum(uint16_t* input, size_t length) {
    uint16_t answer = 0;
    uint32_t sum = 0;
    while (length > 1) {
      sum += *input++;
      length -= 2;
    }
    if (length == 1) {
      *reinterpret_cast<unsigned char*>(&answer) =
        *reinterpret_cast<unsigned char*>(input);
      sum += answer;
    }
    sum = (sum >> 16) + (sum & 0xffff);
    sum += (sum >> 16);
    answer = ~sum;
    return answer;
  }

}
