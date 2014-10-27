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

/** \file ping.h
    \brief This file contains a minimal ping function to compute RTT.
  */

#ifndef PING_H
#define PING_H

#include <cstdint>

#include <string>

namespace velodyne {

  /** The ping function returns the RTT to a host.
      \brief Ping function.
    */
  int64_t ping(std::string hostIp, size_t dataLength, double timeout);

  /** The checksum function returns the checksum of the packet.
      \brief Packet checksum.
    */
  uint16_t checksum(uint16_t* input, size_t length);

}

#endif // PING_H
