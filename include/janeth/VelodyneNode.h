/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
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

/** \file VelodyneNode.h
    \brief This file defines the VelodyneNode class which implements the
           Velodyne node.
  */

#ifndef VELODYNE_NODE_H
#define VELODYNE_NODE_H

#include <string>
#include <memory>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "velodyne_ros/SetRPM.h"

class UDPConnectionServer;
class SerialConnection;
class DataPacket;
class PositionPacket;
class Calibration;
template <typename T> class AcquisitionThread;

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace janeth {

  /** The class VelodyneNode implements the Velodyne node.
      \brief Velodyne node
    */
  class VelodyneNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    VelodyneNode(const ros::NodeHandle& nh);
    /// Copy constructor
    VelodyneNode(const VelodyneNode& other) = delete;
    /// Copy assignment operator
    VelodyneNode& operator = (const VelodyneNode& other) = delete;
    /// Move constructor
    VelodyneNode(VelodyneNode&& other) = delete;
    /// Move assignment operator
    VelodyneNode& operator = (VelodyneNode&& other) = delete;
    /// Destructor
    virtual ~VelodyneNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Publishes the data packet
    void publishDataPacket(const ros::Time& timestamp, const DataPacket& dp);
    /// Publishes the position packet
    void publishPositionPacket(const ros::Time& timestamp, const PositionPacket&
      dp);
    /// Set RPM service
    bool setRPM(velodyne_ros::SetRPM::Request& request,
      velodyne_ros::SetRPM::Response& response);
    /// Diagnose the UDP connection
    void diagnoseUDPConnection(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Diagnose the serial connection
    void diagnoseSerialConnection(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Diagnose the data packet queue
    void diagnoseDataPacketQueue(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Point cloud publisher
    ros::Publisher _pointCloudPublisher;
    /// Frame ID
    std::string _frameId;
    /// Device name
    std::string _deviceName;
    /// Device Port
    int _devicePort;
    /// UDP connection
    std::shared_ptr<UDPConnectionServer> _udpConnection;
    /// Serial device
    std::string _serialDeviceStr;
    /// Serial device baudrate
    int _serialBaudrate;
    /// Serial connection
    std::shared_ptr<SerialConnection> _serialConnection;
    /// Retry timeout for connection failure
    double _retryTimeout;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Frequency diagnostic for data packet
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _dpFreq;
    /// Data packet minimum frequency
    double _dpMinFreq;
    /// Data packet maximum frequency
    double _dpMaxFreq;
    /// Data buffer capacity
    int _bufferCapacity;
    /// Calibration file name
    std::string _calibFileName;
    /// Calibration structure
    std::shared_ptr<Calibration> _calibration;
    /// RPM of the device
    int _spinRate;
    /// Acquisition thread
    std::shared_ptr<AcquisitionThread<DataPacket> > _acqThread;
    /** @}
      */

  };

}

#endif // VELODYNE_NODE_H
