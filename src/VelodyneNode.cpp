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

#include "janeth/VelodyneNode.h"

#include <fstream>

#include <diagnostic_updater/publisher.h>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud.h>

#include <libvelodyne/sensor/DataPacket.h>
#include <libvelodyne/sensor/PositionPacket.h>
#include <libvelodyne/com/UDPConnectionServer.h>
#include <libvelodyne/com/SerialConnection.h>
#include <libvelodyne/sensor/AcquisitionThread.h>
#include <libvelodyne/exceptions/IOException.h>
#include <libvelodyne/exceptions/SystemException.h>
#include <libvelodyne/exceptions/InvalidOperationException.h>
#include <libvelodyne/sensor/Converter.h>
#include <libvelodyne/sensor/Calibration.h>
#include <libvelodyne/sensor/Controller.h>
#include <libvelodyne/data-structures/VdynePointCloud.h>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VelodyneNode::VelodyneNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
    _nodeHandle.param<std::string>("frame_id", _frameId,
      "vehicle_velodyne_link");
    _nodeHandle.param<int>("device_port", _devicePort, 2368);
    _nodeHandle.param<std::string>("serial_device", _serialDeviceStr,
      "/dev/ttyUSB0");
    _nodeHandle.param<int>("baud_rate", _serialBaudrate, 115200);
    _nodeHandle.param<double>("retry_timeout", _retryTimeout, 1);
    _nodeHandle.param<double>("dp_min_freq", _dpMinFreq, 10);
    _nodeHandle.param<double>("dp_max_freq", _dpMaxFreq, 200);
    _nodeHandle.param<int>("buffer_capacity", _bufferCapacity, 100000);
    _nodeHandle.param<std::string>("calibration_file", _calibFileName,
      "conf/calib-HDL-64E.dat");
    _nodeHandle.param<int>("spin_rate", _spinRate, 300);
    _nodeHandle.param<std::string>("device_name", _deviceName,
      "Velodyne HDL-64E S2");
    if (_spinRate < (int)Controller::mMinRPM) {
      _spinRate = Controller::mMinRPM;
      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
        "the range [" << Controller::mMinRPM << ", "
        << Controller::mMaxRPM << "]");
    }
    else if (_spinRate > (int)Controller::mMaxRPM) {
      _spinRate = Controller::mMaxRPM;
      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
        "the range [" << Controller::mMinRPM << ", "
        << Controller::mMaxRPM << "]");
    }
    const int queueDepth = 100;
    _pointCloudPublisher =
      _nodeHandle.advertise<sensor_msgs::PointCloud>("point_cloud",
       queueDepth);
    if (_deviceName == "Velodyne HDL-64E S2")
      ros::ServiceServer setRPMService = _nodeHandle.advertiseService(
        "set_rpm", &VelodyneNode::setRPM, this);
    _updater.setHardwareID(_deviceName);
    _updater.add("UDP connection", this, &VelodyneNode::diagnoseUDPConnection);
    _updater.add("Data packet queue", this,
      &VelodyneNode::diagnoseDataPacketQueue);
    if (_deviceName == "Velodyne HDL-64E S2")
      _updater.add("Serial connection", this,
        &VelodyneNode::diagnoseSerialConnection);
    _dpFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "point_cloud", _updater,
      diagnostic_updater::FrequencyStatusParam(&_dpMinFreq, &_dpMaxFreq,
      0.1, 10)));
    _updater.force_update();
  }

  VelodyneNode::~VelodyneNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VelodyneNode::publishDataPacket(const ros::Time& timestamp,
      const DataPacket& dp) {
    VdynePointCloud pointCloud;
    Converter::toPointCloud(dp, *_calibration, pointCloud);
    sensor_msgs::PointCloudPtr rosCloud(new sensor_msgs::PointCloud);
    rosCloud->header.stamp = ros::Time(dp.getTimestamp());
    rosCloud->header.frame_id = _frameId;
    const size_t numPoints = pointCloud.getSize();
    rosCloud->points.reserve(numPoints);
    rosCloud->channels.resize(1);
    rosCloud->channels[0].name = "intensity";
    rosCloud->channels[0].values.reserve(numPoints);
    for (auto it = pointCloud.getPointBegin(); it != pointCloud.getPointEnd();
        ++it) {
      geometry_msgs::Point32 rosPoint;
      rosPoint.x = it->mX;
      rosPoint.y = it->mY;
      rosPoint.z = it->mZ;
      rosCloud->points.push_back(rosPoint);
      rosCloud->channels[0].values.push_back(it->mIntensity);
    }
    _pointCloudPublisher.publish(rosCloud);
    _dpFreq->tick();
  }

  void VelodyneNode::publishPositionPacket(const ros::Time& timestamp,
      const PositionPacket& dp) {
  }

  bool VelodyneNode::setRPM(velodyne_ros::SetRPM::Request& request,
      velodyne_ros::SetRPM::Response& response) {
    _spinRate = request.SpinRate;
    if (_spinRate < (int)Controller::mMinRPM) {
      _spinRate = Controller::mMinRPM;
      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
        "the range [" << Controller::mMinRPM << ", "
        << Controller::mMaxRPM << "]");
    }
    else if (_spinRate > (int)Controller::mMaxRPM) {
      _spinRate = Controller::mMaxRPM;
      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
        "the range [" << Controller::mMinRPM << ", "
        << Controller::mMaxRPM << "]");
    }
    if (_serialConnection != nullptr && _serialConnection->isOpen()) {
      Controller controller(*_serialConnection);
      controller.setRPM(_spinRate);
      response.Response = true;
      return true;
    }
    else {
      response.Response = false;
      return false;
    }
  }

  void VelodyneNode::diagnoseUDPConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_udpConnection != nullptr && _udpConnection->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "UDP connection opened on %d.",
        _udpConnection->getPort());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "UDP connection closed.");
  }

  void VelodyneNode::diagnoseSerialConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_serialConnection != nullptr && _serialConnection->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Serial connection opened on %s.",
        _serialConnection->getDevicePathStr().c_str());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Serial connection closed.");
  }

  void VelodyneNode::diagnoseDataPacketQueue(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_acqThread != nullptr) {
      status.add("Size", _acqThread->getBuffer().getSize());
      status.add("Dropped elements",
        _acqThread->getBuffer().getNumDroppedElements());
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "Acquisition thread running");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Acquisition thread not running");
  }

  void VelodyneNode::spin() {
    std::ifstream calibFile(_calibFileName);
    _calibration.reset(new Calibration());
    calibFile >> *_calibration;
    _udpConnection.reset(new UDPConnectionServer(_devicePort));
    _serialConnection.reset(new SerialConnection(_serialDeviceStr,
      _serialBaudrate));
    if (_serialConnection != nullptr && _serialConnection->isOpen()) {
      Controller controller(*_serialConnection);
      controller.setRPM(_spinRate);
    }
    _acqThread.reset(new AcquisitionThread<DataPacket>(*_udpConnection));
    _acqThread->getBuffer().setCapacity(_bufferCapacity);
    _acqThread->start();
    Timer timer;
    while (_nodeHandle.ok()) {
      try {
        if (!_acqThread->getBuffer().isEmpty()) {
          std::shared_ptr<DataPacket> dp(_acqThread->getBuffer().dequeue());
          const ros::Time timestamp = ros::Time::now();
          publishDataPacket(timestamp, *dp);
        }
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM("IOException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (SystemException& e) {
        ROS_WARN_STREAM("SystemException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (InvalidOperationException& e) {
        ROS_WARN_STREAM("InvalidOperationException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      _updater.update();
      ros::spinOnce();
    }
  }

}
