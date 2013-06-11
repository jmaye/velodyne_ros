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

#include "VelodyneNode.h"

#include <cmath>

#include <fstream>
#include <sstream>

#include <diagnostic_updater/publisher.h>

#include <boost/shared_ptr.hpp>

#include <snappy.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include <libvelodyne/sensor/DataPacket.h>
#include <libvelodyne/sensor/PositionPacket.h>
#include <libvelodyne/com/UDPConnectionServer.h>
#include <libvelodyne/com/SerialConnection.h>
#include <libvelodyne/sensor/AcquisitionThread.h>
#include <libvelodyne/exceptions/IOException.h>
#include <libvelodyne/exceptions/SystemException.h>
#include <libvelodyne/exceptions/InvalidOperationException.h>
#include <libvelodyne/exceptions/OutOfBoundException.h>
#include <libvelodyne/exceptions/BadArgumentException.h>
#include <libvelodyne/sensor/Converter.h>
#include <libvelodyne/sensor/Calibration.h>
#include <libvelodyne/sensor/Controller.h>
#include <libvelodyne/data-structures/VdynePointCloud.h>

#include "velodyne/TemperatureMsg.h"
#include "velodyne/DataPacketMsg.h"
#include "velodyne/DataChunkMsg.h"
#include "velodyne/LaserDataMsg.h"
#include "velodyne/BinarySnappyMsg.h"

#define GRAV_ACC 9.80665

namespace velodyne {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VelodyneNode::VelodyneNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh),
      _dataPacketCounter(0),
      _positionPacketCounter(0),
      _revolutionPacketCounter(0),
      _lastStartAngle(0),
      _currentPointsPerRevolution(0),
      _lastDPTimestamp(0),
      _lastInterDPTime(0),
      _lastPPTimestamp(0),
      _lastInterPPTime(0) {
    getParameters();
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
    _pointCloudPublisher =
      _nodeHandle.advertise<sensor_msgs::PointCloud>("point_cloud",
      _queueDepth);
    _dataPacketPublisher =
      _nodeHandle.advertise<velodyne::DataPacketMsg>("data_packet",
      _queueDepth);
    _binarySnappyPublisher =
      _nodeHandle.advertise<velodyne::BinarySnappyMsg>("binary_snappy",
        _queueDepth);
    if (_deviceName == "Velodyne HDL-32E") {
      _imuPublisher = _nodeHandle.advertise<sensor_msgs::Imu>("imu",
        _queueDepth);
      _tempPublisher = _nodeHandle.advertise<velodyne::TemperatureMsg>(
        "temperature", _queueDepth);
    }
    if (_deviceName == "Velodyne HDL-64E S2")
      _setRPMService = _nodeHandle.advertiseService("set_rpm",
        &VelodyneNode::setRPM, this);
    _updater.setHardwareID(_deviceName);
    _updater.add("UDP connection DP", this,
      &VelodyneNode::diagnoseUDPConnectionDP);
    if (_deviceName == "Velodyne HDL-32E")
      _updater.add("UDP connection PP", this,
        &VelodyneNode::diagnoseUDPConnectionPP);
    _updater.add("Data packet queue", this,
      &VelodyneNode::diagnoseDataPacketQueue);
    if (_deviceName == "Velodyne HDL-32E")
      _updater.add("Position packet queue", this,
        &VelodyneNode::diagnosePositionPacketQueue);
    if (_deviceName == "Velodyne HDL-64E S2")
      _updater.add("Serial connection", this,
        &VelodyneNode::diagnoseSerialConnection);
    _dpFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "data_packet", _updater,
      diagnostic_updater::FrequencyStatusParam(&_dpMinFreq, &_dpMaxFreq,
      0.1, 10)));
    if (_deviceName == "Velodyne HDL-32E")
      _ppFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
        "imu", _updater,
        diagnostic_updater::FrequencyStatusParam(&_ppMinFreq, &_ppMaxFreq,
        0.1, 10)));
    _updater.force_update();
  }

  VelodyneNode::~VelodyneNode() {
    if (_acqThreadPP)
      _acqThreadPP->interrupt();
    if (_acqThreadDP)
      _acqThreadDP->interrupt();
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VelodyneNode::publishDataPacket(const ros::Time& timestamp,
      const DataPacket& dp) {
    VdynePointCloud pointCloud;
    Converter::toPointCloud(dp, *_calibration, pointCloud, _minDistance,
      _maxDistance);
    sensor_msgs::PointCloudPtr rosCloud(new sensor_msgs::PointCloud);
    rosCloud->header.stamp = timestamp;
    rosCloud->header.frame_id = _frameId;
    rosCloud->header.seq = _dataPacketCounter;
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
    velodyne::DataPacketMsgPtr dataPacketMsg(new velodyne::DataPacketMsg);
    dataPacketMsg->header.stamp = timestamp;
    dataPacketMsg->header.frame_id = _frameId;
    dataPacketMsg->header.seq = _dataPacketCounter;
    for (size_t i = 0; i < DataPacket::mDataChunkNbr; ++i) {
      const DataPacket::DataChunk& dataChunk = dp.getDataChunk(i);
      dataPacketMsg->dataChunks[i].headerInfo = dataChunk.mHeaderInfo;
      dataPacketMsg->dataChunks[i].rotationalInfo = dataChunk.mRotationalInfo;
      for (size_t j = 0; j < DataPacket::DataChunk::mLasersPerPacket; ++j) {
        dataPacketMsg->dataChunks[i].laserData[j].distance =
          dataChunk.mLaserData[j].mDistance;
        dataPacketMsg->dataChunks[i].laserData[j].intensity =
          dataChunk.mLaserData[j].mIntensity;
      }
    }
    _dataPacketPublisher.publish(dataPacketMsg);
    velodyne::BinarySnappyMsgPtr binarySnappyMsg(new velodyne::BinarySnappyMsg);
    binarySnappyMsg->header.stamp = timestamp;
    binarySnappyMsg->header.frame_id = _frameId;
    binarySnappyMsg->header.seq = _dataPacketCounter++;
    std::ostringstream binaryStream;
    dp.writeBinary(binaryStream);
    std::string binaryStreamSnappy;
    snappy::Compress(binaryStream.str().data(),
      binaryStream.str().size(), &binaryStreamSnappy);
    binarySnappyMsg->data.resize(binaryStreamSnappy.size());
    std::copy(binaryStreamSnappy.begin(), binaryStreamSnappy.end(),
      binarySnappyMsg->data.begin());
    _binarySnappyPublisher.publish(binarySnappyMsg);
    _dpFreq->tick();
  }

  void VelodyneNode::publishPositionPacket(const ros::Time& timestamp,
      const PositionPacket& pp) {
    sensor_msgs::ImuPtr imuMsg(new sensor_msgs::Imu);
    imuMsg->header.stamp = timestamp;
    imuMsg->header.frame_id = _frameId;
    imuMsg->header.seq = _positionPacketCounter;
    velodyne::TemperatureMsgPtr tempMsg(new velodyne::TemperatureMsg);
    tempMsg->header.stamp = timestamp;
    tempMsg->header.frame_id = _frameId;
    tempMsg->header.seq = _positionPacketCounter++;
    imuMsg->angular_velocity.x = -pp.getGyro2() * M_PI / 180.0;
    imuMsg->angular_velocity.y = pp.getGyro1() * M_PI / 180.0;
    imuMsg->angular_velocity.z = pp.getGyro3() * M_PI / 180.0;
    imuMsg->linear_acceleration.x = (-pp.getAccel1Y() + pp.getAccel3X()) *
      GRAV_ACC / 2.0;
    imuMsg->linear_acceleration.y = -(pp.getAccel2Y() + pp.getAccel3Y()) *
      GRAV_ACC / 2.0;
    imuMsg->linear_acceleration.z = (pp.getAccel1X() + pp.getAccel2X()) *
      GRAV_ACC / 2.0;
    tempMsg->temperature = (pp.getTemp1() + pp.getTemp2() + pp.getTemp3())
      / 3.0;
    _tempPublisher.publish(tempMsg);
    _imuPublisher.publish(imuMsg);
  }

  bool VelodyneNode::setRPM(velodyne::SetRPM::Request& request,
      velodyne::SetRPM::Response& response) {
    _spinRate = request.spinRate;
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
    response.spinRate = _spinRate;
    if (_serialConnection) {
      Controller controller(*_serialConnection);
      try {
        controller.setRPM(_spinRate);
        response.response = true;
        response.message = "Success";
      }
      catch (const BadArgumentException<size_t>& e) {
        ROS_WARN_STREAM("BadArgumentException: " << e.what());
        response.response = false;
        response.message = e.what();
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM("SystemException: " << e.what());
        response.response = false;
        response.message = e.what();
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM("IOException: " << e.what());
        response.message = e.what();
        response.response = false;
      }
      catch (const OutOfBoundException<size_t>& e) {
        ROS_WARN_STREAM("OutOfBoundException: " << e.what());
        response.message = e.what();
        response.response = false;
      }
    }
    else {
      response.response = false;
      response.message = "No serial connection";
    }
    return true;
  }

  void VelodyneNode::diagnoseUDPConnectionDP(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_udpConnectionDP && _udpConnectionDP->isOpen()) {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "UDP connection opened on %d.",
        _udpConnectionDP->getPort());
    }
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "UDP connection closed on %d.", _devicePortDP);
  }

  void VelodyneNode::diagnoseUDPConnectionPP(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_udpConnectionPP && _udpConnectionPP->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "UDP connection opened on %d.",
        _udpConnectionPP->getPort());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "UDP connection closed on %d.", _devicePortPP);
  }

  void VelodyneNode::diagnoseSerialConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_serialConnection && _serialConnection->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Serial connection opened on %s.",
        _serialConnection->getDevicePathStr().c_str());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
      "Serial connection closed on %s.", _serialDeviceStr.c_str());
  }

  void VelodyneNode::diagnoseDataPacketQueue(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_acqThreadDP) {
      status.add("Size", _acqThreadDP->getBuffer().getSize());
      status.add("Dropped elements",
        _acqThreadDP->getBuffer().getNumDroppedElements());
      status.add("Current points per revolution", _currentPointsPerRevolution);
      status.add("Target points per revolution", _targetPointsPerRevolution);
      status.add("Inter packet time [s]", _lastInterDPTime);
      if (std::fabs(_currentPointsPerRevolution - _targetPointsPerRevolution) >
          0.2 * _targetPointsPerRevolution)
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
          "Acquisition thread running (potential data loss)");
      else
        status.summary(diagnostic_msgs::DiagnosticStatus::OK,
          "Acquisition thread running");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Acquisition thread not running");
  }

  void VelodyneNode::diagnosePositionPacketQueue(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_acqThreadPP) {
      status.add("Size", _acqThreadPP->getBuffer().getSize());
      status.add("Dropped elements",
        _acqThreadPP->getBuffer().getNumDroppedElements());
      status.add("Inter packet time [s]", _lastInterPPTime);
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
    try {
      calibFile >> *_calibration;
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM("IOException: " << e.what());
    }
    if (_deviceName == "Velodyne HDL-64E S2") {
      _serialConnection.reset(new SerialConnection(_serialDeviceStr,
        _serialBaudrate));
      Controller controller(*_serialConnection);
      try {
        controller.setRPM(_spinRate);
      }
      catch (const BadArgumentException<size_t>& e) {
        ROS_WARN_STREAM("BadArgumentException: " << e.what());
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM("SystemException: " << e.what());
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM("IOException: " << e.what());
      }
      catch (const OutOfBoundException<size_t>& e) {
        ROS_WARN_STREAM("OutOfBoundException: " << e.what());
      }
    }
    _udpConnectionDP.reset(new UDPConnectionServer(_devicePortDP));
    _acqThreadDP.reset(new AcquisitionThread<DataPacket>(*_udpConnectionDP));
    _acqThreadDP->getBuffer().setCapacity(_bufferCapacity);
    _acqThreadDP->start();
    if (_deviceName == "Velodyne HDL-32E") {
      _udpConnectionPP.reset(new UDPConnectionServer(_devicePortPP));
      _acqThreadPP.reset(new AcquisitionThread<PositionPacket>(
        *_udpConnectionPP));
      _acqThreadPP->getBuffer().setCapacity(_bufferCapacity);
      _acqThreadPP->start();
    }
    Timer timer;
    while (_nodeHandle.ok()) {
      try {
        if (!_acqThreadDP->getBuffer().isEmpty()) {
          std::shared_ptr<DataPacket> dp(_acqThreadDP->getBuffer().dequeue());
          const double startAngle = Calibration::deg2rad(
            dp->getDataChunk(0).mRotationalInfo /
            (double)DataPacket::mRotationResolution);
          const double endAngle = Calibration::deg2rad(
            dp->getDataChunk(DataPacket::mDataChunkNbr - 1).mRotationalInfo /
            (double)DataPacket::mRotationResolution);
          if ((_lastStartAngle > endAngle || startAngle > endAngle) && _revolutionPacketCounter) {
            _currentPointsPerRevolution = _revolutionPacketCounter * 384;
            _revolutionPacketCounter = 0;
          }
          else {
            _revolutionPacketCounter++;
          }
          _lastStartAngle = startAngle;
          const double timestamp = dp->getTimestamp();
          if (_lastDPTimestamp)
            _lastInterDPTime = timestamp - _lastDPTimestamp;
          _lastDPTimestamp = timestamp;
          publishDataPacket(ros::Time(timestamp), *dp);
        }
        if (_deviceName == "Velodyne HDL-32E" &&
            !_acqThreadPP->getBuffer().isEmpty()) {
          std::shared_ptr<PositionPacket> pp(
            _acqThreadPP->getBuffer().dequeue());
          const double timestamp = pp->getTimestamp();
          if (_lastPPTimestamp)
            _lastInterPPTime = timestamp - _lastPPTimestamp;
          _lastPPTimestamp = timestamp;
          publishPositionPacket(ros::Time(timestamp), *pp);
        }
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM("IOException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM("SystemException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (const InvalidOperationException& e) {
        ROS_WARN_STREAM("InvalidOperationException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      _updater.update();
      ros::spinOnce();
    }
  }

  void VelodyneNode::getParameters() {
    _nodeHandle.param<std::string>("ros/frame_id", _frameId,
      "/velodyne_link");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<int>("udp_connection/device_port_dp", _devicePortDP,
      2368);
    _nodeHandle.param<int>("udp_connection/device_port_pp", _devicePortPP,
      8308);
    _nodeHandle.param<std::string>("serial_connection/serial_device",
      _serialDeviceStr, "/dev/janeth/velodyne");
    _nodeHandle.param<int>("serial_connection/baud_rate", _serialBaudrate,
      115200);
    _nodeHandle.param<double>("connection/retry_timeout", _retryTimeout, 1);
    if (_deviceName == "Velodyne HDL-64E S2") {
      // we should have 3472.166666667 packets per second +- 20%
      _nodeHandle.param<double>("diagnostics/dp_min_freq", _dpMinFreq, 2777.73);
      _nodeHandle.param<double>("diagnostics/dp_max_freq", _dpMaxFreq, 4166.6);
    }
    else {
      // we should have 1736.083333333 packets per second +- 20%
      _nodeHandle.param<double>("diagnostics/dp_min_freq", _dpMinFreq, 1388.87);
      _nodeHandle.param<double>("diagnostics/dp_max_freq", _dpMaxFreq, 2083.30);
    }
    _nodeHandle.param<double>("diagnostics/pp_min_freq", _ppMinFreq, 0.1);
    _nodeHandle.param<double>("diagnostics/pp_max_freq", _ppMaxFreq, 100);
    _nodeHandle.param<int>("sensor/buffer_capacity", _bufferCapacity, 100000);
    _nodeHandle.param<double>("sensor/min_distance", _minDistance, 0.9);
    _nodeHandle.param<double>("sensor/max_distance", _maxDistance, 120);
    _nodeHandle.param<std::string>("sensor/device_name", _deviceName,
      "Velodyne HDL-64E S2");
    if (_deviceName == "Velodyne HDL-64E S2")
      _nodeHandle.param<std::string>("sensor/calibration_file", _calibFileName,
        "conf/calib-HDL-64E.dat");
    else if (_deviceName == "Velodyne HDL-32E")
      _nodeHandle.param<std::string>("sensor/calibration_file", _calibFileName,
        "conf/calib-HDL-32E.dat");
    else
      ROS_ERROR_STREAM("Unknown device: " << _deviceName);
    _nodeHandle.param<int>("sensor/spin_rate", _spinRate, 300);
    if (_deviceName == "Velodyne HDL-64E S2")
      // from the datasheet
      _targetPointsPerRevolution = 20833.0 * 64.0 / (_spinRate / 60.0);
    else
      // from the datasheet
      _targetPointsPerRevolution = 20833.0 * 32.0 / 10.0;
  }

}
