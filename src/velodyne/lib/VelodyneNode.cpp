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

#include <fstream>

#include <diagnostic_updater/publisher.h>

#include <boost/shared_ptr.hpp>

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
#include <libvelodyne/sensor/Converter.h>
#include <libvelodyne/sensor/Calibration.h>
#include <libvelodyne/sensor/Controller.h>
#include <libvelodyne/data-structures/VdynePointCloud.h>
#include <libvelodyne/data-structures/VdyneScanCloud.h>

#include "velodyne/TemperatureMsg.h"
#include "velodyne/ScanCloudMsg.h"
#include "velodyne/ScanMsg.h"
#include "velodyne/DataPacketMsg.h"
#include "velodyne/DataChunkMsg.h"
#include "velodyne/LaserDataMsg.h"

#define GRAV_ACC 9.80665

namespace velodyne {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VelodyneNode::VelodyneNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
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
    if (_dataPacketPublish == "point_cloud")
      _pointCloudPublisher =
        _nodeHandle.advertise<sensor_msgs::PointCloud>(_dataPacketPublish,
        _queueDepth);
    else if (_dataPacketPublish == "scan_cloud")
      _scanCloudPublisher =
        _nodeHandle.advertise<velodyne::ScanCloudMsg>(_dataPacketPublish,
        _queueDepth);
    else if (_dataPacketPublish == "data_packet")
      _dataPacketPublisher =
        _nodeHandle.advertise<velodyne::DataPacketMsg>(_dataPacketPublish,
        _queueDepth);
    if (_deviceName == "Velodyne HDL-32E") {
      _imuPublisher = _nodeHandle.advertise<sensor_msgs::Imu>("imu",
        _queueDepth);
      _tempPublisher = _nodeHandle.advertise<velodyne::TemperatureMsg>(
        "temperature", _queueDepth);
    }
//    if (_deviceName == "Velodyne HDL-64E S2")
//      ros::ServiceServer setRPMService = _nodeHandle.advertiseService(
//        "set_rpm", &VelodyneNode::setRPM, this);
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
      _dataPacketPublish, _updater,
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
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VelodyneNode::publishDataPacket(const ros::Time& timestamp,
      const DataPacket& dp) {
    if (_dataPacketPublish == "point_cloud") {
      VdynePointCloud pointCloud;
      Converter::toPointCloud(dp, *_calibration, pointCloud, _minDistance,
        _maxDistance);
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
    }
    else if (_dataPacketPublish == "scan_cloud") {
      VdyneScanCloud scanCloud;
      Converter::toScanCloud(dp, *_calibration, scanCloud, _minDistance,
        _maxDistance);
      velodyne::ScanCloudMsgPtr scanCloudMsg(
        new velodyne::ScanCloudMsg);
      scanCloudMsg->header.stamp = ros::Time(dp.getTimestamp());
      scanCloudMsg->header.frame_id = _frameId;
      const size_t numScans = scanCloud.getSize();
      scanCloudMsg->Scans.reserve(numScans);
      for (auto it = scanCloud.getScanBegin(); it != scanCloud.getScanEnd();
          ++it) {
        velodyne::ScanMsg scan;
        scan.Range = it->mRange;
        scan.Heading = it->mHeading;
        scan.Pitch = it->mPitch;
        scan.Intensity = it->mIntensity;
        scanCloudMsg->Scans.push_back(scan);
      }
      scanCloudMsg->StartAngle = scanCloud.getStartRotationAngle();
      scanCloudMsg->EndAngle = scanCloud.getEndRotationAngle();
      _scanCloudPublisher.publish(scanCloudMsg);
    }
    else if (_dataPacketPublish == "data_packet") {
      velodyne::DataPacketMsgPtr dataPacketMsg(
        new velodyne::DataPacketMsg);
      dataPacketMsg->header.stamp = ros::Time(dp.getTimestamp());
      dataPacketMsg->header.frame_id = _frameId;
      for (size_t i = 0; i < DataPacket::mDataChunkNbr; ++i) {
        const DataPacket::DataChunk& dataChunk = dp.getDataChunk(i);
        dataPacketMsg->DataChunks[i].HeaderInfo = dataChunk.mHeaderInfo;
        dataPacketMsg->DataChunks[i].RotationalInfo = dataChunk.mRotationalInfo;
        for (size_t j = 0; j < DataPacket::DataChunk::mLasersPerPacket; ++j) {
          dataPacketMsg->DataChunks[i].LaserData[j].Distance =
            dataChunk.mLaserData[j].mDistance;
          dataPacketMsg->DataChunks[i].LaserData[j].Intensity =
            dataChunk.mLaserData[j].mIntensity;
        }
      }
      _dataPacketPublisher.publish(dataPacketMsg);
    }
    _dpFreq->tick();
  }

  void VelodyneNode::publishPositionPacket(const ros::Time& timestamp,
      const PositionPacket& pp) {
    sensor_msgs::ImuPtr imuMsg(new sensor_msgs::Imu);
    imuMsg->header.stamp = ros::Time(pp.getTimestamp());
    imuMsg->header.frame_id = _frameId;
    velodyne::TemperatureMsgPtr tempMsg(new velodyne::TemperatureMsg);
    tempMsg->header.stamp = ros::Time(pp.getTimestamp());
    tempMsg->header.frame_id = _frameId;
    imuMsg->angular_velocity.x = -pp.getGyro2() * M_PI / 180.0;
    imuMsg->angular_velocity.y = pp.getGyro1() * M_PI / 180.0;
    imuMsg->angular_velocity.z = pp.getGyro3() * M_PI / 180.0;
    imuMsg->linear_acceleration.x = (-pp.getAccel1Y() + pp.getAccel3X()) *
      GRAV_ACC / 2.0;
    imuMsg->linear_acceleration.y = -(pp.getAccel2Y() + pp.getAccel3Y()) *
      GRAV_ACC / 2.0;
    imuMsg->linear_acceleration.z = (pp.getAccel1X() + pp.getAccel2X()) *
      GRAV_ACC / 2.0;
    tempMsg->Temperature = (pp.getTemp1() + pp.getTemp2() + pp.getTemp3())
      / 3.0;
    _tempPublisher.publish(tempMsg);
    _imuPublisher.publish(imuMsg);
  }

//  bool VelodyneNode::setRPM(velodyne::SetRPM::Request& request,
//      velodyne::SetRPM::Response& response) {
//    _spinRate = request.SpinRate;
//    if (_spinRate < (int)Controller::mMinRPM) {
//      _spinRate = Controller::mMinRPM;
//      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
//        "the range [" << Controller::mMinRPM << ", "
//        << Controller::mMaxRPM << "]");
//    }
//    else if (_spinRate > (int)Controller::mMaxRPM) {
//      _spinRate = Controller::mMaxRPM;
//      ROS_WARN_STREAM("VelodyneNode::VelodyneNode(): the RPMs must lie in "
//        "the range [" << Controller::mMinRPM << ", "
//        << Controller::mMaxRPM << "]");
//    }
//    if (_serialConnection != nullptr && _serialConnection->isOpen()) {
//      Controller controller(*_serialConnection);
//      controller.setRPM(_spinRate);
//      response.Response = true;
//      return true;
//    }
//    else {
//      response.Response = false;
//      return false;
//    }
//  }

  void VelodyneNode::diagnoseUDPConnectionDP(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_udpConnectionDP != nullptr && _udpConnectionDP->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "UDP connection opened on %d.",
        _udpConnectionDP->getPort());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "UDP connection closed.");
  }

  void VelodyneNode::diagnoseUDPConnectionPP(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_udpConnectionPP != nullptr && _udpConnectionPP->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "UDP connection opened on %d.",
        _udpConnectionPP->getPort());
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
    if (_acqThreadDP != nullptr) {
      status.add("Size", _acqThreadDP->getBuffer().getSize());
      status.add("Dropped elements",
        _acqThreadDP->getBuffer().getNumDroppedElements());
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "Acquisition thread running");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Acquisition thread not running");
  }

  void VelodyneNode::diagnosePositionPacketQueue(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_acqThreadPP != nullptr) {
      status.add("Size", _acqThreadPP->getBuffer().getSize());
      status.add("Dropped elements",
        _acqThreadPP->getBuffer().getNumDroppedElements());
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
    _udpConnectionDP.reset(new UDPConnectionServer(_devicePortDP));
    _serialConnection.reset(new SerialConnection(_serialDeviceStr,
      _serialBaudrate));
    if (_serialConnection != nullptr && _serialConnection->isOpen()) {
      Controller controller(*_serialConnection);
      controller.setRPM(_spinRate);
    }
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
          const ros::Time timestamp = ros::Time::now();
          publishDataPacket(timestamp, *dp);
        }
        if (_deviceName == "Velodyne HDL-32E" &&
            !_acqThreadPP->getBuffer().isEmpty()) {
          std::shared_ptr<PositionPacket> pp(
            _acqThreadPP->getBuffer().dequeue());
          const ros::Time timestamp = ros::Time::now();
          publishPositionPacket(timestamp, *pp);
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

  void VelodyneNode::getParameters() {
    _nodeHandle.param<std::string>("ros/frame_id", _frameId,
      "vehicle_velodyne_link");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<int>("udp_connection/device_port_dp", _devicePortDP,
      2368);
    _nodeHandle.param<int>("udp_connection/device_port_pp", _devicePortPP,
      8308);
    _nodeHandle.param<std::string>("serial_connection/serial_device",
      _serialDeviceStr, "/dev/ttyUSB0");
    _nodeHandle.param<int>("serial_connection/baud_rate", _serialBaudrate,
      115200);
    _nodeHandle.param<double>("connection/retry_timeout", _retryTimeout, 1);
    _nodeHandle.param<double>("diagnostics/dp_min_freq", _dpMinFreq, 10);
    _nodeHandle.param<double>("diagnostics/dp_max_freq", _dpMaxFreq, 200);
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
    _nodeHandle.param<std::string>("sensor/data_packet_publish",
      _dataPacketPublish, "point_cloud");
    if (_dataPacketPublish != "point_cloud" &&
       _dataPacketPublish != "scan_cloud" &&
       _dataPacketPublish != "data_packet")
      ROS_ERROR_STREAM("Unknown publisher: " << _dataPacketPublish);
  }

}
