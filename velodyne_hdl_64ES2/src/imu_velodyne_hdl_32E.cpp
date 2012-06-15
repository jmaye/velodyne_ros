#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "velodyne_hdl_64ES2/Temperature.h"
#include <libvelodyne/sensor/AcquisitionThread.h>
#include <libvelodyne/sensor/PositionPacket.h>
#include <libvelodyne/exceptions/IOException.h>
#include <libvelodyne/exceptions/OutOfBoundException.h>

#include <iostream>
#include <fstream>
#include <string>

#define GRAV_ACC 9.80665		// gravitational acceleration

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_imu");
  // Using the tilde signifies a nodehandle linked to our private namespace
  ros::NodeHandle n("~");
  int returnCode = 0;

  // Having the nodehandle outside of the try/catch allows us to use ROS_ERROR for reporting errors
  try 
    {        
      // Publishers for IMU and temperature
      ros::Publisher imuPub = n.advertise<sensor_msgs::Imu>("/velodyne/imu", 10);
      ros::Publisher tempPub = n.advertise<velodyne_hdl_64ES2::Temperature>("/velodyne/temperature", 10);
	
      // connection
      UDPConnectionServer connection(8308);
      AcquisitionThread<PositionPacket> acqThread(connection);
      acqThread.getBuffer().setCapacity(100000);
      acqThread.start();
	
      unsigned counter = 0;
      int lastDropped=0;
      while (ros::ok())
        {
          // show statistics
          if ((++counter) % 10000 == 0)
            {
              int queueDepth = acqThread.getBuffer().getSize();
              if(queueDepth != 0)
                {
                  ROS_WARN_STREAM("Queued velodyne positioning packets: " << queueDepth);
                }

              int dropped = acqThread.getBuffer().getNumDroppedElements();
              if(dropped != lastDropped)
                {
                  ROS_INFO_STREAM("Number of packets dropped: " << dropped);
                  lastDropped = dropped;
                } 
            }
          // get velodyne packet and transform it
          while (acqThread.getBuffer().isEmpty())
            {
              //ros::spinOnce();
              //usleep(100);
            }

          std::shared_ptr<PositionPacket> vdynePacket(acqThread.getBuffer().dequeue());
          ros::Time acqTime(vdynePacket->getTimestamp());
	  
	  // for testing:
// 	  std::cout << std::fixed << "Time: " << vdynePacket->getTimestamp() << std::endl;
// 	  std::cout << "Gyro 1: " << vdynePacket->getGyro1() << std::endl;
// 	  std::cout << "Gyro 2: " << vdynePacket->getGyro2() << std::endl;
// 	  std::cout << "Gyro 3: " << vdynePacket->getGyro3() << std::endl;
// 	  std::cout << "Temp 1: " << vdynePacket->getTemp1() << std::endl;
// 	  std::cout << "Temp 2: " << vdynePacket->getTemp2() << std::endl;
// 	  std::cout << "Temp 3: " << vdynePacket->getTemp3() << std::endl;
// 	  std::cout << "Accel 1 X: " << vdynePacket->getAccel1X() << std::endl;
// 	  std::cout << "Accel 1 Y: " << vdynePacket->getAccel1Y() << std::endl;
// 	  std::cout << "Accel 2 X: " << vdynePacket->getAccel2X() << std::endl;
// 	  std::cout << "Accel 2 Y: " << vdynePacket->getAccel2Y() << std::endl;
// 	  std::cout << "Accel 3 X: " << vdynePacket->getAccel3X() << std::endl;
// 	  std::cout << "Accel 3 Y: " << vdynePacket->getAccel3Y() << std::endl;
// 	  std::cout << "GPS timestamp: " << vdynePacket->getGPSTimestamp() << std::endl;
	  	
	  // create ROS IMU message
	  sensor_msgs::ImuPtr imuMsg(new sensor_msgs::Imu);
	  imuMsg->header.stamp = acqTime;
	  imuMsg->header.frame_id = "/velodyne";
	  
	  // create ROS temperature message
	  velodyne_hdl_64ES2::TemperaturePtr tempMsg(new velodyne_hdl_64ES2::Temperature);
	  tempMsg->header.stamp = acqTime;
	  tempMsg->header.frame_id = "/velodyne";
	  
	  // GPS time/position: currently not integrated with ROS
	  // TODO?
	  
	  
	  // compute IMU values
	  // angular velocities: convert to radians
	  imuMsg->angular_velocity.x = -vdynePacket->getGyro2()*M_PI/180.0;
	  imuMsg->angular_velocity.y = vdynePacket->getGyro1()*M_PI/180.0;
	  imuMsg->angular_velocity.z = vdynePacket->getGyro3()*M_PI/180.0;
	  // linear accelerations: average and convert to m/s^2
	  imuMsg->linear_acceleration.x = (-vdynePacket->getAccel1Y()+vdynePacket->getAccel3X())*GRAV_ACC/2.0;
	  imuMsg->linear_acceleration.y = -(vdynePacket->getAccel2Y()+vdynePacket->getAccel3Y())*GRAV_ACC/2.0;
	  imuMsg->linear_acceleration.z = (vdynePacket->getAccel1X()+vdynePacket->getAccel2X())*GRAV_ACC/2.0;
	  
	  // compute and publish temperature (averaging)
	  tempMsg->temperature = (vdynePacket->getTemp1() + vdynePacket->getTemp2() + vdynePacket->getTemp3())/3.0;
	  
	  // publish
	  tempPub.publish(tempMsg);
	  imuPub.publish(imuMsg);
        } // end main while loop
    }
  catch(const IOException & e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
      returnCode = -3;
    }
  catch(const std::exception & e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
      returnCode = -1;
    }
  catch(...)
    {
      ROS_ERROR_STREAM("Unknown error during processing");
      returnCode = -2;
    }
	
  return returnCode;
}
