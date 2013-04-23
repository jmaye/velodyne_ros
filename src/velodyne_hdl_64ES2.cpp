#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <libvelodyne/sensor/AcquisitionThread.h>
#include <libvelodyne/sensor/Calibration.h>
#include <libvelodyne/sensor/Controller.h>
#include <libvelodyne/sensor/DataPacket.h>
#include <libvelodyne/sensor/Converter.h>
#include <libvelodyne/data-structures/VdynePointCloud.h>
#include <libvelodyne/exceptions/IOException.h>
#include <libvelodyne/exceptions/OutOfBoundException.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne");
  // Using the tilde signifies a nodehandle linked to our private namespace
  ros::NodeHandle n("~");
  int returnCode = 0;

  ROS_INFO_STREAM("Bringing up the velodyne");
  // Having the nodehandle outside of the try/catch allows us to use ROS_ERROR for reporting errors
  try 
    {        

      ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud>("/velodyne/cloud", 16);
	
      // read calibration
      Calibration vdyneCalibration;
      string calibrationFileName;
      n.param<string>("/velodyne/calibrationFileName", calibrationFileName, "calib.dat");
      ifstream calibFile(calibrationFileName.c_str());
      if (!calibFile.good())
        {
          cerr << "Cannot open calibration file " << calibrationFileName << endl;
          return 1;
        }
      calibFile >> vdyneCalibration;

      // Set the velodyne parameters
      // Sensor RPMs
      int rpms;
      n.param<int>("spin_rate_rpm", rpms, 0);
      if(rpms != 0)
        {
          if(rpms < 300)
            {
              ROS_WARN_STREAM("The velodyne rpms must be between 300 and 900 (inclusive). The value " << rpms << " is out of range. Setting to 300.");
              rpms = 300;
            }
          if(rpms > 900)
            {
              ROS_WARN_STREAM("The velodyne rpms must be between 300 and 900 (inclusive). The value " << rpms << " is out of range. Setting to 900.");
              rpms = 900;
            }
          
          // Device interface
          std::string ttyIface;
          n.param<std::string>("tty_device_path", ttyIface, "/dev/ttyUSB0");
          
          // Scope this part so it goes away after we set the rpms.
          {
            ROS_DEBUG_STREAM("Setting velodyne to spin at " << rpms << " rpms using serial interface " << ttyIface);
            SerialConnection serialConnection(ttyIface);
            Controller controller(serialConnection);
            controller.setRPM(rpms);
            ROS_DEBUG_STREAM("Successfully set the velodyne rpm parameters");
          }
        } // end if we are setting the laser spin rate

      // Velodyne connection
      UDPConnectionServer connection(2368);
      AcquisitionThread<DataPacket> acqThread(connection);
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
                  ROS_WARN_STREAM("Queued velodyne packets: " << queueDepth);
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

          std::shared_ptr<DataPacket> vdynePacket(acqThread.getBuffer().dequeue());
          ros::Time acqTime(vdynePacket->getTimestamp());
          VdynePointCloud vdynePointCloud;
          Converter::toPointCloud(*vdynePacket, vdyneCalibration, vdynePointCloud);
		
          // create ROS point cloud
          const VdynePointCloud::ConstPointIterator vdyneCloudStart(vdynePointCloud.getPointBegin());
          const VdynePointCloud::ConstPointIterator vdyneCloudEnd(vdynePointCloud.getPointEnd());
          const size_t vdynePointCount(vdyneCloudEnd-vdyneCloudStart);
          
          sensor_msgs::PointCloudPtr rosCloud(new sensor_msgs::PointCloud);
          rosCloud->header.stamp = acqTime;
          rosCloud->header.frame_id = "/velodyne";
          //rosCloud->header.seq = vdynePacket->getSpinCount();
          rosCloud->points.reserve(vdynePointCount);
          rosCloud->channels.resize(1);
          rosCloud->channels[0].name = "intensity";
          rosCloud->channels[0].values.reserve(vdynePointCount);
          for (VdynePointCloud::ConstPointIterator it(vdyneCloudStart); it != vdyneCloudEnd; ++it)
            {
              geometry_msgs::Point32 rosPoint;
              rosPoint.x = it->mX;
              rosPoint.y = it->mY;
              rosPoint.z = it->mZ;
              rosCloud->points.push_back(rosPoint);
              rosCloud->channels[0].values.push_back(it->mIntensity);
            }
		
          // publish point cloud
          pointCloudPub.publish(rosCloud);
          //ros::spinOnce();
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
