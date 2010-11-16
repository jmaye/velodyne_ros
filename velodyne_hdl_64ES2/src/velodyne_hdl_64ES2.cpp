#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "AcquisitionThread.h"
#include "VelodyneCalibration.h"
#include "VelodynePointCloud.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_hdl_64ES2");
	ros::NodeHandle n;
	ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud>("/velodyne/cloud", 16);
	
	// read calibration
	VelodyneCalibration vdyneCalibration;
	string calibrationFileName;
	n.param<string>("calibrationFileName", calibrationFileName, "calib.dat");
	ifstream calibFile(calibrationFileName.c_str());
	calibFile >> vdyneCalibration;
	
	// Velodyne connection
	AcquisitionThread acqThread;
	acqThread.run();
	
	while (ros::ok())
	{
		// get velodyne packet and transform it
		boost::shared_ptr<VelodynePacket> vdynePacket(acqThread.getPacket());
		VelodynePointCloud vdynePointCloud(*vdynePacket, vdyneCalibration);
		
		// create ROS point cloud
		const VelodynePointCloud::Point3DVectorConstIterator vdyneCloudStart(vdynePointCloud.getStartIterator());
		const VelodynePointCloud::Point3DVectorConstIterator vdyneCloudEnd(vdynePointCloud.getEndIterator());
		const size_t vdynePointCount(vdyneCloudEnd-vdyneCloudStart);
		sensor_msgs::PointCloud rosPointCloud;
		rosPointCloud.points.reserve(vdynePointCount);
		for (VelodynePointCloud::Point3DVectorConstIterator it(vdyneCloudStart); it != vdyneCloudEnd; ++it)
		{
			geometry_msgs::Point32 rosPoint;
			rosPoint.x = it->mf64X;
			rosPoint.y = it->mf64Y;
			rosPoint.z = it->mf64Z;
			rosPointCloud.points.push_back(rosPoint);
		}
		
		// publish point cloud
		pointCloudPub.publish(rosPointCloud);
		ros::spinOnce();
	}
	
	return 0;
}
