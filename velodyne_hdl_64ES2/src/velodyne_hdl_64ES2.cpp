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
	ros::init(argc, argv, "velodyne");
	ros::NodeHandle n;
	ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud>("/velodyne/cloud", 16);
	
	// read calibration
	VelodyneCalibration vdyneCalibration;
	string calibrationFileName;
	n.param<string>("/velodyne/calibrationFileName", calibrationFileName, "calib.dat");
	ifstream calibFile(calibrationFileName.c_str());
	if (!calibFile.good())
	{
		cerr << "Cannot open calibration file " << calibrationFileName << endl;
		return 1;
	}
	calibFile >> vdyneCalibration;
	
	// Velodyne connection
	AcquisitionThread acqThread;
	acqThread.run();
	
	unsigned counter = 0;
	while (ros::ok())
	{
		// show statistics
		//if ((++counter) % 100 == 0)
			ROS_INFO_STREAM("Average packet dropped: " << acqThread.getQueueDroppedPackages() << " %"); 
		// get velodyne packet and transform it
		while (acqThread.getQueueContent() == 0)
		{
			ros::spinOnce();
			usleep(1000);
		}
		boost::shared_ptr<VelodynePacket> vdynePacket(acqThread.getPacket());
		VelodynePointCloud vdynePointCloud(*vdynePacket, vdyneCalibration);
		
		// create ROS point cloud
		const VelodynePointCloud::Point3DVectorConstIterator vdyneCloudStart(vdynePointCloud.getStartIterator());
		const VelodynePointCloud::Point3DVectorConstIterator vdyneCloudEnd(vdynePointCloud.getEndIterator());
		const size_t vdynePointCount(vdyneCloudEnd-vdyneCloudStart);
		
		sensor_msgs::PointCloudPtr rosCloud(new sensor_msgs::PointCloud);
		rosCloud->header.stamp = ros::Time::now();
		rosCloud->header.frame_id = "/velodyne";
		rosCloud->points.reserve(vdynePointCount);
		rosCloud->channels.resize(1);
		rosCloud->channels[0].name = "intensity";
		rosCloud->channels[0].values.reserve(vdynePointCount);
		for (VelodynePointCloud::Point3DVectorConstIterator it(vdyneCloudStart); it != vdyneCloudEnd; ++it)
		{
			geometry_msgs::Point32 rosPoint;
			rosPoint.x = it->mf64X;
			rosPoint.y = it->mf64Y;
			rosPoint.z = it->mf64Z;
			rosCloud->points.push_back(rosPoint);
			rosCloud->channels[0].values.push_back(it->mu8Intensity);
		}
		
		// publish point cloud
		pointCloudPub.publish(rosCloud);
		ros::spinOnce();
	}
	
	return 0;
}
