#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "UDPConnection.h"
#include "VelodynePacket.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_hdl_64ES2");
	ros::NodeHandle n;
	ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud>("cloud", 16);
	
	// read calibration
	VelodyneCalibration vdyneCalibration;
	ifstream calibFile(argv[2]);
	calibFile >> vdyneCalibration;
	
	// Velodyne connection
	UDPConnection com;
	com.open();
	
	while (ros::ok())
	{
		// get velodyne packet and transform it
		VelodynePacket vdynePacket;
		com >> vdynePacket;
		VelodynePointCloud vdynePointCloud(vdynePacket, vdyneCalibration);
		
		// create ROS point cloud
		const size_t vdynePointCount(vdynePointCloud.mPointCloudVector.size());
		sensor_msgs::PointCloud rosPointCloud;
		rosPointCloud.points.reserve(vdynePointCount);
		for (size_t i = 0; i < vdynePointCount; ++i)
		{
			geometry_msgs::Point32 rosPoint;
			rosPoint.x = vdynePointCloud.mPointCloudVector[i].mf64X;
			rosPoint.y = vdynePointCloud.mPointCloudVector[i].mf64Y;
			rosPoint.z = vdynePointCloud.mPointCloudVector[i].mf64Z;
			rosPointCloud.push_back(rosPoint);
		}
		
		// publish point cloud
		pointCloudPub.publish(rosPointCloud);
		ros::spinOnce();
	}
	
	return 0;
}