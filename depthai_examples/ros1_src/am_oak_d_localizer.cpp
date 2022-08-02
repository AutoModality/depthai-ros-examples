/*
 * am_ai_localizer.cpp
 *
 *  Created on: Aug 2, 2022
 *      Author: alireza
 */

#include <ros/ros.h>
#include <world_lib/world_model.h>
#include <nav_msgs/Odometry.h>
#include <brain_box_msgs/FeatureStatusList.h>

std::string NODE_NAME_ ("am_oak_d_localizer");

namespace am
{
class OAKD2DLocalizer
{
public:
	OAKD2DLocalizer(ros::NodeHandle &nh)
	{

		getParams();

		if(mxId_ == "")
		{

			ROS_ERROR("%s: MxId is not set", ros::this_node::getName().c_str());
			return;
		}

		odom_pub_ = nh.advertise<nav_msgs::Odometry>("/feature/odometry",1);

	}
	~OAKD2DLocalizer()
	{
	}
private:

	ros::Publisher odom_pub_;

	ros::Subscriber ai_sub_;

	ros::Subscriber fsl_sub_;

	std::string feature_prefix_ = "tree";

	std::string mxId_ = "";

	std::string ai_topic_ = "";

	bool enabled_ {false};

	void getParams()
	{
		ros::param::get("mxId", mxId_);

		ai_topic_ = "/stereo_inertial_publisher_" + mxId_ +"/color/yolov4_Spatial_detections";

		ros::param::get("feature_prefix", feature_prefix_);
	}

};
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME_);

	ros::NodeHandle nh;

	am::OAKD2DLocalizer oakd_2d_localizer(nh);

	ros::spin();

	return 0;
}


