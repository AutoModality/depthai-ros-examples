/*
 * stereo_leveller_nodelet.cpp
 *
 *  Created on: Jul 6, 2022
 *      Author: alireza
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.h>


namespace depthai_examples
{
class StereoLevellerNodelet : public nodelet::Nodelet
{
public:
	virtual void onInit() override
	{
		ros::NodeHandle pnh = getPrivateNodeHandle();
	}
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoLevellerNodelet, nodelet::Nodelet)
}
