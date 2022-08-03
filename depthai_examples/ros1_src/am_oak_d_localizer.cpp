/*
 * am_ai_localizer.cpp
 *
 *  Created on: Aug 2, 2022
 *      Author: alireza
 */

#include <ros/ros.h>
#include <world_lib/world_model.h>
#include <vb_util_lib/transformer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <brain_box_msgs/FeatureStatusList.h>
#include <depthai_ros_msgs/SpatialDetectionArray.h>

std::string NODE_NAME_ ("am_oak_d_localizer");

namespace am
{
class OAKD2DLocalizer
{
public:
	OAKD2DLocalizer(ros::NodeHandle &nh): transformer_(), wm_(nh)
	{

		getParams();

		if(mxId_ == "")
		{

			ROS_ERROR("%s: MxId is not set", ros::this_node::getName().c_str());
			return;
		}

		if(!wm_.loadModelSpec(world_file_, false, false))
		{
			ROS_ERROR("%s: WorldFile cannot be opened", ros::this_node::getName().c_str());
			return;
		}

		//load the static transforms
		loadTransforms();


		odom_pub_ = nh.advertise<nav_msgs::Odometry>("/feature/odometry",1);

		ai_sub_ = nh.subscribe<depthai_ros_msgs::SpatialDetectionArray>(ai_topic_, 1, &OAKD2DLocalizer::aiCB, this);

	}
	~OAKD2DLocalizer()
	{
	}
private:

	ros::Publisher odom_pub_;

	ros::Subscriber ai_sub_;

	ros::Subscriber fsl_sub_;

	std::string mxId_ = "";

	std::string ai_topic_ = "";

	std::string world_file_ = "";

	double min_probability_ {0.75};

	double max_model_deviation_ {0.5};

	//the list of features to generate odometry for as key, and the minimum threshold for the key as value
	std::vector<std::string> feature_list_;

	//when an object is detected, the message that returns has an id which is an integer >= 0. This refers to the index number in the class_name_
	std::vector<std::string> class_names_;

	//transformer object
	am::Transformer transformer_;

	//worldmodel object
	wld_mod::WorldModel wm_;

	geometry_msgs::TransformStamped camera_to_body_tf_;

	bool enabled_ {false};

	void getParams()
	{
		static bool isFirstRun = true;

		ros::param::get("mxId", mxId_);

		ai_topic_ = "/stereo_inertial_publisher_" + mxId_ +"/color/yolov4_Spatial_detections";

		ros::param::get(ros::this_node::getName()+"/class_names", class_names_);

		ros::param::get(ros::this_node::getName()+"/feature_list", feature_list_);

		ros::param::get(ros::this_node::getName()+"/min_probability", min_probability_);

		ros::param::get(ros::this_node::getName()+"/max_model_deviation", max_model_deviation_);


		if(isFirstRun)
		{
			ros::param::get("/World/model_file", world_file_);
		}
	}

	void loadTransforms()
	{

	}

	//ai boundingbox callback
	void aiCB (const depthai_ros_msgs::SpatialDetectionArray::ConstPtr &msg)
	{
		if(!enabled_)
		{
			return;
		}

		//iterating through the detection
		for(const depthai_ros_msgs::SpatialDetection &detection : msg->detections)
		{
			if(detection.results.size() == 0)
			{
				continue;
			}

			//use the zero index of the result?????
			if(detection.results[0].score < min_probability_ || !isFound(feature_list_, class_names_[detection.results[0].id]))
			{
				continue;
			}
		}
	}

	//isFound function: a function that search through the given array and looks for the key
	bool isFound(const std::vector<std::string> &src, const std::string &key)
	{

		if(std::find(src.begin(), src.end(), key) != src.end())
		{
			return true;
		}

		return false;
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


