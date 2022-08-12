/*
 * am_ai_localizer.cpp
 *
 *  Created on: Aug 2, 2022
 *      Author: alireza
 */

#include <ros/ros.h>
#include <world_lib/world_model.h>
#include <vb_util_lib/transformer.h>
#include <vb_util_lib/frame_names.h>
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
		if(!loadTransforms())
		{
			ROS_ERROR("%s: Failed to get all the transforms", ros::this_node::getName().c_str());
			return;
		}


		odom_pub_ = nh.advertise<nav_msgs::Odometry>("/debug/feature/odometry",1);

		fsl_sub_ = nh.subscribe<brain_box_msgs::FeatureStatusList>("/feature/search_ids", 1, &OAKD2DLocalizer::fsCB, this);

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

	std::string camera_tf_str_ = "";

	double min_allowed_distance_ = {1.0};

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

	//feature status list
	void fsCB(const brain_box_msgs::FeatureStatusList::ConstPtr &msg)
	{
		bool enable = false;

		for (brain_box_msgs::FeatureStatus fs : msg->features)
		{
			if(fs.feature_id.find("path") != std::string::npos)
			{
				enable = true;
				break;
			}
		}

		if((enable && !enabled_) || (!enable && enabled_))
		{
			enabled_ = enable;
		}

	}

	//get params
	void getParams()
	{
		static bool isFirstRun = true;

		ros::param::get("mxId", mxId_);

		ai_topic_ = "/stereo_inertial_publisher_" + mxId_ +"/color/yolov4_Spatial_detections";

		camera_tf_str_ = "oak_d_"+mxId_;

		ros::param::get(ros::this_node::getName()+"/class_names", class_names_);

		ros::param::get(ros::this_node::getName()+"/feature_list", feature_list_);

		ros::param::get(ros::this_node::getName()+"/min_probability", min_probability_);

		ros::param::get(ros::this_node::getName()+"/max_model_deviation", max_model_deviation_);

		ros::param::get(ros::this_node::getName() + "/min_allowed_distance", min_allowed_distance_);


		if(isFirstRun)
		{
			ros::param::get("/World/model_file", world_file_);
		}
	}

	//a function to load all the transforms
	bool loadTransforms()
	{
		int counter = 20;
		bool result = false;
		while(counter >= 0 && !result)
		{
			result = transformer_.getTransform(body_FLU, camera_tf_str_, camera_to_body_tf_, 1.0, false);
			counter--;
		}

		return result;
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

			//todo: complete the function to produce appropriate feature odometry

			nav_msgs::Odometry odom;
			odom.header = msg->header;
			odom.header.frame_id = Asset_Frame;
			odom.child_frame_id = getFeatureId(detection.position.z, -detection.position.x);
			if(odom.child_frame_id == "")
			{
				continue;
			}
			odom.pose.pose.position.x = detection.position.z;
			odom.pose.pose.position.y = -detection.position.x;

			odom.pose.covariance[0] = 1.0;
			odom.pose.covariance[1] = 1.0;
			
			odom_pub_.publish(odom);
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

	std::string getFeatureId(double x, double y)
	{
		std::string result = "";

		geometry_msgs::TransformStamped tbs_;
		if(!transformer_.getTransform(Asset_Frame, body_FLU, tbs_, 1.0, false))
		{
			ROS_WARN("Could not find transform between Asset_Frame and body_FLU");
			return result;
		}
		tbs_.transform.translation.x += x;
		tbs_.transform.translation.y += y;


		double min_distance = 1000.0;
		int min_idx = -1;
		for(int i = 0; i < wm_.features.surfaces.size(); i++)
		{

			std::string feature_id = wm_.features.surfaces[i].id;
			if(feature_id.find("tree") == std::string::npos)
			{
				continue;
			}

			geometry_msgs::TransformStamped ts_;
			if(!transformer_.getTransform(feature_id, body_FLU, ts_, 1.0, false))
			{
				ROS_WARN("Could not find transform between %s and body_FLU", feature_id.c_str());
				continue;
			}

			//double tf_distance = sqrt(pow(ts_.transform.translation.x,2) + pow(ts_.transform.translation.y,2));

			//double diff = abs(tf_distance - feature_dist);
			double diff = sqrt(pow(tbs_.transform.translation.x - ts_.transform.translation.x,2) + pow(tbs_.transform.translation.y - ts_.transform.translation.y,2));

			if(diff < min_distance && diff < min_allowed_distance_)
			{
				min_distance = diff;
				min_idx = i;
			}
		}

		if(min_idx >= 0)
		{
			result = wm_.features.surfaces[0].id;
		}


		return result;
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


