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
#include <vb_util_lib/rotate.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vb_util_lib/imu_class.h>


namespace depthai_examples
{
class StereoLevellerNodelet : public nodelet::Nodelet
{
public:
	virtual void onInit() override
	{
		ros::NodeHandle pnh = getPrivateNodeHandle();

		getParams(pnh);

		imu_class_ = std::make_shared<am::ImuClass>(pnh, "/stereo_inertial_publisher_" + mxId_ +"/imu");

		pcl_pub_ = pnh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/sensor/"+mxId_+"/pcl",1);

		pc_sub_ = pnh.subscribe<sensor_msgs::PointCloud2>("/stereo_inertial_publisher_"+ mxId_ +"/stereo/points",1,
				&StereoLevellerNodelet::pc2CB, this);

	}


private:

	std::string mxId_;

	ros::Subscriber pc_sub_;

	ros::Publisher pcl_pub_;

	std::shared_ptr<am::ImuClass> imu_class_;

	void getParams(ros::NodeHandle &nh)
	{
		ros::param::get(nodelet::Nodelet::getName()+"/mxId", mxId_);
	}

	//pointcloud2 callback
	void pc2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud)
	{
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

		pcl::fromROSMsg(*cloud, pcl_cloud);

		for(pcl::PointXYZ &p : pcl_cloud.points)
		{
			imu_class_->transform(p.x, p.y, p.z, cloud->header.stamp);
		}

		pcl_pub_.publish(pcl_cloud);
	}
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoLevellerNodelet, nodelet::Nodelet)
}
