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

		pc_sub_ = pnh.subscribe<sensor_msgs::PointCloud2>("/stereo_inertial_publisher_"+ mxId_ +"/stereo/points_filtered",1,
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

		sensor_msgs::Imu imu_msg;
		imu_class_->getImu(imu_msg, cloud->header.stamp);
		double yaw = 0.0;

		tf::Quaternion q_sensor_FLU(-0.5, 0.5, -0.5, 0.5);

		tf::Quaternion q_imu;
		yaw = am::Rotate::getYaw(imu_msg.orientation);


		q_imu = tf::Quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
		tf::Quaternion q_imu_anti_yaw;
		q_imu_anti_yaw.setRPY(0.0, 0.0, -yaw);
		q_imu = q_imu*q_imu_anti_yaw;
		//q_imu.setRPY(roll, pitch, 0.0);


		tf::Quaternion q_final = q_imu*q_sensor_FLU;
		q_final.normalize();
		geometry_msgs::Quaternion q;
		q.x = q_final.x();
		q.y = q_final.y();
		q.z = q_final.z();
		q.w = q_final.w();

		//ROS_INFO("r: %f, p: %f, y: %f", roll, pitch, yaw);

		for(pcl::PointXYZ &p : pcl_cloud.points)
		{
			double x = p.x;
			double y = p.y;
			double z = p.z;
			//ROS_INFO("before: x: %f, y: %f, z: %f", x,y,z);
			am::Rotate::rotate(x, y, z, q);
			//ROS_INFO("after: x: %f, y: %f, z: %f", x,y,z);
			p.x = x;
			p.y = y;
			p.z = z;

		}

		pcl_cloud.header.frame_id = "sensor_Level_FLU";
		pcl_pub_.publish(pcl_cloud);
	}
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoLevellerNodelet, nodelet::Nodelet)
}
