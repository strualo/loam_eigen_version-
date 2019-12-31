//
// Created by fansa on 2019/12/19.
//

#ifndef SRC_ROS_COMMAN_H
#define SRC_ROS_COMMAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace ros_comman{
	
	template <typename PointT>
	inline void PublishCloudMsg(ros::Publisher& publisher, const pcl::PointCloud<PointT>& cloud, std_msgs::Header header) {
		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(cloud, msg);
		msg.header.stamp = header.stamp;
		msg.header.frame_id = header.frame_id;
		publisher.publish(msg);
	}
} // namespace lio
#endif //SRC_ROS_COMMAN_H
