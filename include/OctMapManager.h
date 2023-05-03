//
// Created by soulde on 2023/4/23.
//

#ifndef FINAL_WS_OCT_MAP_MANAGER_H
#define FINAL_WS_OCT_MAP_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <thread>
#include <mutex>
#include <queue>

class OctMapManager {
public:
    explicit OctMapManager(ros::NodeHandle &nh, tf::TransformListener &listener);

private:
    std::string cloud_in_topic, cloud_out_topic, fix_frame_name, source_frame_name;
    ros::Publisher cloud_pub;
    ros::Subscriber cloud_sub;
    tf::TransformListener &tf_listener;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloud_last;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;
    std::queue<pcl::PointCloud<pcl::PointXYZRGB>> cloud_buf;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    int
    transform(const std::string &in_frame, const sensor_msgs::PointCloud2::ConstPtr &in, const std::string &out_frame,
              sensor_msgs::PointCloud2::Ptr &out);
};


#endif //FINAL_WS_OCT_MAP_MANAGER_H
