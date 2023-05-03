//
// Created by soulde on 2023/4/23.
//

#include "OctMapManager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


OctMapManager::OctMapManager(ros::NodeHandle &nh, tf::TransformListener &listener) : cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>), cloud_last(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_out(
        new pcl::PointCloud<pcl::PointXYZRGB>),tf_listener(listener) {
    nh.getParam("/oct_map_manager/cloudInTopic", cloud_in_topic);
    nh.getParam("/oct_map_manager/fixFrameName", fix_frame_name);
    nh.getParam("/oct_map_manager/cloudOutTopic", cloud_out_topic);
    nh.getParam("/oct_map_manager/sourceFrameName", source_frame_name);
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_in_topic, 1,
                                                       [this](auto &&PH1) {
                                                           cloudCallback(std::forward<decltype(PH1)>(PH1));
                                                       });
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_out_topic, 1);


}

void OctMapManager::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::PointCloud2::Ptr cloud_ptr(new sensor_msgs::PointCloud2), cloud_out_msg(new sensor_msgs::PointCloud2);

    int ret = transform(source_frame_name, msg, fix_frame_name, cloud_ptr);
    if (ret == -1) {
        return;
    }

    pcl::fromROSMsg(*cloud_ptr, *cloud);

    pcl::PCLPointCloud2::Ptr cloud_in2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    toPCLPointCloud2(*cloud, *cloud_in2);
    sor.setInputCloud(cloud_in2);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    fromPCLPointCloud2(*cloud_filtered, *cloud);


    icp.setInputSource(cloud);
    icp.setInputTarget(cloud_last);
    icp.align(*cloud_out);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
//    *cloud_full = *cloud + *cloud_full;

    pcl::toROSMsg(*cloud_out, *cloud_out_msg);
    cloud_out_msg->header.frame_id = fix_frame_name;
    cloud_out_msg->header.stamp = msg->header.stamp;
    cloud_out_msg->header.seq = msg->header.seq;

    cloud_pub.publish(cloud_out_msg);
    cloud_last = cloud;
}

int OctMapManager::transform(const std::string &in_frame, const sensor_msgs::PointCloud2::ConstPtr &in,
                             const std::string &out_frame,
                             sensor_msgs::PointCloud2::Ptr &out) {

//out_id就是目标坐标系，如果输入就在目标坐标系那就不用动了
    if (out_frame != in_frame) {
        tf::StampedTransform transform;
        try {
            tf_listener.waitForTransform(out_frame, in_frame, ros::Time(0), ros::Duration(1, 0));
            tf_listener.lookupTransform(out_frame, in_frame, ros::Time(0), transform);
            Eigen::Matrix4f eigen_transform;

            pcl_ros::transformAsMatrix(transform, eigen_transform);

            pcl_ros::transformPointCloud(eigen_transform, *in, *out);

            out->header.frame_id = out_frame;
        }
        catch (
                tf::ExtrapolationException &e
        ) {
            ROS_INFO("%s \nto %s", e.what(), in->header.frame_id.c_str());
            return -1;
        }

    } else {
        *out = *in;
    }
    return 0;
}
