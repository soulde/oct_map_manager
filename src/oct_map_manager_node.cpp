//
// Created by soulde on 2023/4/23.
//


#include "OctMapManager.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "oct_map_manager");

    ros::NodeHandle nh;
    tf::TransformListener listener(nh);
    OctMapManager manager(nh, listener);
    while (ros::ok()) {
        ros::spinOnce();
    }
}