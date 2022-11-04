//
// Created by jay on 3/11/20.
//

#include "ros/ros.h"

//#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_publisher");

    ros::NodeHandle n;
    ros::NodeHandle nh;
    //publisher for the planning scene
    std::string octomap_path = nh.param("octomap_path", std::string(""));
    ros::Publisher octo_pub = n.advertise<octomap_msgs::Octomap>("octomap", 1);
    ros::Rate loop_rate(1);
    if (octomap_path == ""){
        ROS_ERROR("Octomap Path is invalid");
    }
    else{
        ROS_INFO("Octomap Path is valid");
    }

    while (ros::ok())
    {
        static octomap_msgs::Octomap octomap;
        static bool msgGenerated = false;

        //Turn the octomap .bt file into an octree format, which is needed by BinaryMapToMsg

        if ( msgGenerated == false)
        {

            octomap::OcTree myOctomap(octomap_path);
//               octomap::OcTree myOctomap("/home/jay/data/virtual_obs.bt");
            octomap_msgs::binaryMapToMsg(myOctomap, octomap);
            octomap.header.frame_id = "world";
            msgGenerated = true;
            ROS_INFO("Published Octomap");
            std::cout<<myOctomap.getBBXMin()<<std::endl;
        }
        octo_pub.publish(octomap);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
