//
// Created by jay on 3/10/20.
//

#ifndef SRC_OCTOMAP_MAP_REPRESENTATION_H
#define SRC_OCTOMAP_MAP_REPRESENTATION_H

#include <core_map_representation_interface/map_representation.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <tf/transform_listener.h>
#include <tflib/tflib.h>


#include <vector>

class OctomapMapRepresentation : public MapRepresentation {

private:

    ros::Subscriber octomap_sub;
    ros::Publisher cloud_map_pub;

    std::shared_ptr<octomap::OcTree> tree ;
    std::shared_ptr<DynamicEDTOctomap> distmap;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud;
    tf::TransformListener* listener;
    std::string target_frame;
    bool debug;
    double max_dist;
    bool have_distmap;

    void octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg);
    bool is_map ;
    void calc_distmap();
    void publish_debug();



public:
    OctomapMapRepresentation();
    ~OctomapMapRepresentation();
    void getLowBounds(double &x, double &y, double &z);
    void getHighBounds(double &x, double &y, double &z);
    bool isvalid(geometry_msgs::Point point);
    virtual std::vector<std::vector<double>> get_values(std::vector< std::vector<geometry_msgs::PointStamped> > trajectories);
    virtual void clear();



};


#endif //SRC_OCTOMAP_MAP_REPRESENTATION_H
