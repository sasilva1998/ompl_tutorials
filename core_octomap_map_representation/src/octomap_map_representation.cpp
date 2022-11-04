//
// Created by jay on 3/10/20.
//
#include <pluginlib/class_list_macros.h>
#include <core_octomap_map_representation/octomap_map_representation.h>


OctomapMapRepresentation::OctomapMapRepresentation()
: debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()){

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap",1,&OctomapMapRepresentation::octomap_callback,this);
//    is_map = false;
//    ROS_INFO("OctoMap Initialized");
    target_frame = nhp.param("octomap_map/target_frame",std::string("octomap_map"));
    debug = nhp.param("octomap_map/debug",false);
    max_dist = nhp.param("octomap_map/max_distance_dt",10.0);
    debug_cloud->header.frame_id = "uav1/map";
    listener = new tf::TransformListener();
    cloud_map_pub = nh.advertise<sensor_msgs::PointCloud2>("debug_points",100);
    have_distmap = false;
    octomap::OcTree* tree = new octomap::OcTree(0.1);

}

void OctomapMapRepresentation::octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg) {
     is_map = true;
     if (!have_distmap) {
         octomap::OcTree* map = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));

         tree = std::shared_ptr<octomap::OcTree>(map);
         ROS_INFO("OctoMap Initialized");
         calc_distmap();
         ROS_INFO("Calculated DT");
         have_distmap = true;
     }


}



void OctomapMapRepresentation::clear() {
    tree->clear();
    ROS_INFO("Map Cleared");

}

void OctomapMapRepresentation::calc_distmap() {


    double x,y,z;
    tree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);

    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);

    bool unknownasoccupied = false;
    distmap = std::make_shared<DynamicEDTOctomap>(max_dist, tree.get(), min, max, unknownasoccupied);
    distmap->update();





}

std::vector<std::vector<double> > OctomapMapRepresentation::get_values(std::vector<std::vector<geometry_msgs::PointStamped> > trajectories) {
    std::vector< std::vector<double> > values(trajectories.size());
    for(int i = 0; i < trajectories.size(); i++)
        //for(int j = 0; j < trajectories[i].waypoints.size(); j++)
        for(int j = 0; j < trajectories[i].size(); j++)
            values[i].push_back(0); //maybe not needed? TODO:check
    for(int i = 0; i < trajectories.size(); i++)
    {
        for(int j = 0; j < trajectories[i].size(); j++) {
            geometry_msgs::PointStamped point_stamped = trajectories[i][j];

            tf::StampedTransform transform;
            listener->waitForTransform(target_frame, point_stamped.header.frame_id, point_stamped.header.stamp, ros::Duration(0.1));
            listener->lookupTransform(target_frame, point_stamped.header.frame_id, point_stamped.header.stamp, transform);
            tf::Vector3 position = transform*tflib::to_tf(point_stamped.point);

//            octomap::point3d p_d (1,0,0);
//            distmap->update();

//            ROS_INFO("%f",distmap->getDistance(p_d));
//            geometry_msgs::PointStamped Point;
//            Point.point.x = 0.0;
//            Point.point.y = 0.0;
//            Point.point.z = 0.0;


//            tf::Vector3 position_d = transform*tflib::to_tf(Point.point);
//            octomap::point3d p_d(point_stamped.point.x,point_stamped.point.y,point_stamped.point.z);
//            ROS_INFO("%s",point_stamped.header.frame_id);
//            ROS_INFO("%f %f %f %f",distmap->getDistance(p_d),p_d.x(),p_d.y(),p_d.z());

            octomap::point3d p(position.x(),position.y(),position.z());
            values[i][j] = distmap->getDistance(p);

            if (values[i][j]>0) {
                pcl::PointXYZRGB debug_point;
//                ROS_INFO("%f %f %f %f", distmap->getDistance(p), position.x(), position.y(), position.z());
                debug_point.x = point_stamped.point.x;
                debug_point.y = point_stamped.point.y;
                debug_point.z = point_stamped.point.z;
                debug_point.r = 255 * (values[i][j] / 2.0);
                debug_point.g = 255 * (values[i][j] / 2.0);
                debug_point.b = 255 * (values[i][j] / 2.0);

                debug_cloud->points.push_back(debug_point);
            }

        }
    }

    return values;
}

void OctomapMapRepresentation::publish_debug() {
//    pcl_conversions::toPCL(ros::Time::now(), debug_cloud->header.stamp);
//    cloud_map_pub.publish(debug_cloud);
//    ROS_INFO("Publishing Debug");
    sensor_msgs::PointCloud2 debug_cloud2;
    pcl::toROSMsg(*debug_cloud, debug_cloud2);
    debug_cloud2.header.frame_id = target_frame;
    cloud_map_pub.publish(debug_cloud2);
    debug_cloud->points.clear();


}

void OctomapMapRepresentation::getLowBounds(double &x, double &y, double &z) {
    tree->getMetricMin(x,y,z);
    ROS_INFO(" Lower Bounds %f %f %f",x,y,z);
    octomap::point3d point(x,y,z);
    tree->setBBXMin(point);


}
void OctomapMapRepresentation::getHighBounds(double &x, double &y, double &z) {
    tree->getMetricMax(x,y,z);
    ROS_INFO(" Upper Bounds %f %f %f",x,y,z);
    octomap::point3d point(x,y,z);
    tree->setBBXMax(point);

}

bool OctomapMapRepresentation::isvalid(geometry_msgs::Point pos) {


    octomap::point3d query(pos.x,pos.y,pos.z);
    auto result = tree->search(query);

    double threshold = 0.5;  //TODO make a param
//    ROS_INFO("Debug1");
//    double x,y,z;
//    tree->getMetricMin(x,y,z);
//    ROS_INFO(" Lower Bounds %f %f %f",x,y,z);
//    std::cout<<tree->getBBXMin()<<" "<<tree->inBBX(query)<<" "<<pos.x<<" "<<pos.y<<std::endl;
    if (result->getOccupancy() >= threshold)
    {
        return false;
    }
    else
    {
        return true;
    }


}

OctomapMapRepresentation::~OctomapMapRepresentation() {
    ROS_INFO("Destructor called");
    tree->clear();
}



PLUGINLIB_EXPORT_CLASS(OctomapMapRepresentation, MapRepresentation)
