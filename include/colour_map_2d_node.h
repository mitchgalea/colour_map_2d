#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "colour_map_2d/grid.h"
#include "colour_map_2d/map_transform.h"

#include <string>
#include <mutex>
#include <thread>
#include <chrono>

#ifndef COLOUR_MAP_2D_NODE_H
#define COLOUR_MAP_2D_NODE_H

#endif // COLOUR_MAP_2D_NODE_H

namespace ColourMap2D{
//ColourMap2D Node class
//Subscribes to Occupancy Grid and Point Cloud
//Publishes Map Image
class ColourMap2DNode{
private:
    //ROS MEMBERS
    ros::NodeHandle nh_;
    ros::Subscriber og_sub_;
    ros::Subscriber pc_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    //MEMBERS
    Grid grid_;
    cv_bridge::CvImage colour_map_image_;
    double map_image_rate_;
    
    //TRANSFER MEMBERS
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;
public:
    //CONSTRUCTOR
    //Constructor initializes subscribers and publishers as well as taking inputs for member variables
    ColourMap2DNode(ros::NodeHandle nh);
    //ROS CALLBACKS
    void ogCallback(const nav_msgs::OccupancyGrid og);
    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud);
    //PUBLISH THREAD
    void mapImageThread();
};
}
