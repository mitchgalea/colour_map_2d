
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_ros/point_cloud.h>

#include "colour_map_2d/grid_cell_container.h"
#include "colour_map_2d/colour_point_container.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

using namespace ColourMap2D;

class ColourMap2DNode{

    ros::NodeHandle nh_;
    ros::Subscriber og_sub_;
    ros::Subscriber pc_sub_;

    GridCellContainer grid_cells_;
    ColourPointContainer colour_points_;
    nav_msgs::MapMetaData map_data;
public:
    ColourMap2DNode(ros::NodeHandle nh)
    : nh_(nh)
    {
        og_sub_ = nh_.subscribe("/map", 2, &ColourMap2DNode::ogCallback, this);
        pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points/filtered", 2, &ColourMap2DNode::pcCallback, this);

    }

    void ogCallback(const nav_msgs::OccupancyGrid& og)
    {
        grid_cells_.processOGMap(og);
        map_data = og.info;
        ROS_INFO("Occupied Cells: %d", static_cast<int>(grid_cells_.size()));
    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg (*pCloud, *cloud);

        colour_points_.processPointCloud(cloud);
        ROS_INFO("Points Proccessed");
    }

    ~ColourMap2DNode()
    {
        std::string path("image.png");

        cv::Mat image= colour_points_.outputImage(map_data);
        imwrite( path, image);

        std::cout << "image done" << std::endl;
    }

};


int main(int argc, char **argv)
{
  chdir("/home/mitch");
  ros::init(argc, argv, "colour_map_2d_node");

  ros::NodeHandle nh;

  ColourMap2DNode colour_map_2d_node(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}

