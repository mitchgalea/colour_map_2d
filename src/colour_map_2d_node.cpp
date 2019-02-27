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


using namespace ColourMap2D;

class ColourMap2DNode{

    ros::NodeHandle nh_;
    ros::Subscriber og_sub_;
    ros::Subscriber pc_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    Grid grid_;
    cv_bridge::CvImage colour_map_image_;
    double map_image_rate_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;

public:
    ColourMap2DNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), target_frame_("map"), tf_listener_(tf_buffer_)
    {
        og_sub_ = nh_.subscribe("/map", 2, &ColourMap2DNode::ogCallback, this);
        pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points/filtered", 2, &ColourMap2DNode::pcCallback, this);
        image_pub_ = it_.advertise("/colour_map_image", 1);

        double hit_prob;
        double miss_prob;
        double min_prob;
        
        int cell_occupied;
        int cell_obstacle;
        int cell_unknown;
        int cell_empty;
        int frame;
        
        double spawn_noise;
        int spawn_rate;
        int spawn_size;

        ros::NodeHandle pn("~");
        pn.param<double>("rate", map_image_rate_, 1.0);
        pn.param<double>("hit_prob", hit_prob, 0.6);
        pn.param<double>("miss_prob", miss_prob, 0.2);
        pn.param<double>("min_prob", min_prob, 0.7);
        pn.param<int>("cell_occupied", cell_occupied, 50);
        pn.param<int>("cell_occupied", cell_obstacle, 100);
        pn.param<int>("cell_occupied", cell_unknown, -1);
        pn.param<int>("cell_occupied", cell_empty, 0);
        pn.param<int>("spawn_number", spawn_size, 10);
        pn.param<int>("spawn_rate", spawn_rate, 100);
        pn.param<int>("frame", frame, 100);
        pn.param<double>("spawn_noise", spawn_noise, 0.01);

        grid_ = Grid(hit_prob, miss_prob, min_prob, cell_occupied, cell_obstacle, cell_empty, cell_unknown,
                     spawn_rate, spawn_noise, spawn_size, frame);
        colour_map_image_.encoding = sensor_msgs::image_encodings::RGB8;
    }

    void ogCallback(const nav_msgs::OccupancyGrid og)
    {

        if(!grid_.initialized())
        {
            grid_.initializeGrid(og.info);
            grid_.initializeMapImage(colour_map_image_.image);
        }
        grid_.processOGMap(og);
        ROS_INFO("OG Map Proccessed");
    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg (*pCloud, *cloud);
        
        std::vector<ColourPoint> colour_points;
        for(auto point:cloud->points)
        {
            geometry_msgs::PointStamped point_in;
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;
            point_in.header = pCloud->header;
            geometry_msgs::PointStamped point_out;
            
            bool exception = false;
            try 
            {
              tf_buffer_.transform(point_in, point_out, target_frame_);
            }
            catch (tf2::TransformException &ex) 
            {
              //ROS_WARN("Failure %s\n", ex.what());
              exception = true;

            }
            if(!exception)
            {
                colour_points.push_back(ColourPoint(point_out.point.x, point_out.point.y, point.r, point.g, point.b);
                /*geometry_msgs::Pose2D pose;
                pose.x = point_out.point.x;
                pose.y = point_out.point.y;
                int point_index = ColourMap2D::MapTransform::posetoIndex(pose, grid_.getGridInfo());
                grid_.proccessPoint(point_index, point.r, point.g, point.b);*/
            }
            grid_.processPoints(colour_points);
        }

        ROS_INFO("Points Proccessed");
    }

    void mapImageThread()
    {
        ros::Rate rate_limiter(1);

        while(ros::ok())
        {
            if(grid_.initialized())
            {
                grid_.updateImage(colour_map_image_.image);
                ROS_INFO("Proccessed");
                image_pub_.publish(colour_map_image_.toImageMsg());
            }
            rate_limiter.sleep();
        }
    }

    ~ColourMap2DNode()
    {
        imwrite( "image.png", colour_map_image_.image);
    }

};


int main(int argc, char **argv)
{
  chdir("/home/mitch");
  ros::init(argc, argv, "colour_map_2d_node");

  ros::NodeHandle nh;

  ColourMap2DNode colour_map_2d_node(nh);

  std::thread image_thread(&ColourMap2DNode::mapImageThread, std::ref(colour_map_2d_node));

  ros::spin();

  ros::shutdown();

  image_thread.join();

  return 0;
}

