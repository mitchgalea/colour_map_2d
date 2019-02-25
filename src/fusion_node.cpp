
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>


using namespace cv;
//class FusionNode{

//    ros::NodeHandle nh_;

//public:
//    FusionNode(ros::NodeHandle nh)
//    : nh_(nh)
//    {

//    }

//    ~FusionNode()
//    {}
//};

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

Scalar ScalarHSV2RGB(uint8_t h, uint8_t s, uint8_t v) {
    Mat rgb;
    Mat hsv(1,1, CV_8UC3, Scalar(h,s,v));
    cvtColor(hsv, rgb, CV_HSV2RGB);
    return Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

Scalar ScalarRGB2HSV(uint8_t r, uint8_t g, uint8_t b) {
    Mat hsv;
    Mat rgb(1,1, CV_8UC3, Scalar(r,g,b));
    cvtColor(rgb, hsv, CV_RGB2HSV);
    return Scalar(hsv.data[0], hsv.data[1], hsv.data[2]);
}

int main(int argc, char **argv)
{
  chdir("/home/mitch");

  ros::init(argc, argv, "fusion_node");

  ros::NodeHandle nh;

  namedWindow(window_capture_name);
  namedWindow(window_detection_name);
//   Trackbars to set thresholds for HSV values
  createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
  createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
  createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
  createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
  createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
  createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
  Mat frame, frame_HSV, frame_threshold;

  Scalar hsv = ScalarRGB2HSV(255, 0, 0);

  std::cout << "H: " << hsv[0] << ", S: " << hsv[1] << ", V: " << hsv[2] << std::endl;
  
  std::string path = "Pictures/light.jpg";
  frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);
  while (true) {

      if(frame.empty())
      {
          break;
      }
      // Convert from BGR to HSV colorspace
      cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
      // Show the frames
      imshow(window_capture_name, frame);
      imshow(window_detection_name, frame_threshold);
      char key = (char) waitKey(30);
      if (key == 'q' || key == 27)
      {
          break;
      }
  }

  ros::spin();

  ros::shutdown();

  return 0;
}

