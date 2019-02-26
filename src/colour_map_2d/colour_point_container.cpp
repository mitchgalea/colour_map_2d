#include "colour_map_2d/colour_point_container.h"

namespace ColourMap2D{

ColourPointContainer::ColourPointContainer()
{

}

void ColourPointContainer::processPoint(pcl::PointXYZRGB cloud_point)
{
    std::lock_guard<std::mutex> locker(mutex_);
    ColourPoint point(cloud_point);
    if(point.getColour() != ColourLib::Colour::white)
    {
        addPoint(point);
    }
}

void ColourPointContainer::addPoint(ColourPoint point)
{
    point.print();
    points_.push_back(point);
}

size_t ColourPointContainer::size()
{
    return points_.size();
}

cv::Mat ColourPointContainer::outputImage(nav_msgs::MapMetaData map_data)
{
//    std::lock_guard<std::mutex> locker(mutex_);
//    cv::Mat image(map_data.height, map_data.width, CV_8UC3, cv::Scalar(255, 255, 255));

//    int radius = 1;
//    int thickness = -1;

//    for(auto point:points_)
//    {
//        std::pair<int, int> grid_pos = MapTransform::posetoGrid(point.getPose(), map_data);
//        cv::circle(image, cv::Point(grid_pos.second, grid_pos.first), radius, ColourLib::getRGB(point.getColour()),
//                   thickness);
//    }
//    return image;
}
}



