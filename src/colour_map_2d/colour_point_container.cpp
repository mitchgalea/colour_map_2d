#include "colour_map_2d/colour_point_container.h"

namespace ColourMap2D{

ColourPointContainer::ColourPointContainer()
{

}

void ColourPointContainer::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::lock_guard<std::mutex> locker(mutex_);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        ColourPoint point(cloud->points[i]);
        if(point.getColour() != ColourLib::Colour::white /*&& points_.size() < MAX_SIZE*/)
        {
            addPoint(point);
        }
    }
}

void ColourPointContainer::addPoint(ColourPoint point)
{
    points_.push_back(point);
}

cv::Mat ColourPointContainer::outputImage(nav_msgs::MapMetaData map_data)
{
    std::lock_guard<std::mutex> locker(mutex_);
    cv::Mat image(map_data.height, map_data.width, CV_8UC3, cv::Scalar(255, 255, 255));

    int radius = 1;
    int thickness = -1;

    for(auto point:points_)
    {
        std::pair<int, int> grid_pos = MapTransform::posetoGrid(point.getPose(), map_data);
        cv::circle(image, cv::Point(grid_pos.first, grid_pos.second), radius, ColourLib::getRGB(point.getColour()),
                   thickness);
    }
    return image;
}
}



