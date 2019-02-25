#include "colour_map_2d/colour_point.h"
#include "colour_lib/colour.h"
#include "colour_map_2d/map_transform.h"

#include "nav_msgs/MapMetaData.h"
#include "pcl/point_types.h"
#include <mutex>


#ifndef COLOURPOINTMAP_H
#define COLOURPOINTMAP_H_H

const int MAX_SIZE = 1000;

namespace ColourMap2D{

class ColourPointContainer
{
private:
    std::vector<ColourPoint> points_;
    std::mutex mutex_;

public:
    ColourPointContainer();

    void processPoint(pcl::PointXYZRGB cloud_point);
    void addPoint(ColourPoint point);
    size_t size();

    cv::Mat outputImage(nav_msgs::MapMetaData map_data);

};

}

#endif // COLOURPOINTMAP_H_H
