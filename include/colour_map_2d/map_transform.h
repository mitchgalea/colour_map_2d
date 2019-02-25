#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/MapMetaData.h"

#include <utility>
#include <math.h>

#ifndef MAPTRANSFORM_H
#define MAPTRANSFORM_H

namespace ColourMap2D{

class MapTransform
{
private:
    MapTransform();
public:
    static geometry_msgs::Pose2D gridtoPose(int col, int row, nav_msgs::MapMetaData map_data);
    static std::pair<int, int> posetoGrid(geometry_msgs::Pose2D pose, nav_msgs::MapMetaData map_data);
};

}

#endif // MAPTRANSFORM_H
