#include "colour_map_2d/map_transform.h"

namespace ColourMap2D{

geometry_msgs::Pose2D MapTransform::gridtoPose(int col, int row, nav_msgs::MapMetaData map_data)
{
    geometry_msgs::Pose2D pose;
    pose.x = map_data.origin.position.x + col * map_data.resolution;
    pose.y = map_data.origin.position.y + row * map_data.resolution;
    return pose;
}

std::pair<int, int> MapTransform::posetoGrid(geometry_msgs::Pose2D pose, nav_msgs::MapMetaData map_data)
{
    std::pair<int, int> grid_cell;
    int col = round((pose.x - map_data.origin.position.x) / map_data.resolution);
    int row = round((pose.y - map_data.origin.position.y) / map_data.resolution);

    if(col >= 0 && row >= 0)
    {
        grid_cell.first = col;
        grid_cell.second = row;
        return grid_cell;
    }
    else
    {
        grid_cell.first = -1;
        grid_cell.second = -1;
        return grid_cell;
    }
}

}

