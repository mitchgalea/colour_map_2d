#include "ros/ros.h"
#include "colour_map_2d/grid_cell.h"
#include "colour_map_2d/map_transform.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <vector>
#include <mutex>

#ifndef GRID_H
#define GRID_H

namespace ColourMap2D{

class Grid
{
private:
    std::vector<GridCell> grid_cells_;
    nav_msgs::MapMetaData grid_info_;

    double hit_prob_;
    double miss_prob_;
    int cell_occupied_;

    std::mutex mutex_;

    bool initialized_;
public:
    Grid();
    Grid(double hit_prob, double miss_prob, int cell_occupied);

    ////GETTERS
    nav_msgs::MapMetaData getGridInfo() const;
    bool initialized() const;

    ////METHODS
    void processOGMap(const nav_msgs::OccupancyGrid &og_map);
    void initializeGrid(nav_msgs::MapMetaData og_map_data);
    void proccessPoint(int index, uint8_t r, uint8_t g, uint8_t b);
    void updateImage(cv::Mat &image);
    void initializeMapImage(cv::Mat &image);

};

}
#endif // GRID_H
