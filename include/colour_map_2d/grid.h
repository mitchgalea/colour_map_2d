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
    double min_prob_;
    double k_;
    int cell_occupied_;

    bool initialized_;
    
    double neighbourProb(int spaces);
public:
    Grid();
    Grid(double hit_prob, double miss_prob, int cell_occupied, double min_prob, double k);

    ////GETTERS
    nav_msgs::MapMetaData getGridInfo() const;
    bool initialized() const;

    ////METHODS
    void processOGMap(const nav_msgs::OccupancyGrid &og_map);
    void initializeGrid(nav_msgs::MapMetaData og_map_data);
    void proccessPoint(int index, uint8_t r, uint8_t g, uint8_t b);
    void updateImage(cv::Mat &image);
    void initializeMapImage(cv::Mat &image);
    double findMaxProb(int spaces, int index);
    void updateCellNeighbours(unsigned index, int steps);
    void updateGridNeighbours(int steps);
};

}
#endif // GRID_H
