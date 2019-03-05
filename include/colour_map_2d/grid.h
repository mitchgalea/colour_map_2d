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
//// Grid Class stores a vector of grid cells to represent the Colour Probabilites of the Occupancy Grid
//   Also contains members for histogram filter
class Grid
{
private:
    ////MEMBERS
    std::vector<GridCell> grid_cells_;  //container for grid cells
    nav_msgs::MapMetaData grid_info_;   //information about Occupancy Grid
    mutable std::mutex mutex_;          //mutex

    ////FILTER MEMBERS
    double hit_prob_;                   //histogram filter hit probability
    double miss_prob_;                  //histogram filter miss probability
    double min_prob_;                   //minimum probability for a colour to be used
    double prob_thresh_;                //the low threshold for a probability

    ////CELL STATE MEMBERS
    int cell_obstacle_;                 //og map value for obstacle
    int cell_occupied_;                 //og map value for occupied
    int cell_empty_;                    //og map value for empty
    int cell_unknown_;                  //og map value for unknown
    int frame_;                         //pixels that will be disregarded from each side

    ////SPAWN MEMBERS
    double spawn_noise_;                //noise for spawning new points
    int spawn_rate_;                    //rate of spawning new points
    int spawn_size_;                    //number of new points to be spawned

    ////BOOL MEMBERS
    bool initialized_;                  //boolean containing whether the grid has been initialized
    
    ////PRIVATE METHODS
    double findMaxProb(int spaces, int index);
public:
    ////CONSTRUCTORS
    //basic constructor
    Grid();
    //constructor for all variables
    Grid(double hit_prob, double miss_prob, double min_prob, double prob_thresh,
         int cell_occupied, int cell_obstacle, int cell_empty, int cell_unknown,
         int spawn_rate, double spawn_noise, int spawn_size, int frame);

    ////COPYASSIGNMENT
    Grid& operator = (const Grid& other);

    ////GETTERS
    bool initialized() const;

    ////METHODS
    //method used to process og map
    void processOGMap(const nav_msgs::OccupancyGrid og_map);
    //initializes a grid based on grid info
    void initializeGrid(nav_msgs::MapMetaData og_map_data);
    //processes points from a point cloud
    void processPoints(std::vector<ColourPoint> &points);
    //updates an image for the colour map
    void updateImage(cv::Mat &image);
    //initializes an image for the map
    void initializeMapImage(cv::Mat &image);
    //prints grid info
    void print();
};

}
#endif // GRID_H
