#include "colour_map_2d/grid.h"

namespace ColourMap2D {

Grid::Grid():initialized_(false)
{}

Grid::Grid(double hit_prob, double miss_prob, int cell_occupied, double min_prob, double k)
    :hit_prob_(hit_prob), miss_prob_(miss_prob), cell_occupied_(cell_occupied), min_prob_(min_prob), k_(k), initialized_(false)
{}

////GETTERS
nav_msgs::MapMetaData Grid::getGridInfo() const {return grid_info_;}
bool Grid::initialized() const { return initialized_; }

////METHODS
void Grid::processOGMap(const nav_msgs::OccupancyGrid &og_map)
{
    if(og_map.info.width != grid_info_.width || og_map.info.height != grid_info_.height)
    {
        ROS_ERROR("MAP SIZE DIFFERS");
    }
    for(size_t i = 0; i < og_map.data.size(); i++)
    {
        if(og_map.data[i] == cell_occupied_) grid_cells_[i].setOccupied(true);
    }
}

void Grid::initializeGrid(nav_msgs::MapMetaData og_map_data)
{
    grid_info_ = og_map_data;
    for(unsigned row = 0; row < og_map_data.height; row++)
    {
        for(unsigned col = 0; col < og_map_data.width; col++)
        {
            unsigned index = row * og_map_data.width + col;
            grid_cells_.push_back(GridCell(index));
        }
    }
    initialized_ = true;
}

void Grid::proccessPoint(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index > 0)
    {
        grid_cells_[index].processPoint(r, g, b, hit_prob_, miss_prob_);
    }
}

void Grid::updateImage(cv::Mat &image)
{
    std::cout << "image: " << image.rows << " " << image.cols << std::endl;
    std::cout << "grid : " << grid_info_.height << " " << grid_info_.width << std::endl;
    for(int row = 0; row < image.rows; row++)
    {
        for(int col = 0; col < image.cols; col++)
        {
            unsigned index = row * grid_info_.width + col;
            if(!grid_cells_[index].occupied())
            {
                image.at<cv::Vec3b>(cv::Point(col, row)) = cv::Vec3b(255, 255, 255);
            }
            else if(!grid_cells_[index].probChecked())
            {
                std::pair<ColourLib::Colour, double> max_prob = grid_cells_[index].getMaxProb();
                image.at<cv::Vec3b>(cv::Point(col, row)) = ColourLib::getRGB(max_prob.first);
            }

        }
    }
}

void Grid::initializeMapImage(cv::Mat &image)
{
    image = cv::Mat(grid_info_.height, grid_info_.width,  CV_8UC3, cv::Vec3b(255,255,255));
}
    
void Grid::updateCellNeighbours(unsigned index, int steps)
{
    int cell_col = grid_cells_[index].col_;
    int cell_row = grid_cells_[index].row_;
    
    for(int row = cell_row - steps; row < cell_row + steps; row ++)
    {
        for(int col = cell_col - steps; col < cell_col + steps; col ++)
        {
            if(row >= 0 && row <= grid_info_.height && col >=0 && col <= grid_info.width)
            {
                unsigned neighbour_index = row * grid_info_.width + col;
                std::pair<ColourLib::Colour, double> max_prob = grid_cells_[neighbour_index].getMaxProb();
                if(!grid_cells_[neighbour_index].probNeighbour() && max_prob.second > min_prob_)
                {
                    grid_cells_[index].processPoint(ColourLib::getR(max_prob.first), ColourLib::getG(max_prob.first), ColourLib::getB(max_prob.first), 
                                                    neighbourProb(spaces), miss_prob_, true);
                }
            }
        }
    }
}

////PRIVATE METHODS
double Grid::neighbourProb(int spaces)
{
    return miss_prob_ + (hit_prob_ - miss_prob_) * exp(-k_ * static_cast<double>(spaces));
}

}

