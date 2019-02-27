#include "colour_map_2d/grid.h"

namespace ColourMap2D {

Grid::Grid():initialized_(false)
{}

Grid::Grid(double hit_prob, double miss_prob, double min_prob, double k,
         int cell_occupied, int cell_obstacle, int cell_empty, int cell_unknown,
         int spawn_rate, double spawn_noise, int frame)
    :hit_prob_(hit_prob), miss_prob_(miss_prob), min_prob_(min_prob), k_(k), initialized_(false), frame_(80),
    cell_occupied_(cell_occupied), cell_obstacle_(cell_obstacle), cell_empty_(cell_empty), cell_unknown_(cell_unknown),
    spawn_rate_(spawn_rate), spawn_noise_(spawn_noise), frame_(frame)
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
        if(og_map.data[i] == cell_occupied_)
        {
            grid_cells_[i].setCellState(CellState::occupied);
        }
        else if(og_map.data[i] == cell_obstacle_)
        {
            grid_cells_[i].setCellState(CellState::obstacle);
        }
        else if(og_map.data[i] == cell_empty_)
        {
            grid_cells_[i].setCellState(CellState::empty);
        }
        else
        {
            grid_cells_[i].setCellState(CellState::unknown);
        }
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

void Grid::proccessPoint(int index, uint8_t r, uint8_t g, uint8_t b, int spawn)
{
    if(index > 0)
    {
        grid_cells_[index].processPoint(r, g, b, hit_prob_, miss_prob_);
        if(spawn > 0 && grid_cells_[index].getCellState() == CellState::obstacle)
        {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator (seed);
            std::uniform_int_distribution<int> spawn_distribution(0, spawn_rate_);
            if(spawn_distribution(generator) == spawn_rate_)
            {
                std::normal_distribution<double> noise_distribution(0.0, spawn_noise_);
                geometry_msgs::Pose2D point_pose = MapTransform::indextoPose(index, grid_info_);
                for(int i = 0; i < spawn; i++)
                {
                    geometry_msgs::Pose2D temp_pose;
                    temp_pose.x = point_pose.x + noise_distribution(generator);
                    temp_pose.y = point_pose.y + noise_distribution(generator);
                    unsigned temp_index = MapTransform::posetoIndex(temp_pose);
                    grid_cells_[temp_index].processPoint(r, g, b, hit_prob_, miss_prob_);
                }
            }
        }
    }
}

void Grid::updateImage(cv::Mat &image)
{
    for(int row = 0; row < image.rows; row++)
    {
        for(int col = 0 ; col < image.cols; col++)
        {
            unsigned index = ((row + frame_) * grid_info_.width) + col + frame_;
            CellState cell_state = grid_cells_[index].getCellState();
            switch(cell_state)
            {
            case CellState::empty:
            {
                image.at<cv::Vec3b>(cv::Point((image.rows - 1) - row, (image.cols - 1) - col)) = cv::Vec3b(255, 255, 255);//ColourLib::getRGB(ColourLib::Colour::white);
                break;
            }
            case CellState::unknown:
            {
                image.at<cv::Vec3b>(cv::Point((image.rows - 1) - row, (image.cols - 1) - col)) = cv::Vec3b(150, 150, 150);//ColourLib::getRGB(ColourLib::Colour::grey);
                break;
            }
            case CellState::obstacle:
            {
                std::pair<ColourLib::Colour, double> max_prob = grid_cells_[index].getMaxProb();
                if(max_prob.second > min_prob_) image.at<cv::Vec3b>(cv::Point((image.rows - 1) - row, (image.cols - 1) - col)) = ColourLib::getRGB(max_prob.first);
                else image.at<cv::Vec3b>(cv::Point((image.rows - 1) - row, (image.cols - 1) - col)) = cv::Vec3b(0, 0, 0);//ColourLib::getRGB(ColourLib::Colour::black);
                break;
            }
            case CellState::occupied:
            {
                image.at<cv::Vec3b>(cv::Point((image.rows - 1) - row, (image.cols - 1) - col)) = cv::Vec3b(0, 0, 0);//ColourLib::getRGB(ColourLib::Colour::black);
                break;
            }
            }
        }
    }
}

void Grid::initializeMapImage(cv::Mat &image)
{
    image = cv::Mat(grid_info_.height - 2*frame_, grid_info_.width - 2*frame_,  CV_8UC3, cv::Vec3b(150, 150, 150));
}
    
void Grid::updateCellNeighbours(unsigned index, int steps)
{
    int cell_col = grid_cells_[index].getCol();
    int cell_row = grid_cells_[index].getRow();
    
    for(int row = cell_row - steps; row < cell_row + steps; row ++)
    {
        for(int col = cell_col - steps; col < cell_col + steps; col ++)
        {
            if(row >= 0 && row <= grid_info_.height && col >=0 && col <= grid_info_.width)
            {
                unsigned neighbour_index = row * grid_info_.width + col;
                std::pair<ColourLib::Colour, double> max_prob = grid_cells_[neighbour_index].getMaxProb();
                if(!grid_cells_[neighbour_index].probNeighbour() && max_prob.second > min_prob_)
                {
                    grid_cells_[index].processPoint(ColourLib::getR(max_prob.first), ColourLib::getG(max_prob.first), ColourLib::getB(max_prob.first), 
                                                    neighbourProb(steps), miss_prob_, true);
                }
            }
        }
    }
}
    
void Grid::updateGridNeighbours(int steps)
{
    for(size_t i = 0; i < grid_cells_.size(); i++)
    {
        std::pair<ColourLib::Colour, double> max_prob = grid_cells_[i].getMaxProb();
        if(max_prob.second < min_prob_)
        {
            updateCellNeighbours(i, steps);
        }
    }
}

////PRIVATE METHODS
double Grid::neighbourProb(int spaces)
{
    return miss_prob_ + (hit_prob_ - miss_prob_) * exp(-k_ * static_cast<double>(spaces));
}

}

