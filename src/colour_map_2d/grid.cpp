#include "colour_map_2d/grid.h"

namespace ColourMap2D {

Grid::Grid():initialized_(false)
{}

Grid::Grid(double hit_prob, double miss_prob, double min_prob, 
         int cell_occupied, int cell_obstacle, int cell_empty, int cell_unknown,
         int spawn_rate, double spawn_noise, int spawn_size, int frame)
    :hit_prob_(hit_prob), miss_prob_(miss_prob), min_prob_(min_prob), initialized_(false),
    cell_occupied_(cell_occupied), cell_obstacle_(cell_obstacle), cell_empty_(cell_empty), cell_unknown_(cell_unknown),
    spawn_rate_(spawn_rate), spawn_noise_(spawn_noise), spawn_size_(spawn_size), frame_(frame)
{}

////GETTERS
bool Grid::initialized() const { return initialized_; }

////METHODS
void Grid::processOGMap(const nav_msgs::OccupancyGrid &og_map)
{
    std::lock_guard<std::mutex> locker(mutex_);
    if(og_map.info.width != grid_info_.width || og_map.info.height != grid_info_.height)
    {
        ROS_ERROR("MAP SIZE DIFFERS");
    }
    for(size_t i = 0; i < og_map.data.size(); i++)
    {
        if(og_map.data[i] == cell_occupied_ && grid_cells_[i].getCellState != CellState::occupied)
        {
            grid_cells_[i].setCellState(CellState::occupied);
        }
        else if(og_map.data[i] == cell_obstacle_ && grid_cells_[i].getCellState != CellState::obstacle)
        {
            grid_cells_[i].setCellState(CellState::obstacle);
        }
        else if(og_map.data[i] == cell_empty_ && grid_cells_[i].getCellState != CellState::empty) 
        {
            grid_cells_[i].setCellState(CellState::empty);
        }
        else if(og_map.data[i] == cell_unknown_ && grid_cells_[i].getCellState != CellState::unknown)
        {
            grid_cells_[i].setCellState(CellState::unknown);   
        }
    }
}

void Grid::initializeGrid(nav_msgs::MapMetaData og_map_data)
{
    std::lock_guard<std::mutex> locker(mutex_);
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

/*void Grid::proccessPoint(int index, uint8_t r, uint8_t g, uint8_t , int spawn)
{
    if(index > 0)
    {
        grid_cells_[index].processPoint(r, g, b, hit_prob_, miss_prob_);
        if(spawn_size_ > 0 && grid_cells_[index].getCellState() == CellState::obstacle)
        {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator (seed);
            std::uniform_int_distribution<int> spawn_distribution(0, spawn_rate_);
            if(spawn_distribution(generator) == spawn_rate_)
            {
                std::normal_distribution<double> noise_distribution(0.0, spawn_noise_);
                geometry_msgs::Pose2D point_pose = MapTransform::indextoPose(index, grid_info_);
                for(int i = 0; i < spawn_size_; i++)
                {
                    geometry_msgs::Pose2D temp_pose;
                    temp_pose.x = point_pose.x + noise_distribution(generator);
                    temp_pose.y = point_pose.y + noise_distribution(generator);
                    unsigned temp_index = MapTransform::posetoIndex(temp_pose, grid_info_);
                    grid_cells_[temp_index].processPoint(r, g, b, hit_prob_, miss_prob_);
                }
            }
        }
    }
}*/
         
void Grid::proccessPoints(std::vector<ColourPoint> &points)
{
    std::lock_guard<std::mutex> locker(mutex_);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> spawn_distribution(0, spawn_rate_);  
    std::normal_distribution<double> noise_distribution(0.0, spawn_noise_);
         
    for(size_t i = 0; i < points.size(); i++)
    {
        unsigned index = ColourMap2D::MapTransform::posetoIndex(points[i].getX(), points[i].getY(), grid_.getGridInfo());
        grid_cells_[index].processPoint(points[i], hit_prob_, miss_prob_);
        if(spawn_size > 0 && grid_cells_[index].getCellState() == CellState::obstacle && spawn_distribution(generator) == spawn_rate_)
        {
            for(int i = 0; i < spawn_size_; i++)
            {
                ColourPoint point(points[i], noise_distribution(generator), noise_distribution(generator));
                unsigned temp_index = MapTransform::xytoIndex(point.getX(), point.getY(), grid_info_);
                grid_cells_[temp_index].proccessPoint(point, hit_prob_, miss_prob_);
            }
        }
    }
}

void Grid::updateImage(cv::Mat &image)
{
    std::lock_guard<std::mutex> locker(mutex_);
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
    std::lock_guard<std::mutex> locker(mutex_);
    image = cv::Mat(grid_info_.height - 2*frame_, grid_info_.width - 2*frame_,  CV_8UC3, cv::Vec3b(150, 150, 150));
}

}

