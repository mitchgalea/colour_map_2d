#include "colour_map_2d/grid_cell_container.h"

namespace ColourMap2D {

GridCellContainer::GridCellContainer()
{}

void GridCellContainer::processOGMap(nav_msgs::OccupancyGrid og_map)
{
    std::lock_guard<std::mutex> locker(mutex_);

    for(unsigned row = 0; row < og_map.info.height; row++)
    {
        for(unsigned col = 0; col < og_map.info.width; col++)
        {
            unsigned index = row * og_map.info.width + col;
            if(og_map.data[index] == 100)
            {
                addCellInternal(GridCell(col, row, MapTransform::gridtoPose(col, row, og_map.info)));
            }
        }
    }
}

bool GridCellContainer::addCellInternal(GridCell grid_cell)
{
    if(!contains(grid_cell))
    {
        grid_cells_.push_back(grid_cell);
        return true;
    }
    return false;
}

bool GridCellContainer::addCell(GridCell grid_cell)
{
    std::lock_guard<std::mutex> locker(mutex_);
    if(!contains(grid_cell))
    {
        grid_cells_.push_back(grid_cell);
        return true;
    }
    return false;
}

size_t GridCellContainer::size()
{
    std::lock_guard<std::mutex> locker(mutex_);
    return grid_cells_.size();
}

bool GridCellContainer::contains(GridCell grid_cell)
{
    for(auto cell_it = grid_cells_.begin(); cell_it < grid_cells_.end(); cell_it++)
    {
        if(cell_it->getCol() == grid_cell.getCol() && cell_it->getRow() == grid_cell.getRow()) return true;
    }
    return false;
}


}

