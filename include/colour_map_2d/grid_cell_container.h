#include "colour_map_2d/grid_cell.h"
#include "colour_map_2d/map_transform.h"

#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <mutex>

#ifndef GRIDCELLCONTAINER_H
#define GRIDCELLCONTAINER_H

namespace ColourMap2D{

class GridCellContainer
{
private:
    std::vector<GridCell> grid_cells_;
    std::mutex mutex_;

    bool addCellInternal(GridCell grid_cell);
    bool contains(GridCell grid_cell);
public:
    GridCellContainer();

    void processOGMap(nav_msgs::OccupancyGrid og_map);

    bool addCell(GridCell grid_cell);
    size_t size();

};

}
#endif // GRIDCELLCONTAINER_H
