#include <geometry_msgs/Pose2D.h>

#include <vector>

#ifndef GRIDCELL_H
#define GRIDCELL_H

namespace ColourMap2D{

class GridCell
{
private:
    int col_;
    int row_;
    geometry_msgs::Pose2D pose_;

public:
    GridCell(int col, int row, geometry_msgs::Pose2D pose);

    int getCol() const;
    int getRow() const;
    geometry_msgs::Pose2D getPose() const;

    void setCol(int col);
    void setRow(int row);
    void setPose(geometry_msgs::Pose2D pose);
};

}

#endif // GRIDCELL_H
