#include "colour_map_2d/grid_cell.h"

namespace ColourMap2D{

GridCell::GridCell(int col, int row, geometry_msgs::Pose2D pose): col_(col), row_(row), pose_(pose)
{}

int GridCell::getCol() const {return col_;}
int GridCell::getRow() const {return row_;}
geometry_msgs::Pose2D GridCell::getPose() const {return pose_;}

void GridCell::setCol(int col) {col_ = col;}
void GridCell::setRow(int row) {row_ = row;}
void GridCell::setPose(geometry_msgs::Pose2D pose) {pose_ = pose;}
}

