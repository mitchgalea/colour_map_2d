#include <geometry_msgs/Pose2D.h>
#include "colour_map_2d/colour_point.h"
#include "colour_lib/colour.h"

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
	bool occupied_;
	std::vector<*ColourPoint> colour_point_ptrs_;

public:
    GridCell(int col, int row, geometry_msgs::Pose2D pose);

	////GETTERS
    int getCol() const;
    int getRow() const;
    geometry_msgs::Pose2D getPose() const;
	bool occupied() const;
    ////SETTERS
    void setCol(int col);
    void setRow(int row);
    void setPose(geometry_msgs::Pose2D pose);
	
	ColourLib::Colour maxColour();
	int maxColourCount();
	int colourCount(ColourLib::Colour colour);
	cv::Scalar getMaxRGB(int req_colour_points);
	cv::Scalar getRGB(ColourLib::Colour colour, int req_colour_points);
};

}

#endif // GRIDCELL_H
