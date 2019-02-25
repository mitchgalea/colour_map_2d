#include "geometry_msgs/Pose2D.h"
#include "colour_lib/colour.h"
#include "pcl/point_types.h"

#include <vector>

#ifndef COLOURPOINT_H
#define COLOURPOINT_H

namespace ColourMap2D{

class ColourPoint
{
private:
    geometry_msgs::Pose2D pose_;
    ColourLib::Colour colour_;
    uint16_t id_;
    uint8_t connections_;
public:
    ColourPoint(double x, double y, ColourLib::Colour colour);
    ColourPoint(pcl::PointXYZRGB point);

    ////GETTERS
    geometry_msgs::Pose2D getPose() const;
    ColourLib::Colour getColour() const;

    ////SETTERS
    void setID(int16_t id);

    void print();
};

}

#endif // COLOURPOINT_H
