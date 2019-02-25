#include "colour_map_2d/colour_point.h"

namespace ColourMap2D{

ColourPoint::ColourPoint(double x, double y, ColourLib::Colour colour)
    :colour_(colour)
{
    pose_.x = x;
    pose_.y = y;
}

ColourPoint::ColourPoint(pcl::PointXYZRGB point)
{
    colour_ = ColourLib::Identifier::identifyHSVThresh(point.r, point.g, point.b);
    pose_.x = point.x;
    pose_.y = point.y;
}

////GETTERS
geometry_msgs::Pose2D ColourPoint::getPose() const {return pose_;}
ColourLib::Colour ColourPoint::getColour() const {return colour_;}

////SETTERS
void ColourPoint::setID(int16_t id) {id_ = id;}

void ColourPoint::print()
{
    std::cout << "Colour: " << ColourLib::getName(colour_) << ", Pose (x: " << pose_.x << ", y: " << pose_.y << ")" << std::endl;
}
}






