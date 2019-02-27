#ifndef COLOUR_POINT_H
#define COLOUR_POINT_H

namespace ColourMap2D{

class ColourPoint{
    double x_;
    double y_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
public:
    ////CONSTRUCTORS
    ColourPoint(double x_in, double y_in, uint8_t r_in, uint8_t g_in, uint8_t b_in);
    
    ////GETTERS
    double getX() const;
    double getY() const;
    double getR() const;
    double getG() const;
    double getB() const;
};

}

#endif // COLOUR_POINT_H