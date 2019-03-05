#include <stdint.h>

#ifndef COLOUR_POINT_H
#define COLOUR_POINT_H

namespace ColourMap2D{
//Class used to store transformed Point Cloud information in
class ColourPoint{
    double x_;      //x pose in meters
    double y_;      //y pose in meters  
    uint8_t r_;     //r pixel value
    uint8_t g_;     //g pixel value
    uint8_t b_;     //b pixel value
public:
    ////CONSTRUCTORS
    //constructor that takes input for all variables
    ColourPoint(double x_in, double y_in, uint8_t r_in, uint8_t g_in, uint8_t b_in);
    //constructor that generates a point based on another point with added noise
    ColourPoint(ColourPoint point, double x_noise, double y_noise);
    
    ////GETTERS
    double getX() const;
    double getY() const;
    uint8_t getR() const;
    uint8_t getG() const;
    uint8_t getB() const;
};

}

#endif // COLOUR_POINT_H
