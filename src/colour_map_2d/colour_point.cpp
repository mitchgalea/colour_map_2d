#include "colour_map_2d/colour_point.h"

namespace {
////CONSTRUCTORS
ColourPoint::ColourPoint(double x_in, double y_in, uint8_t r_in, uint8_t g_in, uint8_t b_in)
    ::x(x_in), y(y_in), r(r_in), g(g_in), b(b_in)
    {}

////GETTERS
double ColourPoint::getX() const { return x_; }
double ColourPoint::gety() const { return y_; }
uint8_t ColourPoint::getR() const { return r_; }
uint8_t ColourPoint::getG() const { return b_; }
uint8_t ColourPoint::getB() const { return g_; }
}
