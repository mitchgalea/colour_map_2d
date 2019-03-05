#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

#ifndef COLOUR_H
#define COLOUR_H

//RED INFO
const std::vector<uint8_t> RED_H = {0, 12, 170, 180};
const std::vector<uint8_t> RED_S = {100, 255};
const std::vector<uint8_t> RED_V = {100, 255};
const uint8_t RED_R = 255;
const uint8_t RED_G = 0;
const uint8_t RED_B = 0;

//GREEN INFO
const std::vector<uint8_t> GREEN_H = {36, 180};
const std::vector<uint8_t> GREEN_S = {80, 255};
const std::vector<uint8_t> GREEN_V = {60, 255};
const uint8_t GREEN_R = 0;
const uint8_t GREEN_G = 255;
const uint8_t GREEN_B = 0;

//BLUE INFO
const std::vector<uint8_t> BLUE_H = {100, 135};
const std::vector<uint8_t> BLUE_S = {90, 255};
const std::vector<uint8_t> BLUE_V = {50, 255};
const uint8_t BLUE_R = 0;
const uint8_t BLUE_G = 0;
const uint8_t BLUE_B = 255;


namespace ColourLib{
// Enum Class for Colours
enum class Colour {
    red,
    blue,
    green,
    white,
    black,
    grey
};

// ColourInfo is the struct that contains RGB and HSV information about colours to be detected
struct ColourInfo{
    ColourInfo(Colour colour_input, std::string name_input,
               std::vector<uint8_t> h_input, std::vector<uint8_t> s_input, std::vector<uint8_t> v_input,
               uint8_t r_input, uint8_t g_input, uint8_t b_input);
    ColourInfo(Colour colour_input, std::string name_input,
               uint8_t r_input, uint8_t g_input, uint8_t b_input);
    Colour colour;
    std::vector<uint8_t> h;
    std::vector<uint8_t> s;
    std::vector<uint8_t> v;
    std::string name;

    uint8_t r;
    uint8_t g;
    uint8_t b;
};
//Returns RGB information about a colour
cv::Vec3b getRGB(Colour colour);
//Returns R value for a colour
uint8_t getR(Colour colour);
//Returns G value for a colour
uint8_t getG(Colour colour);
//Returns B value for a colour
uint8_t getB(Colour colour);
//Returns name of a colour
std::string getName(Colour colour);

//Colours being Used
const ColourInfo RED(Colour::red, "RED", RED_H, RED_S, RED_V, RED_R, RED_G, RED_B);
const ColourInfo GREEN(Colour::green, "GREEN", GREEN_H, GREEN_S, GREEN_V, GREEN_R, GREEN_G, GREEN_B);
const ColourInfo BLUE(Colour::blue, "BLUE", BLUE_H, BLUE_S, BLUE_V, BLUE_R, BLUE_G, BLUE_B);
const ColourInfo BLACK(Colour::black, "BLACK", 0, 0, 0);
const ColourInfo WHITE(Colour::white, "WHITE", 255, 255, 255);
const ColourInfo GREY(Colour::grey, "GREY", 150, 150, 150);

//Colour Vectors
const std::vector<ColourInfo> COLOURS = {RED, BLUE, GREEN, BLACK};
const std::vector<ColourInfo> ALL_COLOURS = {RED, BLUE, GREEN, BLACK};

//Static Class for Identifying if colours are within Thresholds
class Identifier
{
private:
    Identifier();
    static bool inThresh(std::vector<uint8_t> thresh, uint8_t input);
    static cv::Vec3b vecRGB2HSV(cv::Vec3b rgb);
    static cv::Vec3b vecHSV2RGB(cv::Vec3b hsv);
public:
    static Colour identifyHSVThresh(cv::Vec3b RGB);
    static Colour identifyHSVThresh(uint8_t r, uint8_t g, uint8_t b);
    static Colour identifyRGB(cv::Vec3b RGB);
    static Colour identifyRGB(uint8_t r, uint8_t g, uint8_t b);
};

}


#endif // COLOUR_H
