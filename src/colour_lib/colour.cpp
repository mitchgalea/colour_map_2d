#include "colour_lib/colour.h"

namespace ColourLib {

ColourInfo::ColourInfo(Colour colour_input, std::string name_input,
                       std::vector<uint8_t> h_input, std::vector<uint8_t> s_input, std::vector<uint8_t> v_input,
                       uint8_t r_input, uint8_t g_input, uint8_t b_input)
    :colour(colour_input), name(name_input), h(h_input), s(s_input), v(v_input), r(r_input), g(g_input), b(b_input)
{}

ColourInfo::ColourInfo(Colour colour_input, std::__cxx11::string name_input,
                       uint8_t r_input, uint8_t g_input, uint8_t b_input)
    :colour(colour_input), name(name_input), r(r_input), g(g_input), b(b_input)
{}


Colour Identifier::identifyHSVThresh(cv::Scalar RGB)
{
    cv::Scalar hsv = Identifier::scalarRGB2HSV(RGB);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(Identifier::inThresh(COLOURS.at(i).h, hsv[0]) &&
           Identifier::inThresh(COLOURS.at(i).s, hsv[1]) &&
           Identifier::inThresh(COLOURS.at(i).v, hsv[2])) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

Colour Identifier::identifyHSVThresh(uint8_t r, uint8_t g, uint8_t b)
{
    cv::Scalar RGB(r, g, b);
    cv::Scalar hsv = Identifier::scalarRGB2HSV(RGB);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(Identifier::inThresh(COLOURS.at(i).h, hsv[0]) &&
           Identifier::inThresh(COLOURS.at(i).s, hsv[1]) &&
           Identifier::inThresh(COLOURS.at(i).v, hsv[2])) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

Colour Identifier::identifyRGB(cv::Scalar RGB)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(RGB[0] == COLOURS.at(i).r &&
           RGB[1] == COLOURS.at(i).b &&
           RGB[2] == COLOURS.at(i).g) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

Colour Identifier::identifyRGB(uint8_t r, uint8_t g, uint8_t b)
{
    cv::Scalar RGB(r, g, b);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(RGB[0] == COLOURS.at(i).r &&
           RGB[1] == COLOURS.at(i).b &&
           RGB[2] == COLOURS.at(i).g) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

cv::Scalar Identifier::scalarRGB2HSV(cv::Scalar rgb_scalar)
{
    cv::Mat hsv;
    cv::Mat rgb(1,1, CV_8UC3, rgb_scalar);
    cv::cvtColor(rgb, hsv, CV_RGB2HSV);
    return cv::Scalar(hsv.data[0], hsv.data[1], hsv.data[2]);
}

cv::Scalar Identifier::scalarHSV2RGB(cv::Scalar hsv_scalar)
{
    cv::Mat rgb;
    cv::Mat hsv(1,1, CV_8UC3, hsv_scalar);
    cv::cvtColor(hsv, rgb, CV_HSV2RGB);
    return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

bool Identifier::inThresh(std::vector<uint8_t> thresh, uint8_t input)
{
    bool in_range = false;
    for(int i = 0; i < thresh.size(); i = i + 2)
    {
        if(input >= thresh.at(i) && input <= thresh.at(i+1)) in_range = true;
    }
    return in_range;
}

cv::Scalar getRGB(Colour colour)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(colour == COLOURS.at(i).colour){
            return cv::Scalar(COLOURS.at(i).r, COLOURS.at(i).g, COLOURS.at(i).b);
        }
    }
    return cv::Scalar(0,0,0);
}
uint8_t getR(Colour colour)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(colour == COLOURS.at(i).colour){
            return COLOURS.at(i).r;
        }
    }
    return 0;
}
uint8_t getG(Colour colour)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(colour == COLOURS.at(i).colour){
            return COLOURS.at(i).g;
        }
    }
    return 0;
}
uint8_t getB(Colour colour)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(colour == COLOURS.at(i).colour){
            return COLOURS.at(i).b;
        }
    }
    return 0;
}

std::string getName(Colour colour)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(colour == COLOURS.at(i).colour){
            return COLOURS.at(i).name;
        }
    }
    return " ";
}
}
