#include "colour_map_2d/colour_point.h"
#include "colour_lib/colour.h"
#include <vector>
#include <utility>

#ifndef GRIDCELL_H
#define GRIDCELL_H

namespace ColourMap2D{
//enum class for cellstates
enum class CellState{
    obstacle,
    occupied,
    empty,
    unknown
};
//// Grid Cell class contains information and methods for a grid cell
//   contains probabilites for each colour specified in colour.h
//   histogram filter functionalities for colour probabilities
class GridCell
{
private:
    ////MEMBERS
    unsigned index_;                    //og map index
    CellState cell_state_;              //state of cell
    std::vector<double> colour_probs_;  //vector containing colour probailities in reference to the colours vector in colour.h
    
    ////BOOLEANS
    bool state_changed_;                //boolean that shows whether the state of the cell has been changed
public:
    ////CONSTRUCTORS
    GridCell(unsigned index, CellState cell_state = CellState::unknown);

    ////GETTERS
    unsigned getIndex() const;
    int getCol() const;
    int getRow() const;
    CellState getCellState() const;
    bool stateChanged() const;

    ////SETTERS
    void setIndex(unsigned index);
    void setCellState(CellState cell_state);

    ////METHODS
    //processes a Point with separate r g b informationm - histogram filters
    void processPoint(uint8_t r, uint8_t g, uint8_t b, double hit_prob, double miss_prob, double prob_thresh);
    //processes a Point - histogram filter
    void processPoint(ColourPoint point,  double hit_prob, double miss_prob, double prob_thresh);
    //normalizes the probabilites
    void normalizeProbs();
    //ensures all probabilities are above threshold
    void probThresh(double prob_thresh);
    //returns the max probability colour
    std::pair<ColourLib::Colour, double> getMaxProb(bool state_observed = true);
    //gets the probability of a colour
    double getProb(ColourLib::Colour colour);
    //prints the probabilities
    void printProbs();

};

}

#endif // GRIDCELL_H
