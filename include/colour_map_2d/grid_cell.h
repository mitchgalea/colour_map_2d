#include "colour_lib/colour.h"
#include <vector>
#include <utility>

#ifndef GRIDCELL_H
#define GRIDCELL_H

namespace ColourMap2D{

enum class CellState{
    occupied,
    empty,
    unknown
};

class GridCell
{
private:
    unsigned index_;
    bool occupied_;
    CellState cell_state_;
    bool prob_checked_;
    bool prob_neighbour_;
    int col_;
    int row_;
    
    std::vector<double> colour_probs_;
public:
    GridCell(unsigned index, bool occupied_in = false, CellState cell_state = empty);
    GridCell(unsigned index, int col, int row, bool occupied_in = false, CellState cell_state = empty);

    ////GETTERS
    unsigned getIndex() const;
    int getCol() const;
    int getRow() const;
    CellState getCellState() const;
    bool occupied() const;
    bool probChecked() const;
    bool probNeighbour() const;

    ////SETTERS
    void setIndex(unsigned index);
    void setOccupied(bool occupied_in);
    void setCellState(CellState cell_state);

    ////METHODS
    void processPoint(uint8_t r, uint8_t g, uint8_t b, double hit_prob, double miss_prob, bool neighbour = false);
    void normalizeProbs();
    std::pair<ColourLib::Colour, double> getMaxProb(bool check_prob = true);
    double getProb(ColourLib::Colour);
    void printProbs();
};

}

#endif // GRIDCELL_H
