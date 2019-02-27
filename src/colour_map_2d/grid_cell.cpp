#include "colour_map_2d/grid_cell.h"

namespace ColourMap2D{

////CONSTRUCTORS
GridCell::GridCell(unsigned index, bool occupied_in, CellState cell_state)
    : index_(index), occupied_(occupied_in), prob_neighbour_(false), cell_state_(cell_state)
{
    for(size_t i = 0; i < ColourLib::COLOURS.size() - 1; i++)
    {
        colour_probs_.push_back(1 / static_cast<double>(ColourLib::COLOURS.size() + 1));
    }
    colour_probs_.push_back(2 / static_cast<double>(ColourLib::COLOURS.size() + 1));
}
    
GridCell::GridCell(unsigned index, int col, int row, bool occupied_in, CellState cell_state)
    : index_(index), occupied_(occupied_in), col_(col), row_(row), prob_neighbour_(false), cell_state_(cell_state)
{
    for(size_t i = 0; i < ColourLib::COLOURS.size() - 1; i++)
    {
        colour_probs_.push_back(1 / static_cast<double>(ColourLib::COLOURS.size() + 1));
    }
    colour_probs_.push_back(2 / static_cast<double>(ColourLib::COLOURS.size() + 1));
}

////GETTERS
unsigned GridCell::getIndex() const {return index_;}
int GridCell::getCol() const {return col_;}
int GridCell::getRow() const {return row_;}
CellState GridCell::getCellState() const {return cell_state_;}
bool GridCell::occupied() const {return occupied_;}
bool GridCell::probChecked() const {return prob_checked_;}
bool GridCell::probNeighbour() const {return prob_neighbour_;}

////SETTERS
void GridCell::setIndex(unsigned index) {index_ = index;}
void GridCell::setCellState(CellState cell_state) {cell_state_ = cell_state;}
void GridCell::setOccupied(bool occupied_in) {occupied_ = occupied_in;}

////METHODS
void GridCell::processPoint(uint8_t r, uint8_t g, uint8_t b, double hit_prob, double miss_prob, bool neighbour)
{
    if(cell_state_ == CellState::obstacle)
    {
        ColourLib::Colour colour = ColourLib::Identifier::identifyHSVThresh(r, g, b);
        for(size_t i = 0; i < ColourLib::COLOURS.size(); i++)
        {
            if(colour == ColourLib::COLOURS[i].colour) colour_probs_[i] = colour_probs_[i] * hit_prob;
            else colour_probs_[i] = colour_probs_[i] * miss_prob;
        }
        normalizeProbs();
        prob_checked_ = false;
        if(neighbour) prob_neighbour_ = true;
        else prob_neighbour_ = false;
    }
}

void GridCell::normalizeProbs()
{
    double count = 0;
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        count += colour_probs_[i];
    }
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        colour_probs_[i] = colour_probs_[i] / count;
    }


}

std::pair<ColourLib::Colour, double> GridCell::getMaxProb(bool check_prob)
{
    std::pair<ColourLib::Colour, double> max_prob;
    max_prob.second = 0.0;
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        if(colour_probs_[i] > max_prob.second)
        {
            max_prob.second = colour_probs_[i];
            max_prob.first = ColourLib::COLOURS[i].colour;
        }
    }
    if(check_prob) prob_checked_ = true;
    return max_prob;
}

void GridCell::printProbs()
{
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        std::cout << ColourLib::getName(ColourLib::COLOURS[i].colour) << ": " << colour_probs_[i] << "   ";
    }
    std::cout << std::endl;
}

}

