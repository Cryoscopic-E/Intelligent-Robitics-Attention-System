#include "Transition.hpp"
#include "RobotMoves.hpp"

Transition::Transition(Transition::State state, StateFunction fun)
{
    newState = state;
    function = fun;
};

Transition::~Transition()
{
}