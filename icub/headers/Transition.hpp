#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include "RobotMoves.hpp"
#include <utility>

// function pointer type for RobotMoves functions
typedef void (RobotMoves::*StateFunction)();

/**
 * Transition class
 * 
 * This class is used to reprensent a transition old_state -> new_state
 * Every transition has a newState variable, representing the new state after a trasition
 * and a function pointer that holds the function to call when a transition is invoked.
 * */
class Transition
{
public:
    enum Event
    {
        NO_EVENT,
        FACE_DETECTED,
        MARKER_DETECTED,
        CIRCLE_DETECTED
    };

    enum State
    {
        IDLE,
        FACE,
        MARKER,
        CIRCLE
    };

    State newState;
    Transition(State, StateFunction);

    ~Transition();
    StateFunction function;
};

#endif //TRANSITION_HPP