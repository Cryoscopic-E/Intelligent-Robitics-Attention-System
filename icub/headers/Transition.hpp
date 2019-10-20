#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include "RobotMoves.hpp"
#include <utility>

typedef void (RobotMoves::*StateFunction)();

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
    void call();
};

#endif //TRANSITION_HPP