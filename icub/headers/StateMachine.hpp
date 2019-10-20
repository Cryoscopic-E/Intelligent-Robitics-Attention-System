//
// Heriott Watt University, 2019
// Intelligent Robotics
// Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
//
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "Transition.hpp"
#include "RobotMoves.hpp"

#include <vector>
/*
                    NOEVENT   |   FACE_DETECTED   |   MARKER_DETECTED   |   CIRCLE_DETECTED  (newEvent)

    IDLE           doNothing  |    lookatFace     |    lookAtMarker     |    gestureCircle

    FACE           doNothing  |    lookAtFace     |    lookAtMarker     |    lookAtFace              

    MARKER         doNothing  |    lookAtMarker   |    lookAtMarker     |    lookAtMarker

    CIRCLE         doNothing  |    lookAtFace     |    gestureCircle     |    gestureCircle

    (currentState)
*/

class StateMachine
{
public:
    void OnEvent(Transition::Event);
    void Execute();
    void SetRobot(RobotMoves &);
    StateMachine();
    ~StateMachine();

private:
    RobotMoves *_robot;
    Transition::State currentState;
    Transition::Event lastEvent;
    std::vector<std::vector<Transition>> transitionTable;
};

#endif //STATE_MACHINE_HPP
