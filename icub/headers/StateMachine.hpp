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

/**
 *  StateMachine class
 * 
 * The state machine handles the state changes, it has a table of all possible transition that can happen 
 * */
class StateMachine
{
public:
    // called every time a detection is registerd
    void OnEvent(Transition::Event);
    // execute the function linked to the transition
    void Execute();
    // set che context robot responsible of the transitions
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
