#include "StateMachine.hpp"
#include "RobotMoves.hpp"

StateMachine::StateMachine()
{
    currentState = Transition::State::IDLE;
    lastEvent = Transition::Event::NO_EVENT;
    // Transition table
    // rows: current state
    // columns: last event happend
    transitionTable = {
        {Transition(Transition::State::IDLE, &RobotMoves::idle),
         Transition(Transition::State::FACE, &RobotMoves::followFace),
         Transition(Transition::State::MARKER, &RobotMoves::followMarker),
         Transition(Transition::State::CIRCLE, &RobotMoves::riseRightArm)},
        {Transition(Transition::State::IDLE, &RobotMoves::idle),
         Transition(Transition::State::FACE, &RobotMoves::followFace),
         Transition(Transition::State::MARKER, &RobotMoves::followMarker),
         Transition(Transition::State::FACE, &RobotMoves::followFace)},
        {Transition(Transition::State::IDLE, &RobotMoves::idle),
         Transition(Transition::State::MARKER, &RobotMoves::followMarker),
         Transition(Transition::State::MARKER, &RobotMoves::followMarker),
         Transition(Transition::State::MARKER, &RobotMoves::followMarker)},
        {Transition(Transition::State::IDLE, &RobotMoves::idle),
         Transition(Transition::State::FACE, &RobotMoves::followFace),
         Transition(Transition::State::CIRCLE, &RobotMoves::riseRightArm),
         Transition(Transition::State::CIRCLE, &RobotMoves::riseRightArm)}};
}

StateMachine::~StateMachine()
{
}

void StateMachine::SetRobot(RobotMoves &robot)
{
    _robot = &robot;
}

void StateMachine::OnEvent(Transition::Event newEvent)
{
    lastEvent = newEvent;
}

void StateMachine::Execute()
{
    // call the function with the current context robot
    (_robot->*(transitionTable[currentState][lastEvent].function))();
    // change current state after transition
    currentState = transitionTable[currentState][lastEvent].newState;
}
