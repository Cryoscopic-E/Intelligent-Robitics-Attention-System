#include "StateMachine.hpp"
#include "RobotMoves.hpp"

StateMachine::StateMachine()
{
    currentState = Transition::State::IDLE;
    lastEvent = Transition::Event::NO_EVENT;

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
    (_robot->*(transitionTable[currentState][lastEvent].function))();
    currentState = transitionTable[currentState][lastEvent].newState;
    //printf("Current State: %d      Last Event: %d\n", currentState, lastEvent);
}
