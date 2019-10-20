//
// Heriott Watt University, 2019
// Intelligent Robotics
// Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
//

#ifndef INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP
#define INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP

#include <iostream>
#include <utility>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class RobotMoves
{
public:
    RobotMoves();
    ~RobotMoves();

    int initRobot(PolyDriver *headDriver, PolyDriver *armDriver);
    //idle
    void idle();
    //gesture
    void riseRightArm();
    //following
    void followFace();
    void followMarker();

    void setFaceLastPos(std::pair<double, double>);
    void setMarkerLastPos(std::pair<double, double>);

private:
    Vector _rightArm;
    Vector _head;
    IPositionControl *_headPos;
    IVelocityControl *_headVelocity;
    IEncoders *_headEncoders;
    IPositionControl *_rArmPos;

    std::pair<double, double> _faceLastPos;
    std::pair<double, double> _markerLastPos;

    void lookAt(std::pair<double, double> pos);
    void resetRightArm();
    void resetHead();
};

#endif //INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP
