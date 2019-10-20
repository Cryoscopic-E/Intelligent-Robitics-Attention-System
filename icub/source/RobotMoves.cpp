/*
** Heriott Watt University, 2019
** Intelligent Robotics
** Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
*/

#include "../headers/RobotMoves.hpp"

RobotMoves::RobotMoves()
{
}

RobotMoves::~RobotMoves()
{
}

int RobotMoves::initRobot(PolyDriver *headDriver, PolyDriver *armDriver)
{
    // /*INITIALIZE DRIVERS*/
    Property _armOptions, _headOptions;

    // Initialize Head Driver
    _headOptions.put("device", "remote_controlboard");
    _headOptions.put("local", "/tutorial/motor/client");
    _headOptions.put("remote", "/icubSim/head");

    headDriver->open(_headOptions);

    //_headDriver.
    if (!headDriver->isValid())
    {
        std::cout << "Cannot connect to robot head." << std::endl;
        return -1;
    }
    headDriver->view(_headPos);
    headDriver->view(_headVelocity);
    headDriver->view(_headEncoders);
    if (_headPos == NULL || _headVelocity == NULL || _headEncoders == NULL)
    {
        std::cout << "Interface to robot head unavailable." << std::endl;
        headDriver->close();
        return -1;
    }
    int jnts = 0;

    _headPos->getAxes(&jnts);
    _head.resize(jnts);
    // Initialize Right Arm Driver
    _armOptions.put("device", "remote_controlboard");
    _armOptions.put("local", "/test/client");
    _armOptions.put("remote", "/icubSim/right_arm");

    armDriver->open(_armOptions);
    if (!armDriver->isValid())
    {
        std::cout << "Cannot connect to robot arm." << std::endl;
        return -1;
    }
    armDriver->view(_rArmPos);
    if (_rArmPos == NULL)
    {
        std::cout << "Interface to robot right arm unavailable." << std::endl;
        armDriver->close();
        return -1;
    }

    _rArmPos->getAxes(&jnts);
    _rightArm.resize(jnts);

    return (0);
}

void RobotMoves::idle()
{
    resetRightArm();
    resetHead();
    return;
}

void RobotMoves::followFace()
{
    lookAt(_faceLastPos);
}

void RobotMoves::followMarker()
{
    lookAt(_markerLastPos);
}

void RobotMoves::setFaceLastPos(std::pair<double, double> facePos)
{
    _faceLastPos = facePos;
}

void RobotMoves::setMarkerLastPos(std::pair<double, double> markerPos)
{
    _markerLastPos = markerPos;
}

void RobotMoves::lookAt(std::pair<double, double> targetPos)
{

    //std::cout << "looking to " << targetPos.first << " - " << targetPos.second << std::endl;
    _head[0] = targetPos.second;
    _head[1] = 0;
    _head[2] = targetPos.first * -1;
    _head[3] = 0;
    _head[4] = 0;
    _head[5] = 0;
    _headPos->positionMove(_head.data());
}

void RobotMoves::riseRightArm()
{
    _rightArm[0] = 0;   //shoulder (front) -90 - 10
    _rightArm[1] = 90;  //shoulder (side) 0 - 160
    _rightArm[2] = -35; //shoulder (rotation) -30 - 90
    _rightArm[3] = 90;  //elbow (flex) 20 - 100
    _rightArm[4] = 0;   //elbow (rotation) -90 - 90
    _rightArm[5] = -45; // wrist (up and down) -90 - 0
    _rightArm[6] = 0;   // wrist (right to left) -20 - 50
    _rightArm[7] = 50;  //fingers (spread) 0 - 60
    _rightArm[8] = 0;   //thumb (first art) 10 - 90
    _rightArm[9] = 0;   //thumb (spread) 0 - 90
    _rightArm[10] = 0;  //thumb (all art) 0 - 180
    _rightArm[11] = 0;  //index finger (first art) 0 - 90
    _rightArm[12] = 0;  //index finger (all art) 0 - 180
    _rightArm[13] = 0;  //middle finger (first art) 0 - 90
    _rightArm[14] = 0;  //middle finger (all art) 0 - 180
    _rightArm[15] = 0;  //ring + little finger (all art) 0 - 180
    _rArmPos->positionMove(_rightArm.data());
}

void RobotMoves::resetHead()
{
    for (int i = 0; i < _head.size(); i++)
    {
        _head[i] = 0;
    }
    _headPos->positionMove(_head.data());
}

void RobotMoves::resetRightArm()
{
    _rightArm[0] = 0;  //shoulder (front) -90 - 10
    _rightArm[1] = 0;  //shoulder (side) 0 - 160
    _rightArm[2] = 0;  //shoulder (rotation) -30 - 90
    _rightArm[3] = 20; //elbow (flex) 20 - 100
    _rightArm[4] = 0;  //elbow (rotation) -90 - 90
    _rightArm[5] = 0;  // wrist (up and down) -90 - 0
    _rightArm[6] = 0;  // wrist (right to left) -20 - 50
    _rightArm[7] = 60; //fingers (spread) 0 - 60
    _rightArm[8] = 10; //thumb (first art) 10 - 90
    _rightArm[9] = 0;  //thumb (spread) 0 - 90
    _rightArm[10] = 0; //thumb (all art) 0 - 180
    _rightArm[11] = 0; //index finger (first art) 0 - 90
    _rightArm[12] = 0; //index finger (all art) 0 - 180
    _rightArm[13] = 0; //middle finger (first art) 0 - 90
    _rightArm[14] = 0; //middle finger (all art) 0 - 180
    _rightArm[15] = 0; //ring + little finger (all art) 0 - 180

    _rArmPos->positionMove(_rightArm.data());
}