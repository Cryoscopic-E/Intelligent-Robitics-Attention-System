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

int     RobotMoves::initRobot()
{
    _armOptions.put("device", "remote_controlboard");
    _armOptions.put("local", "/test/client");
    _armOptions.put("remote", "/icubSim/right_arm");
    _headOptions.put("device", "remote_controlboard");
    _headOptions.put("local", "/tutorial/motor/client");
    _headOptions.put("remote", "/icubSim/head");

    return (0);
}

void    RobotMoves::lookAt(std::pair<double, double> targetPos)
{
    PolyDriver robotHead(_headOptions);
    if (!robotHead.isValid()) {
        std::cout << "Cannot connect to robot head." << std::endl;
        return ;
    }

    IPositionControl    *pos;
    IVelocityControl    *vel;
    IEncoders           *enc;
    robotHead.view(pos);
    robotHead.view(vel);
    robotHead.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) {
        std::cout << "Interface to robot head unavailable." << std::endl;
        robotHead.close();
        return ;
    }
    int jnts = 0;

    pos->getAxes(&jnts);
    _head.resize(jnts);

    std::cout << "looking to " << targetPos.first << " - " << targetPos.second << std::endl;
    _head[0] = targetPos.second;
    _head[1] = 0;
    _head[2] = targetPos.first * -1;
    _head[3] = 0;
    _head[4] = 0;
    _head[5] = 0;

    pos->positionMove(_head.data());
}

void    RobotMoves::riseRightArm()
{
    PolyDriver robotDevice(_armOptions);
    if (!robotDevice.isValid()) {
        std::cout << "Cannot connect to right arm." << std::endl;
        return ;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    robotDevice.view(pos);
    robotDevice.view(vel);
    robotDevice.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) {
        std::cout << "Interface to right arm unavailable." << std::endl;
        robotDevice.close();
        return ;
    }
    int jnts = 0;

    pos->getAxes(&jnts);
    _rightArm.resize(jnts);

    _rightArm[0] = 0; //shoulder (front) -90 - 10
    _rightArm[1] = 90; //shoulder (side) 0 - 160
    _rightArm[2] = -35; //shoulder (rotation) -30 - 90
    _rightArm[3] = 90; //elbow (flex) 20 - 100
    _rightArm[4] = 0; //elbow (rotation) -90 - 90
    _rightArm[5] = -45; // wrist (up and down) -90 - 0
    _rightArm[6] = 0; // wrist (right to left) -20 - 50
    _rightArm[7] = 50; //fingers (spread) 0 - 60
    _rightArm[8] = 0; //thumb (first art) 10 - 90
    _rightArm[9] = 0; //thumb (spread) 0 - 90
    _rightArm[10] = 0; //thumb (all art) 0 - 180
    _rightArm[11] = 0; //index finger (first art) 0 - 90
    _rightArm[12] = 0; //index finger (all art) 0 - 180
    _rightArm[13] = 0; //middle finger (first art) 0 - 90
    _rightArm[14] = 0; //middle finger (all art) 0 - 180
    _rightArm[15] = 0;//ring + little finger (all art) 0 - 180

    pos->positionMove(_rightArm.data());
}

void    RobotMoves::resetRightArm()
{
    PolyDriver robotDevice(_armOptions);
    if (!robotDevice.isValid()) {
        std::cout << "Cannot connect to right arm." << std::endl;
        return ;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    robotDevice.view(pos);
    robotDevice.view(vel);
    robotDevice.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) {
        std::cout << "Interface to right arm unavailable." << std::endl;
        robotDevice.close();
        return ;
    }
    int jnts = 0;

    pos->getAxes(&jnts);
    _rightArm.resize(jnts);

    _rightArm[0] = 0; //shoulder (front) -90 - 10
    _rightArm[1] = 0; //shoulder (side) 0 - 160
    _rightArm[2] = 0; //shoulder (rotation) -30 - 90
    _rightArm[3] = 20; //elbow (flex) 20 - 100
    _rightArm[4] = 0; //elbow (rotation) -90 - 90
    _rightArm[5] = 0; // wrist (up and down) -90 - 0
    _rightArm[6] = 0; // wrist (right to left) -20 - 50
    _rightArm[7] = 60; //fingers (spread) 0 - 60
    _rightArm[8] = 10; //thumb (first art) 10 - 90
    _rightArm[9] = 0; //thumb (spread) 0 - 90
    _rightArm[10] = 0; //thumb (all art) 0 - 180
    _rightArm[11] = 0; //index finger (first art) 0 - 90
    _rightArm[12] = 0; //index finger (all art) 0 - 180
    _rightArm[13] = 0; //middle finger (first art) 0 - 90
    _rightArm[14] = 0; //middle finger (all art) 0 - 180
    _rightArm[15] = 0;//ring + little finger (all art) 0 - 180

    pos->positionMove(_rightArm.data());
}

void    RobotMoves::fuckYou()
{
    PolyDriver robotDevice(_armOptions);
    if (!robotDevice.isValid()) {
        std::cout << "Cannot connect to right arm." << std::endl;
        return ;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    robotDevice.view(pos);
    robotDevice.view(vel);
    robotDevice.view(enc);
    if (pos==NULL || vel==NULL || enc==NULL) {
        std::cout << "Interface to right arm unavailable." << std::endl;
        robotDevice.close();
        return ;
    }
    int jnts = 0;

    pos->getAxes(&jnts);
    _rightArm.resize(jnts);

    _rightArm[0] = -50; //shoulder (front) -90 - 10
    _rightArm[1] = 15; //shoulder (side) 0 - 160
    _rightArm[2] = 0; //shoulder (rotation) -30 - 90
    _rightArm[3] = 100; //elbow (flex) 20 - 100
    _rightArm[4] = -90; //elbow (rotation) -90 - 90
    _rightArm[5] = 0; // wrist (up and down) -90 - 0az
    _rightArm[6] = 0; // wrist (right to left) -20 - 50
    _rightArm[7] = 60; //fingers (spread) 0 - 60
    _rightArm[8] = 40; //thumb (first art) 10 - 90
    _rightArm[9] = 0; //thumb (spread) 0 - 90
    _rightArm[10] = 180; //thumb (all art) 0 - 180
    _rightArm[11] = 90; //index finger (first art) 0 - 90
    _rightArm[12] = 180; //index finger (all art) 0 - 180
    _rightArm[13] = 20; //middle finger (first art) 0 - 90
    _rightArm[14] = 0; //middle finger (all art) 0 - 180
    _rightArm[15] = 180;//ring + little finger (all art) 0 - 180

    pos->positionMove(_rightArm.data());
}