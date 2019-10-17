//
// Heriott Watt University, 2019
// Intelligent Robotics
// Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
//

#ifndef INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP
#define INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class RobotMoves {
    public:
    RobotMoves();
    ~RobotMoves();

    int     initRobot();
    void    riseRightArm();
    void    resetRightArm();
    void    fuckYou();

    private:
    Vector  _rightArm;
    Property _options;
};

#endif //INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_ROBOTMOVES_HPP
