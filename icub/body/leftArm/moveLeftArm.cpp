#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <time.h>
#include <thread>
#include <chrono>
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
int main() {
  Network yarp; // set up yarp
  BufferedPort<Vector> targetPort;
  targetPort.open("/dave/target/in");
  Network::connect("/paul/target/out","/dave/target/in");

  Property options;
  options.put("device", "remote_controlboard");
  options.put("local", "/test/client");
  options.put("remote", "/icubSim/left_arm");
  PolyDriver robotDevice(options);
  if (!robotDevice.isValid()) {
    printf("Cannot connect to right arm. These are the devices available:\n");
    return 1;
  }
  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  robotDevice.view(pos);
  robotDevice.view(vel);
  robotDevice.view(enc);
  if (pos==NULL || vel==NULL || enc==NULL) {
    printf("Cannot get interface to right arm\n");
    robotDevice.close();
    return 1;
  }
  int jnts = 0;
  pos->getAxes(&jnts);
  Vector setpoints;
  setpoints.resize(jnts);

  while (1) { // repeat forever
    Vector *target = targetPort.read();  // read a target
    if (target!=NULL) { // check we actually got something
      printf("We got a vector containing");
      for (int i=0; i<target->size(); i++) {
        printf(" %g", (*target)[i]);
      }
      printf("\n");

      double x = (*target)[0];
      double y = (*target)[1];
      double conf = (*target)[2];

      x -= 320/2;
      y -= 240/2;

      double vx = x*1;
      double vy = -y*1;

      // prepare command

      if (conf>0.5) {
        setpoints[1] = 90; //shoulder
        setpoints[2] = 45;
        setpoints[3] = 55;
        setpoints[4] = 0;
        setpoints[5] = 90;
        setpoints[6] = 0;
        setpoints[7] = 55;
        setpoints[8] = 0;
        setpoints[9] = 90;
        setpoints[10] = 0;
        setpoints[11] = 55;
        setpoints[12] = 0;
        setpoints[13] = 90;
        setpoints[14] = 0;
        setpoints[15] = 55;

      } else {
        setpoints[0] = 0;
        setpoints[1] = 0;
        setpoints[2] = 0;
        setpoints[3] = 0;
        setpoints[4] = 0;
        setpoints[5] = 0;
        setpoints[6] = 0;
        setpoints[7] = 0;
        setpoints[8] = 0;
        setpoints[9] = 0;
        setpoints[10] = 0;
        setpoints[11] = 0;
        setpoints[12] = 0;
        setpoints[13] = 0;
        setpoints[14] = 0;
        setpoints[14] = 0;

      }
      pos->positionMove(setpoints.data());
    }
  }
  return 0;
}
