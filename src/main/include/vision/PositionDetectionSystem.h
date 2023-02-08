#ifndef POSITIONDETECTIONSYSTEM_H
#define POSITIONDETECTIONSYSTEM_H

#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include "cscore_cv.h"

#include "Field.h"
#include<vector>

#include <frc/apriltag/AprilTagDetector.h>

#include <thread>
#include <mutex>
#include <map>

class PositionDetectionSystem {
     public:
          PositionDetectionSystem(int cameraCount = 1);
          ~PositionDetectionSystem();

          void update();
     private:
          Field field;

          int cameraCount;
};

#endif