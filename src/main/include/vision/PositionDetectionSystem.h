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

namespace pds {
     void cameraThreadFunction(int cameraCount);

     Field createField();

     void freeField(Field& field);
}

#endif