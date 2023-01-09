#ifndef POSITIONDETECTIONSYSTEM_H
#define POSITIONDETECTIONSYSTEM_H

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagPoseEstimate.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
//#include <frc/apriltag/AprilTagDetector_cv.h>

#include "Field.h"

class PositionDetectionSystem {
     public:
          PositionDetectionSystem();
     private:
          Field field;
};

#endif