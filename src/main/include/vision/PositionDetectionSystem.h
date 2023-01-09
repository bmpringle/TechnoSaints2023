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
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>

#include "Field.h"
#include<vector>

class PositionDetectionSystem {
     public:
          PositionDetectionSystem();
          ~PositionDetectionSystem();
     private:
          Field field;
          std::vector<cs::UsbCamera> cameras;
};

#endif