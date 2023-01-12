#ifndef POSITIONDETECTIONSYSTEM_H
#define POSITIONDETECTIONSYSTEM_H

#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include "cscore_cv.h"

#include "Field.h"
#include<vector>

#include <frc/apriltag/AprilTagDetector.h>

class PositionDetectionSystem {
     public:
          PositionDetectionSystem();
          ~PositionDetectionSystem();

          void update();
     private:
          Field field;
          std::vector<cs::UsbCamera> cameras;
          std::vector<cs::CvSink> cvSinks;
          
          frc::AprilTagDetector detector;

          cs::CvSource outputStream;
          cs::MjpegServer mjpegServer2;
};

#endif