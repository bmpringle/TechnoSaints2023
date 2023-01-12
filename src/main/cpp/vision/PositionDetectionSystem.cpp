#include "vision/PositionDetectionSystem.h"

#include <iostream>

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagDetection.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagPoseEstimate.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>

//unfixable warnings in opencv, so use these preprocesser directives to suppress the warnings
#if defined(__clang__)
     #pragma GCC diagnostic push
     #pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
     #include "opencv2/videoio.hpp"
     #include <opencv2/imgproc.hpp>
     #pragma GCC diagnostic pop
#elif defined(__GNUC__) || defined(__GNUG__)
     #pragma GCC diagnostic push
     #pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
     #include "opencv2/videoio.hpp"
     #include <opencv2/imgproc.hpp>
     #pragma GCC diagnostic pop
#elif defined(_MSC_VER)
     #pragma warning(push, 0)
     #include "opencv2/videoio.hpp"
     #include <opencv2/imgproc.hpp>
     #pragma warning(pop)
#endif

PositionDetectionSystem::PositionDetectionSystem() : field(Field()), cameras({}), detector(frc::AprilTagDetector()), outputStream(cs::CvSource("video", cs::VideoMode::PixelFormat::kMJPEG, 640, 480, 30)), mjpegServer2(cs::MjpegServer("server-video", 1182)) {
     auto cameraInfo = cs::UsbCamera::EnumerateUsbCameras();
     
     for(cs::UsbCameraInfo& usbInfo : cameraInfo) {
          std::cout << "USB CAMERA DETECTED" << std::endl;
          std::cout << "-------------------" << std::endl;
          std::cout << "\tname: " << usbInfo.name << std::endl;
          std::cout << "\tdevID: " << usbInfo.dev << std::endl;

          cameras.push_back(cs::UsbCamera(usbInfo.name, usbInfo.dev));

          cs::CvSink cameraSink("usb camera sink: " + cameras.at(cameras.size() - 1).GetName());
          cameraSink.SetSource(cameras.at(cameras.size() - 1));
          cvSinks.push_back(cameraSink);
     }

     detector.AddFamily("tag16h5");

     mjpegServer2.SetSource(outputStream);

     FieldElement chargeStationBlue;
     chargeStationBlue.elementIdentifier = "chargeStationBlue";
     //distance from grid tape line to center of charge station - charge station width / 2 + distance from grid tape line to edge of field
     chargeStationBlue.xBegin = 114.8125; // 8 * 12 + 21 / 8 - (6 * 12 + 33 / 8) / 2 + (4 * 12 + 6.25)
     //chargeStationBlue xBegin + chargeStationBlue width
     chargeStationBlue.xEnd = chargeStationBlue.xBegin + 76.125; //72 + 33 / 8
     chargeStationBlue.yEnd = 132.375; //first edge of the community boundary on the y axis 
     chargeStationBlue.yBegin = chargeStationBlue.yEnd - 97.25; // chargeStation yEnd - chargeStation length

     field.fieldElements.push_back(chargeStationBlue);

     FieldElement chargeStationRed;
     chargeStationRed.elementIdentifier = "chargeStationRed";
     chargeStationRed.xBegin = field.length - chargeStationBlue.xBegin;
     chargeStationRed.yBegin = chargeStationBlue.yBegin;
     chargeStationRed.yEnd = chargeStationBlue.yEnd;
     chargeStationRed.xEnd = chargeStationRed.xBegin + 76.125;

     field.fieldElements.push_back(chargeStationRed);

     FieldElement blueGrids;
     blueGrids.elementIdentifier = "blueGrids";
     blueGrids.xBegin = 0;
     blueGrids.xEnd = 54.25;
     blueGrids.yBegin = 0;
     blueGrids.yEnd = 216.5;

     FieldElement* blueOuterGridLower = new FieldElement();
     blueOuterGridLower->elementIdentifier = "blueOuterGridLower";
     blueOuterGridLower->xBegin = 0;
     blueOuterGridLower->yBegin = 0;
     blueOuterGridLower->xEnd = 54.25;
     blueOuterGridLower->yEnd = 75;

     blueGrids.subElements.push_back(blueOuterGridLower);

     FieldElement* blueOuterGridMiddle = new FieldElement();
     blueOuterGridMiddle->elementIdentifier = "blueOuterGridMiddle";
     blueOuterGridMiddle->xBegin = 0;
     blueOuterGridMiddle->yBegin = 75;
     blueOuterGridMiddle->xEnd = 54.25;
     blueOuterGridMiddle->yEnd = 141;

     blueGrids.subElements.push_back(blueOuterGridMiddle);

     FieldElement* blueOuterGridUpper = new FieldElement();
     blueOuterGridUpper->elementIdentifier = "blueOuterGridUpper";
     blueOuterGridUpper->xBegin = 0;
     blueOuterGridUpper->yBegin = 141;
     blueOuterGridUpper->xEnd = 54.25;
     blueOuterGridUpper->yEnd = 216; //off by 0.5 from blueGrids for some reason?!?

     blueGrids.subElements.push_back(blueOuterGridUpper);

     field.fieldElements.push_back(blueGrids);

     FieldElement redGrids;
     redGrids.elementIdentifier = "redGrids";
     redGrids.xBegin = field.length - 54.25;
     redGrids.xEnd = field.length;
     redGrids.yBegin = 0;
     redGrids.yEnd = 216.5;

     FieldElement* redOuterGridLower = new FieldElement();
     redOuterGridLower->elementIdentifier = "redOuterGridLower";
     redOuterGridLower->xBegin = field.length - 54.25;
     redOuterGridLower->yBegin = 0;
     redOuterGridLower->xEnd = field.length;
     redOuterGridLower->yEnd = 75;

     redGrids.subElements.push_back(redOuterGridLower);

     FieldElement* redOuterGridMiddle = new FieldElement();
     redOuterGridMiddle->elementIdentifier = "redOuterGridMiddle";
     redOuterGridMiddle->xBegin = field.length - 54.25;
     redOuterGridMiddle->yBegin = 75;
     redOuterGridMiddle->xEnd = field.length;
     redOuterGridMiddle->yEnd = 141;

     redGrids.subElements.push_back(redOuterGridMiddle);

     FieldElement* redOuterGridUpper = new FieldElement();
     redOuterGridUpper->elementIdentifier = "redOuterGridUpper";
     redOuterGridUpper->xBegin = field.length - 54.25;
     redOuterGridUpper->yBegin = 141;
     redOuterGridUpper->xEnd = field.length;
     redOuterGridUpper->yEnd = 216; //off by 0.5 from blueGrids for some reason?!?

     redGrids.subElements.push_back(redOuterGridUpper);

     field.fieldElements.push_back(redGrids);

     std::vector<FieldMarker> aprilTags;

     FieldMarker aprilTag;
     aprilTag.markerIdentifier = "aprilTag-8";
     aprilTag.x = 54.25 - 16; //edge of grid - length of hybrid node
     aprilTag.y = 35;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-7";
     aprilTag.x = 54.25 - 16; //edge of grid - length of hybrid node
     aprilTag.y = 90.5;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-6";
     aprilTag.x = 54.25 - 16; //edge of grid - length of hybrid node
     aprilTag.y = 146;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-5";
     aprilTag.x = 14; //width of the double substation
     aprilTag.y = field.width - 48; //width of the field - half the substation length
     aprilTag.z = 12.375;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-1";
     aprilTag.x = field.length - (54.25 - 16); //field length - (edge of grid - length of hybrid node)
     aprilTag.y = 35;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-2";
     aprilTag.x = field.length - (54.25 - 16); //field length - (edge of grid - length of hybrid node)
     aprilTag.y = 90.5;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-3";
     aprilTag.x = field.length - (54.25 - 16); //field length - (edge of grid - length of hybrid node)
     aprilTag.y = 146;
     aprilTag.z = 18.25;

     aprilTags.push_back(aprilTag);

     aprilTag.markerIdentifier = "aprilTag-4";
     aprilTag.x = field.length - 14; //field length - width of the double substation
     aprilTag.y = field.width - 48; //width of the field - half the substation length
     aprilTag.z = 12.375;

     aprilTags.push_back(aprilTag);

     field.fieldMarkers = aprilTags;
}

void freeAllSubElements(FieldElement* element) {
     for(FieldElement* subElement : element->subElements) {
          freeAllSubElements(subElement);
     }
     delete element;
}

PositionDetectionSystem::~PositionDetectionSystem() {
     for(FieldElement& element : field.fieldElements) {
          for(FieldElement* subElement : element.subElements) {
               freeAllSubElements(subElement);
          }
     }
}

void PositionDetectionSystem::update() {
     for(cs::CvSink& sink : cvSinks) {
          cv::Mat img;
          uint64_t status = sink.GrabFrame(img);
          frc::AprilTagDetector::Results aprilTags = detector.Detect(img.cols, img.rows, img.data);
          for(const frc::AprilTagDetection* aprilTag : aprilTags) {
               auto center = aprilTag->GetCenter();
               cv::rectangle(img, cv::Point(aprilTag->GetCorner(0).x, aprilTag->GetCorner(0).y), cv::Point(aprilTag->GetCorner(2).x, aprilTag->GetCorner(2).y), cv::Scalar(0, 255, 0));
               cv::circle(img, cv::Point(center.x, center.y), 5, cv::Scalar(255, 0, 0), -1);
               cv::putText(img, std::string("tag id: ") + std::to_string(aprilTag->GetId()), cv::Point(center.x, center.y + 10), cv::FONT_HERSHEY_SIMPLEX, 10, cv::Scalar(0, 0, 255));
          }
          outputStream.PutFrame(img); //todo, add vectors for these so that each video stream has its own output 
     }
}