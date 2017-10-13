#ifndef KINECT2_PROJECTOR_HPP
#define KINECT2_PROJECTOR_HPP

#include <opencv2/opencv.hpp>

class Kinect2Projector
{
public:
  Kinect2Projector() = default;

  bool loadColorCalibration(const std::string &path);
  bool loadDepthCalibration(const std::string &path);
  bool loadIrCalibration(const std::string &path);
  bool loadExtrinsicCalibration(const std::string &path);

private:


}

#endif // KINECT2_PROJECTOR_HPP
