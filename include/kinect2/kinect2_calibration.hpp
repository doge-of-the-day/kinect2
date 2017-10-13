#ifndef KINECT2CALIBRATION_HPP
#define KINECT2CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <boost/array.hpp>

struct Kinect2Calibration
{
  /// IAI format
  cv::Mat rgb_camera_matrix;
  cv::Mat rgb_distortion_coefficients;
  cv::Mat rgb_rotation_matrix;
  cv::Mat rgb_projection_matrix;

  double  depth_shift;

  cv::Mat ir_camera_matrix;
  cv::Mat ir_distortion_coefficients;
  cv::Mat ir_rotation_matrix;
  cv::Mat ir_projection_matrix;

  cv::Mat extr_rotation_matrix;
  cv::Mat extr_translation_matrix;
  cv::Mat extr_essentaial_matrix;
  cv::Mat extr_fundamental_matrix;
  cv::Mat extr_transformation;

  bool loadColorCalibration(const std::string &path)
  {
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      fs["cameraMatrix"]            >> rgb_camera_matrix;
      fs["distortionCoefficients"]  >> rgb_distortion_coefficients;
      fs["rotation"]                >> rgb_rotation_matrix;
      fs["projection"]              >> rgb_projection_matrix;
      fs.release();
    } catch (const std::exception &e) {
      std::cerr << "[Kinect2Calibration]: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool loadDepthCalibration(const std::string &path)
  {
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      fs["cameraMatrix"]            >> rgb_camera_matrix;
      fs.release();
    } catch (const std::exception &e) {
      std::cerr << "[Kinect2Calibration]: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool loadIrCalibration(const std::string &path)
  {
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      fs["cameraMatrix"]            >> ir_camera_matrix;
      fs["distortionCoefficients"]  >> ir_distortion_coefficients;
      fs["rotation"]                >> ir_rotation_matrix;
      fs["projection"]              >> ir_projection_matrix;
      fs.release();
    } catch (const std::exception &e) {
      std::cerr << "[Kinect2Calibration]: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool loadExtrinsicCalibration(const std::string &path)
  {
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      fs["rotation"]     >> extr_rotation_matrix;
      fs["translation"]  >> extr_translation_matrix;
      fs["essential"]    >> extr_essentaial_matrix;
      fs["fundamental"]  >> extr_fundamental_matrix;
      fs.release();
    } catch (const std::exception &e) {
      std::cerr << "[Kinect2Calibration]: " << e.what() << std::endl;
      return false;
    }
    extr_transformation = cv::Mat::eye(4, 4, CV_32FC1);
    extr_transformation.at<float>(0,0) = extr_rotation_matrix.at<double>(0,0);
    extr_transformation.at<float>(0,1) = extr_rotation_matrix.at<double>(0,1);
    extr_transformation.at<float>(0,2) = extr_rotation_matrix.at<double>(0,2);
    extr_transformation.at<float>(1,0) = extr_rotation_matrix.at<double>(1,0);
    extr_transformation.at<float>(1,1) = extr_rotation_matrix.at<double>(1,1);
    extr_transformation.at<float>(1,2) = extr_rotation_matrix.at<double>(1,2);
    extr_transformation.at<float>(2,0) = extr_rotation_matrix.at<double>(2,0);
    extr_transformation.at<float>(2,1) = extr_rotation_matrix.at<double>(2,1);
    extr_transformation.at<float>(2,2) = extr_rotation_matrix.at<double>(2,2);
    extr_transformation.at<float>(0,3) = extr_translation_matrix.at<double>(0);
    extr_transformation.at<float>(1,3) = extr_translation_matrix.at<double>(1);
    extr_transformation.at<float>(2,3) = extr_translation_matrix.at<double>(2);
    return true;
  }

  void getDistortionCoefficientsIR(std::vector<double> &D) const
  {
    for(int i = 0 ; i < ir_distortion_coefficients.cols ; ++i)
      D.emplace_back(ir_distortion_coefficients.at<double>(i));
  }


  void getProjectionMatrixIR(boost::array<double, 12> &P) const
  {
    P[0]  = ir_projection_matrix.at<double>(0);
    P[2]  = ir_projection_matrix.at<double>(2);
    P[5]  = ir_projection_matrix.at<double>(5);
    P[6]  = ir_projection_matrix.at<double>(6);
    P[11] = 1.0;
  }

  void getRectificationMatrixIR(boost::array<double, 9> &R) const
  {
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
  }

  void getCameraMatrixIR(boost::array<double, 9> &K) const
  {
    K[0]  = ir_camera_matrix.at<double>(0);
    K[2]  = ir_camera_matrix.at<double>(2);
    K[5]  = ir_camera_matrix.at<double>(5);
    K[6]  = ir_camera_matrix.at<double>(6);
    K[8]  = 1.0;
  }

  void getDistortionCoefficientsRGB(std::vector<double> &D) const
  {
    for(int i = 0 ; i < ir_distortion_coefficients.cols ; ++i)
      D.emplace_back(rgb_distortion_coefficients.at<double>(i));
  }

  void getProjectionMatrixRGB(boost::array<double, 12> &P) const
  {
    P[0]  = rgb_projection_matrix.at<double>(0);
    P[2]  = rgb_projection_matrix.at<double>(2);
    P[5]  = rgb_projection_matrix.at<double>(5);
    P[6]  = rgb_projection_matrix.at<double>(6);
    P[11] = 1.0;
  }

  void getRectificationMatrixRGB(boost::array<double, 9> &R) const
  {
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
  }

  void getCameraMatrixRGB(boost::array<double, 9> &K) const
  {
    K[0]  = rgb_camera_matrix.at<double>(0);
    K[2]  = rgb_camera_matrix.at<double>(2);
    K[5]  = rgb_camera_matrix.at<double>(5);
    K[6]  = rgb_camera_matrix.at<double>(6);
    K[8]  = 1.0;
  }
};


#endif // KINECT2CALIBRATION_HPP
