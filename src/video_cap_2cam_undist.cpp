#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

int main(){

  string cpp_file_path = __FILE__;
  string calibration_dir_path = cpp_file_path.substr(0, cpp_file_path.rfind("video_cap_2cam_undist"));
  string left_cam_calibration_file = calibration_dir_path + "../calibrationdata/left_opencv.yaml";
  string right_cam_calibration_file = calibration_dir_path + "../calibrationdata/right_opencv.yaml";

  cv::VideoCapture cap_0("/dev/video0");
  cv::VideoCapture cap_1("/dev/video1");

  cap_0.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_0.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));
  cap_1.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_1.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));

  cv::Size image_size;
  image_size.height = 1080;
  image_size.width = 1920;

  // // Check if camera opened successfully
  // if(!cap_0.isOpened() || !cap_1.isOpened()){
  //   cout << "Error opening video stream or file" << endl;
  //   return -1;
  // }

  cv::FileStorage fs(left_cam_calibration_file, cv::FileStorage::READ);
  cv::Mat left_intrinsic_matrix, left_distortion_coeffs;
  fs["camera_matrix"] >> left_intrinsic_matrix;
  fs["distortion_coefficients"] >> left_distortion_coeffs;

  fs.open("../calibrationdata/right_opencv.yaml", cv::FileStorage::READ);
  cv::Mat right_intrinsic_matrix, right_distortion_coeffs;
  fs["camera_matrix"] >> right_intrinsic_matrix;
  fs["distortion_coefficients"] >> right_distortion_coeffs;

  fs.release();


  cv::Mat left_undist_map, left_undist_additional_map;
  cv::initUndistortRectifyMap(
    left_intrinsic_matrix,
    left_distortion_coeffs,
    cv::Mat(),
    left_intrinsic_matrix,
    image_size,
    CV_16SC2,
    left_undist_map, left_undist_additional_map
  );

  for(;;) {

      // milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

      cv::Mat left_image;
      cap_0 >> left_image;
      if( left_image.empty() ) break;
      cv::remap(left_image, left_image, left_undist_map, left_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
      cv::imshow("Undistorted", left_image);
      if((cv::waitKey(10) & 255) == 27) break;

      // milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      // milliseconds ms_diff = ms_cur - ms_prev;
      // float fps = 1000.0/(float)ms_diff.count();
      // std::cout << "delay: " << (int)ms_diff.count() << std::endl;
      // ms_prev = ms_cur;
  }











//   Mat frame_0;
//   Mat frame_1;
//
//   while(1){
//
//     // Capture frame-by-frame
//     cap_0 >> frame_0;
//     cap_1 >> frame_1;
//
//     // If the frame is empty, break immediately
//     if (frame_0.empty() || frame_1.empty()) {
//       cout << "Some frame is empty" << endl;
//       break;
//     }
//
//     std::cout << frame_0.rows << " " << frame_0.cols << std::endl;
//     std::cout << frame_1.rows << " " << frame_1.cols << std::endl;
//
//     // Display the resulting frame
//     namedWindow("video0", WINDOW_NORMAL);
//     resizeWindow("video0", 900, 506);
//     imshow( "video0", frame_0 );
//
//     namedWindow("video1", WINDOW_NORMAL);
//     resizeWindow("video1", 900, 506);
//     imshow( "video1", frame_1 );
//
//     // Press  ESC on keyboard to exit
//     char c=(char)waitKey(25);
//     if(c==27)
//       break;
//
//     // milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
//     // milliseconds ms_diff = ms_cur - ms_prev;
//     // float fps = 1000.0/(float)ms_diff.count();
//     // std::cout << "fps: " << fps << std::endl;
//     // ms_prev = ms_cur;
//   }
//
  // When everything done, release the video capture object
  cap_0.release();
  cap_1.release();
//
//   // Closes all the frames
  cv::destroyAllWindows();
//
  return 0;
}
