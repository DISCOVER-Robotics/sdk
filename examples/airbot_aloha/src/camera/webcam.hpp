#ifndef WEBCAM_HPP
#define WEBCAM_HPP
#include <chrono>
#include <ctime>
#include <iostream>

#undef OK        // Avoid conflict with ncurses
#undef KEY_UP    // Avoid conflict with ncurses
#undef KEY_DOWN  // Avoid conflict with ncurses
#include <opencv2/opencv.hpp>
class WebCam {
 private:
  cv::VideoCapture cap_;
  uint16_t frame_h_;
  uint16_t frame_w_;
  uint16_t camera_id_;
  cv::Mat frame_;

 public:
  WebCam(uint16_t camera_id, uint16_t height = 480, uint16_t width = 640);
  ~WebCam();

  const cv::Mat &get_frame();

  std::pair<const cv::Mat &, time_t> get_frame_time();

  void save_frame(const std::string &filename);

  void display_frame(const bool &running);
};
#endif
