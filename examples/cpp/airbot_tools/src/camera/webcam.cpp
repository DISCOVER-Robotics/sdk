#include "webcam.hpp"

WebCam::WebCam(uint16_t camera_id, uint16_t height, uint16_t width)
    : camera_id_(camera_id), frame_h_(height), frame_w_(width) {
  cap_ = cv::VideoCapture(camera_id_);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  if (!cap_.isOpened()) {
    std::cerr << "Error: Unable to open camera " << camera_id_ << std::endl;
    exit(1);
  } else {
    std::cout << "Camera " << camera_id_ << " is opened" << std::endl;
  }
}

WebCam::~WebCam() { cap_.release(); }

const cv::Mat &WebCam::get_frame() {
  bool bSuccess = cap_.read(frame_);
  if (!bSuccess) {
    std::cerr << "Cannot read a frame from camera " << camera_id_ << std::endl;
  }
  return frame_;
}

std::pair<const cv::Mat &, time_t> WebCam::get_frame_time() {
  time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  return std::make_pair(get_frame(), now);
}

void WebCam::save_frame(const std::string &filename) { cv::imwrite(filename, get_frame()); }

void WebCam::display_frame(const bool &running) {
  // Note: A bug exists. You cannot simultaneously call get_frame and
  // display_frame in two threads
  while (true) {
    cv::imshow((std::string("/dev/video") + std::to_string(camera_id_)).c_str(), get_frame());
    if (!running || cv::waitKey(1) == 'q') break;
  }
}
