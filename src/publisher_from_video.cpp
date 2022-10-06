// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>
#include <fstream>
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>


int main(int argc, char ** argv)
{

  double camera_matrix[9];
  double dist_coeffs[5];
  double new_camera_matrix[9];

  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) {return 1;}
  //std::string param_path = argv[2];
  std::ifstream param(argv[2]);

  //cameraMatrix = (cv::Mat1d(3, 3) << 166.0917851807707, 0, 153.1842438002411, 0., 168.1453750396955, 99.501620234996140, 0., 0., 1.);

  for(int i = 0; i < 9; i++) param >> camera_matrix[i];
  for(int i = 0; i < 5; i++) param >> dist_coeffs[i]; 
  for(int i = 0; i < 9; i++) param >> new_camera_matrix[i];
  cv::Mat cameraMatrix= cv::Mat(3, 3, CV_64FC1, camera_matrix);
  cv::Mat newCameraMatrix= cv::Mat(3, 3, CV_64FC1, new_camera_matrix);
  cv::Mat distCoeffs = cv::Mat(1, 5, CV_64FC1, dist_coeffs);
  //distCoeffs=(cv::Mat1d(1, 5) << -0.301297341361392, 0.057655095534194, 0.,0.,0.);
  //newCameraMatrix =(cv::Mat1d(3, 3) << 106.29722595, 0., 157.40584801, 0., 149.60151672, 101.36317909, 0., 0., 1.);


  cv::Mat map1, map2;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("image", 1);

  // Convert the command line parameter index for the video device to an integer
  // std::istringstream video_sourceCmd(argv[1]);
  std::string video_source = argv[1];

  // Check if it is indeed a number
  // if (!(video_sourceCmd >> video_source)) {return 1;}
  cv::Size out_size(320, 180);
  cv::VideoCapture cap(video_source);
  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, out_size, CV_32FC1, map1, map2);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<int>(320));
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<int>(240));
  // Check if video device can be opened with the given index
  if (!cap.isOpened()) {return 1;}
  cv::Mat frame;
  cv::Mat temp;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(30);
  while (rclcpp::ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty()) {
      cv::Mat resized;
      cv::resize(frame, resized, out_size);
      cv::remap(resized, resized, map1, map2, cv::INTER_LINEAR);
      msg = cv_bridge::CvImage(hdr, "bgr8", resized).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}
