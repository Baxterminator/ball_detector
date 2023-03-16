// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <ball_detector/color_detector.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/xphoto/white_balance.hpp>
#include <thread>

using namespace std::this_thread;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/cameras/right_hand_camera/image", 10);
    timer_ = this->create_wall_timer(
      40ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    //auto message = sensor_msgs::msg::Image();
    std_msgs::msg::Header header;
    const std::string encoding = "bgr8";
    cv::Mat im_mat = cv::imread("/user/eleves/tcorroenne2021/ros2/src/test_projet/img/img_0.jpg"); ///home/ecn/ros2/src/test_projet/img/im_0.jpg
    auto im_bridge = cv_bridge::CvImage(header,encoding,im_mat);
    cd.detectColor(52,21,18);  //detect red (r,g,b) (143,11,14)
    //cd.showSegmentation();
    //cd.setSaturationValue(130,60);
    //cv::Ptr<cv::xphoto::WhiteBalancer> wb = cv::xphoto::createSimpleWB();
    //wb->balanceWhite(im_mat,im_white);
    im_white = im_mat;
    //cd.fitCircle();
    //cd.showOutput();
    if (!seg){
        cd.showSegmentation();
        //sleep_for(500ms);
        sleep_for(500ms);

        seg=true;
    }
    cd.process(im_white,im_cv_proccessed,true);
    //sleep_for(500ms);

    //sensor_msgs::msg::Image message = *im_bridge.toImageMsg();
    //message.data = im_bridge;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //std::cout<<"publishing"<<std::endl;
    //publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  bool seg=false;
  ecn::ColorDetector cd = ecn::ColorDetector(255,30,30);
  cv::Mat im_white;
  cv::Mat im_cv_proccessed;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
