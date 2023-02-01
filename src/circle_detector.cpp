// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ball_detector/color_detector.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <string>
#include <baxter_core_msgs/msg/camera_settings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/xphoto/white_balance.hpp>



using namespace std::chrono_literals;
using namespace ecn;
using namespace std;


// a useful function to get the index of a string in a vector of strings
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
    const auto elem = std::find(names.begin(), names.end(), name);
    return std::distance(names.begin(), elem);
}


class CircleDetectorNode : public rclcpp::Node
{
public:
    CircleDetectorNode() : Node("test_node") //ou CircleDetectorNode(rclcpp::NodeOptions options) : Node("test_node", options)
    {
        // init whatever is needed for your node
        //init color
        color = declare_parameter<char>("color", 'r');
        if (color=='r'){
            cd.detectColor(91,30,30);  //detect red (r,g,b) (143,11,14)
            cd.showSegmentation();
            cd.setSaturationValue(130,60);  //value determined on real test
        }
        else {
            if(color=='y'){
                //cd = ColorDetector(255,255,240); //detect yellow
                //cd.setSaturationValue(50,200);  //value determined on real test
            }
        }
        //init side
        right = declare_parameter<bool>("right", true);
        string side = (right) ? "right" : "left";  //if right==true: side=right; else: side=left
        topic_sub_img = "/cameras/"+side+"_hand_camera/image";
        topic_sub_cam_param = "/cameras/"+side+"_hand_camera/camera_info";

        //int fov = 60;
        //cd.setCamera(640,400,fov);

        std::vector<float> K_right =  {404.378473294, 0.0, 643.580188086, 0.0, 404.378473294, 396.392557487, 0.0, 0.0, 1.0};
        std::vector<float> K_left = {403.329926796, 0.0, 656.042542988, 0.0, 403.329926796, 408.450436661, 0.0, 0.0, 1.0};
        std::vector<float> K = (right) ? K_right : K_left;
        auto px=K[0];
        auto py = K[4];
        auto u0=K[2];
        auto v0 = K[5];
        cd.setCamera(px,py,u0,v0);

        // init subscribers
        subscriber_cam_param = create_subscription<sensor_msgs::msg::CameraInfo>(
            topic_sub_cam_param,    // which topic143
            1,         // QoS
            [this](sensor_msgs::msg::CameraInfo::UniquePtr msg)    // callback are perfect for lambdas
            {
                //    [fx  0 cx]   [px 0  u0]
                //K = [ 0 fy cy] = [0  py v0]
                //    [ 0  0  1]   [0  0  1 ]
                RCLCPP_INFO(this->get_logger(), "camera set!");
                //std::vector<float> K =  {404.378473294, 0.0, 643.580188086, 0.0, 404.378473294, 396.392557487, 0.0, 0.0, 1.0};

                auto K = msg->k;
                auto px=K[0];
                auto py = K[4];
                auto u0=K[2];
                auto v0 = K[5];
                cd.setCamera(px,py,u0,v0);
        });
        subscriber = create_subscription<sensor_msgs::msg::Image>(
            topic_sub_img,    // which topic
            100,         // QoS
            [this](sensor_msgs::msg::Image::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_image = *msg;
                //std::cout<<im_init<<" _ ";
                //RCLCPP_INFO(this->get_logger(), "getting: '%s'");
                im_init=true;
                im_cv = cv_bridge::toCvCopy(last_image, "bgr8")->image;
                cv::Mat im_white;
                cv::Ptr<cv::xphoto::WhiteBalancer> wb = cv::xphoto::createSimpleWB();
                //wb->balanceWhite(im_cv,im_white);
                im_white = im_cv;
                cv::Mat im_cv_proccessed;
                cd.fitCircle();
                cd.showOutput();
                cd.process(im_white,im_cv_proccessed,true);

                //std::string path_processed = "/home/ecn/ros2/src/test_projet/img/img_processed_"+std::to_string(count)+".jpg"; //
                std::string path_processed = "/user/eleves/tcorroenne2021/ros2/src/test_projet/img/img_processed_"+std::to_string(count)+".jpg";

                //RCLCPP_INFO(this->get_logger(), path_processed);
                //cv::imwrite(path_processed,im_cv_proccessed);
                //std::string path_cam = "/home/ecn/ros2/src/test_projet/img/img_"+std::to_string(count)+".jpg";
                //std::string path_cam = "/user/eleves/tcorroenne2021/ros2/src/test_projet/img/img_"+std::to_string(count)+".jpg";

                //cv::imwrite(path_cam,im_cv);
                count++;
                std_msgs::msg::Header header;
                const std::string encoding = "bgr8";
                auto im_bridge = cv_bridge::CvImage(header,encoding,im_cv_proccessed);

                sensor_msgs::msg::Image message = *im_bridge.toImageMsg();
                publisher_img->publish(message);
            });

        // init publishers
        publisher = create_publisher<std_msgs::msg::Float32MultiArray>("circle_", 10);   // topic + QoS
        publisher_img = create_publisher<sensor_msgs::msg::Image>("robot/xdisplay", 10);   // topic + QoS
      
        // init timer 143- the function will be called with the given rate
        publish_timer = create_wall_timer(1000ms,    // rate
                                          [&](){read_image();});

    }   
//    CircleDetectorNode(rclcpp::NodeOptions options) : Node("test_node", options)

private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    string topic_sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_cam_param;
    string topic_sub_cam_param;
    sensor_msgs::msg::Image last_image;
    bool im_init=false;
    bool seg=false;
    int count = 0;
    bool right;
    char color;

    ;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_img;
    std_msgs::msg::Float32MultiArray circle;
    rclcpp::TimerBase::SharedPtr publish_timer;
    float x_c;
    float y_c;
    float area;
    cv::Mat im_cv;   //used to store the image and process it
    ColorDetector cd;//=ColorDetector(143,11,14);

    void read_image()
    {
        // use last_msg to build and publish command
        RCLCPP_INFO(this->get_logger(), "publish: '%s'");
        std::cout<<"publish"<<" _ ";
        if (im_init){
            //cd.findMainContour(im_cv);
            x_c = cd.x();
            y_c = cd.y();
            area = cd.area();
            create_message_circle(x_c, y_c, area); //put x_c,y_c,area in circle, ready to be published
            publisher->publish(circle);
        }
        if (!seg && im_init){

            cd.showSegmentation();  // also gives trackbars for saturation / value
            //seg=true;
        }
    }
    inline void create_message_circle(float x_c,float y_c,float area){
        std_msgs::msg::MultiArrayDimension dim;
        dim.label="cercle : x, y, area";
        dim.size=4;
        dim.stride=4;
        std::vector<std_msgs::msg::MultiArrayDimension> dims;
        dims.push_back(dim);
        std_msgs::msg::MultiArrayLayout layout;
        layout.dim = dims;
        layout.data_offset=0;
        circle.layout=layout;
        std::vector<float> datas={x_c,y_c,area,0};
        circle.data = datas;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

// register this plugin
//#include "rclcpp_components/register_node_macro.hpp"
//RCLCPP_COMPONENTS_REGISTER_NODE(CircleDetectorNode)
