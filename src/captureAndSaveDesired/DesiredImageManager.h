#ifndef __DesiredImageManager_H__
#define __DesiredImageManager_H__

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpImageTools.h>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std;
using namespace visp;

class DesiredImageManager:public rclcpp::Node
{
    public:
        DesiredImageManager();
        ~DesiredImageManager();
        void init();
        
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        std::string image_topic;
        std::string filename_write_image;
        std::string folder_write_image;

        vpImage<unsigned char> toVispImage(const sensor_msgs::msg::Image & src);
        
    private:
        double f_;
        double ku_;
        double px_;
        double py_;
        double u0_,v0_;
        double k1_,k2_,k3_;
        double p1_,p2_;
        int binning_val_;
        int Height_, Width_;

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::Rect validRoi;
        double alpha; // 0 = crop all black borders, 1 = keep all pixels
        cv::Mat newCameraMatrix;


        bool INDICATOR;
        image_transport::Subscriber sub_;
        std::string joint_topic_;
        std_msgs::msg::Float64MultiArray j_init;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
        vpImage<unsigned char> desired_image_temp;
        vpImage<unsigned char> m_desired_image;
        std::stringstream ss_desired_image;
        std::ofstream positions;
 
};
#endif