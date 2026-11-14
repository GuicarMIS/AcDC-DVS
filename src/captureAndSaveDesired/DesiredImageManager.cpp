#include "DesiredImageManager.h"

#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

DesiredImageManager::DesiredImageManager():Node("CaptureAndSaveDesiredImage")
{
    this->declare_parameter<std::string>("Image_topic", "currentImage");
    this->declare_parameter<std::string>("Log_folder", "/home/belinda-naamani/.ros/ros_dvs_bridge/Id.png");
    this->declare_parameter<std::string>("joint_topic", "cmd/joint");

    this->declare_parameter<double>("focal", double(1.0));
    f_ = this->get_parameter("focal").as_double();

    this->declare_parameter<double>("pixelSize", double(1.0));
    ku_ = this->get_parameter("pixelSize").as_double();

    this->declare_parameter<double>("u0", double(1.0));
    u0_ = this->get_parameter("u0").as_double();    

    this->declare_parameter<double>("v0", double(1.0));
    v0_ = this->get_parameter("v0").as_double();

    this->declare_parameter<double>("k1", double(0.0));
    k1_ = this->get_parameter("k1").as_double();

    this->declare_parameter<double>("k2", double(0.0));
    k2_ = this->get_parameter("k2").as_double();

    this->declare_parameter<double>("k3", double(0.0));
    k3_ = this->get_parameter("k3").as_double();

    this->declare_parameter<double>("p1", double(0.0));
    p1_ = this->get_parameter("p1").as_double();

    this->declare_parameter<double>("p2", double(0.0));
    p2_ = this->get_parameter("p2").as_double();

    this->declare_parameter<double>("J1", double(52.30));
    this->declare_parameter<double>("J2", double(-72.92));
    this->declare_parameter<double>("J3", double(79.45));
    this->declare_parameter<double>("J4", double(-96.57));
    this->declare_parameter<double>("J5", double(-90.43));
    this->declare_parameter<double>("J6", double(54.13));
    this->declare_parameter<bool>("INDICATOR", true);

    INDICATOR = this->get_parameter("INDICATOR").as_bool();
    
    
    image_topic=this->get_parameter("Image_topic").as_string();
    folder_write_image= this->get_parameter("Log_folder").as_string();
    joint_topic_ = this->get_parameter("joint_topic").as_string();
    j_init.data.resize(6);
    
    j_init.data[0]=(vpMath::rad(this->get_parameter("J1").as_double()));
    j_init.data[1]=(vpMath::rad(this->get_parameter("J2").as_double()));
    j_init.data[2]=(vpMath::rad(this->get_parameter("J3").as_double())) ;
    j_init.data[3]=(vpMath::rad(this->get_parameter("J4").as_double()));
    j_init.data[4]=(vpMath::rad(this->get_parameter("J5").as_double()))  ;
    j_init.data[5]=(vpMath::rad(this->get_parameter("J6").as_double()));

    this->declare_parameter<int>("Height", false);
    Height_ = this->get_parameter("Height").as_int();

    this->declare_parameter<int>("Width", false);
    Width_ = this->get_parameter("Width").as_int();

    this->declare_parameter<int>("binning", false);
    binning_val_ = this->get_parameter("binning").as_int();
    
    Height_ /= binning_val_;
    Width_ /= binning_val_;
    
    ku_ = binning_val_*ku_;
    if (ku_>0)
        px_ = py_ = f_/ku_;
    else 
    px_ = py_ = f_;

    // 3x3 Camera Matrix [fx 0 cx; 0 fy cy; 0 0 1]
    cameraMatrix = (cv::Mat_<double>(3, 3) << 
        px_,   0.0, u0_,
          0.0, py_, v0_,
          0.0,   0.0,   1.0);

    // Distortion Coefficients [k1, k2, p1, p2, k3]
    distCoeffs = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);

    alpha = 0.0; // 0 = crop all black borders, 1 = keep all pixels

    // Compute the optimal new camera matrix
    newCameraMatrix = cv::getOptimalNewCameraMatrix(
        cameraMatrix, distCoeffs, cv::Size(Width_, Height_), alpha, cv::Size(Width_, Height_), &validRoi
    );

}
DesiredImageManager::~DesiredImageManager()
{
    
}
void DesiredImageManager::init()
{

    image_transport::ImageTransport it(this->shared_from_this());
    sub_ = it.subscribe(image_topic, 1, std::bind(&DesiredImageManager::imageCallback, this,  std::placeholders::_1));
    joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(joint_topic_,1);

    joint_pub_->publish(j_init);
    vpTime::wait(5000);
    
    
}

void DesiredImageManager::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{


    try
    {
        //m_desired_image = visp_bridge::toVispImage(*msg);
        m_desired_image = toVispImage(*msg);

        std::string file_path;
        ss_desired_image.str("");
        ss_desired_image.clear();
        ss_desired_image << folder_write_image<<"/Id_dist.png";
        file_path = ss_desired_image.str();
        filename_write_image = vpIoTools::path(file_path.c_str());
        vpImageIo::write(m_desired_image, filename_write_image);

        cv::Mat I_cv;           // Target OpenCV image
        // Convert ViSP RGBA image to OpenCV BGR Mat
        vpImageConvert::convert(m_desired_image, I_cv);
        
        cv::Mat dst;
        //std::cout << cameraMatrix << std::endl << distCoeffs << std::endl;

        cv::undistort(I_cv, dst, cameraMatrix, distCoeffs, newCameraMatrix);

        vpImageConvert::convert(dst, m_desired_image);

        ss_desired_image.str("");
        ss_desired_image.clear();
        ss_desired_image << folder_write_image<<"/Id.png";
        file_path = ss_desired_image.str();
        filename_write_image = vpIoTools::path(file_path.c_str());
        vpImageIo::write(m_desired_image, filename_write_image);

        vpTime::wait(100);
    }
    catch (const std::exception &e)
    {
        std::cout << "error DesiredImageManager::imageCallback" << std::endl;
        // Error handling in case conversion fails
        RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
    }

}

vpImage<unsigned char> DesiredImageManager::toVispImage(const sensor_msgs::msg::Image & src)
{
  using sensor_msgs::image_encodings::BGR8;
  using sensor_msgs::image_encodings::BGRA8;
  using sensor_msgs::image_encodings::MONO8;
  using sensor_msgs::image_encodings::RGB8;
  using sensor_msgs::image_encodings::RGBA8;

  vpImage<unsigned char> dst(src.height, src.width);

  std::cout << src.encoding << " " << std::endl;

  if(src.encoding == sensor_msgs::image_encodings::MONO8)
    memcpy(dst.bitmap, &(src.data[0]), dst.getHeight() * src.step * sizeof(unsigned char));
  else if(src.encoding == sensor_msgs::image_encodings::RGB8 || src.encoding == RGBA8
          || src.encoding == sensor_msgs::image_encodings::BGR8 || src.encoding == sensor_msgs::image_encodings::BGRA8)
  {
    unsigned nc = sensor_msgs::image_encodings::numChannels(src.encoding);
    unsigned cEnd = (src.encoding == RGBA8 || src.encoding == sensor_msgs::image_encodings::BGRA8) ? nc - 1 : nc;

    for(unsigned i = 0; i < dst.getWidth(); ++i)
    {
      for(unsigned j = 0; j < dst.getHeight(); ++j)
      {
        int acc = 0;
        for(unsigned c = 0; c < cEnd; ++c) acc += src.data[j * src.step + i * nc + c];
        dst[j][i] = acc / nc;
      }
    }
  }
  return dst;
}
