#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera("image", 1);

#if 0
  cv::Mat image = cv::imread(argv[1]); // color
  sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&((IplImage)image), "bgr8");
#else
  cv::Mat image = cv::imread(argv[1], 0); // grayscale
  sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&((IplImage)image), "mono8");
#endif
  msg->encoding = "bayer_rggb8"; // prosilica

  ros::NodeHandle local_nh("~");
  std::string url;
  local_nh.param("camera_info_url", url, std::string());
  CameraInfoManager manager(nh, "prosilica", url);
  sensor_msgs::CameraInfo cam_info = manager.getCameraInfo();
  cam_info.header.frame_id = "high_def_optical_frame";

  ros::Rate loop_rate(4);
  while (nh.ok()) {
    pub.publish(*msg, cam_info, ros::Time::now());
    ros::spinOnce();
    loop_rate.sleep();
  }
}
