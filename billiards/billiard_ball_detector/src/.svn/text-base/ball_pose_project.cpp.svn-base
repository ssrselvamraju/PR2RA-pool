#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <billiards_msgs/TableState.h>
#include <boost/foreach.hpp>

class BallDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_;
  sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  billiards_msgs::TableStateConstPtr table_state_;

public:
  BallDrawer()
    : it_(nh_)
  {
    sub_ = it_.subscribeCamera("image", 1, &BallDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
  }

  void tableCb(const billiards_msgs::TableStateConstPtr& table_msg)
  {
    // Hang on to table state for use with next prosilica image
    table_state_ = table_msg;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!table_state_) {
      ROS_INFO("[ball_pose_project] Waiting for table state");
      return;
    }
    ROS_INFO("[ball_pose_project] Have table state, processing");
    
    cv::Mat image = bridge_.imgMsgToCv(image_msg, "bgr8");
    cam_model_.fromCameraInfo(info_msg);

    cv::Mat display = image.clone();
    BOOST_FOREACH(const billiards_msgs::BallState& ball, table_state_->balls) {
      geometry_msgs::PointStamped out;
      const geometry_msgs::PointStamped& in = ball.point;
      // Assuming robot pose doesn't change between tableCb and imageCb
      try {
        tf_.waitForTransform(cam_model_.tfFrame(), in.header.frame_id, in.header.stamp, ros::Duration(0.5));
        tf_.transformPoint(cam_model_.tfFrame(), in, out);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[ball_pose_project] TF exception:\n%s", ex.what());
        return;
      }

      cv::Point3d pt3d(out.point.x, out.point.y, out.point.z);
      cv::Point2d uv;
      cam_model_.project3dToPixel(pt3d, uv);

      static const int RADIUS = 9;
      cv::circle(display, uv, RADIUS, CV_RGB(255,0,0), -1);
    }

    cv::Mat display_small;
    cv::resize(display, display_small, cv::Size(640, 536));
    pub_.publish( bridge_.cvToImgMsg(&(IplImage)display_small, "bgr8") );
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_pose_project");
  BallDrawer node;
  ros::spin();
}
