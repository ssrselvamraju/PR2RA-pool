// ROS communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <billiard_ball_detector/BlobDetectorConfig.h>

// Messages
#include <visualization_msgs/Marker.h>
#include <billiards_msgs/Constants.h>
#include <billiards_msgs/TableState.h>

// Actions
#include <actionlib/server/simple_action_server.h>
#include <billiards_msgs/GetTableStateAction.h>

// Msg->OpenCV
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_calibration_parsers/parse.h>

// OpenCV & blob detection
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <BlobResult.h>
#include <billiard_ball_detector/blob_utils.h>

#include <boost/thread.hpp>
#include <cmath>
#include <cstdio>


enum BallClassification { REJECTED_TABLE,
                          REJECTED_BEHIND,
                          REJECTED_SIZE,
                          UNKNOWN_BALL,
                          CUE,
                          RED,
                          YELLOW,
                          EIGHT };

struct BlobStats
{
  // Inputs
  cv::Mat hue, saturation;
  int low_saturation_threshold;

  // Outputs
  int hue_accumulator;
  int hue_samples;
  int sat_accumulator;
  int sat_samples;

  void clear()
  {
    hue_accumulator = hue_samples = sat_accumulator = sat_samples = 0;
  }

  void operator()(cv::Point pt)
  {
    int pt_sat = saturation.at<uint8_t>(pt);
    int pt_hue = hue.at<uint8_t>(pt);

    sat_accumulator += pt_sat;
    sat_samples++;

    // Ignore hue for points with low color saturation
    if (pt_sat > low_saturation_threshold) {
      if (pt_hue > 120) {
        // Ignore hues in (120, 160), probably outliers
        if (pt_hue < 160) return;
        // Wrap high red hues back around
        pt_hue -= 180;
      }
      hue_accumulator += pt_hue;
      hue_samples++;
    }
  }
};

class BallDetectorNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  tf::TransformListener tf_;
  image_transport::Subscriber image_sub_;
  image_transport::CameraSubscriber cam_sub_;
  bool use_cam_info_;
  sensor_msgs::CameraInfoPtr info_from_file_;
  
  ros::Publisher table_pub_;
  image_transport::Publisher smoothed_pub_;
  image_transport::Publisher binarized_pub_;
  image_transport::Publisher blobs_pub_;
  ros::Publisher info_copy_pub_;
  bool show_images_;

  sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  
  cv::Mat smoothed_, source_hsv_, source_channels_[3];
  cv::Mat saturation_thresholded_, value_thresholded_, binarized_;
  cv::Mat blob_image_;
  CBlobResult blobs_;
  boost::mutex image_mutex_;

  // Processing parameters
  int threshold_saturation_min_;
  int threshold_value_min_;
  int blob_area_absolute_min_;
  int blob_area_absolute_max_;
  double blob_area_expected_tolerance_;
  double blob_compactness_;
  int low_saturation_threshold_;
  int red_yellow_hue_threshold_;
  bool use_width_as_height_;
  //int threshold_hue_min_;
  //int threshold_hue_max_;

  // Object dimensions
  double table_length_;
  double table_width_;
  double table_height_;
  double ball_radius_;
  bool do_geometric_filtering_;
  bool filter_on_table_frame_;
  std::string reference_frame_;
  double reference_frame_offset_; // ball center height above table frame
  //tf::Vector3 table_normal_; // assume (0, 0, 1)

  typedef billiard_ball_detector::BlobDetectorConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

  typedef billiards_msgs::GetTableStateAction Action;
  typedef actionlib::SimpleActionServer<Action> ActionServer;
  boost::shared_ptr<ActionServer> as_;
  ros::Timer action_timer_;
  bool action_mode_;
  
public:
  BallDetectorNode()
    : it_(nh_),
      do_geometric_filtering_(true)
  {
    ros::NodeHandle local_nh("~");

    // Constants we need to compute the 3d ball positions and filter
    local_nh.param("table_length", table_length_, (double)billiards_msgs::Constants::TABLE_LENGTH);
    local_nh.param("table_width",  table_width_,  (double)billiards_msgs::Constants::TABLE_WIDTH);
    local_nh.param("table_height", table_height_, (double)billiards_msgs::Constants::TABLE_HEIGHT);
    local_nh.param("ball_radius",  ball_radius_,  (double)billiards_msgs::Constants::BALL_RADIUS);

    // 3 possible modes:
    //  - If use_cam_info=True, subscribe to the camera_info topic.
    //  - If cam_info_file set, load the calibration from disk.
    //  - Otherwise just do the blob detection, only useful for visualization.
    local_nh.param("use_cam_info", use_cam_info_, true);
    std::string cam_info_file;
    if (local_nh.getParam("cam_info_file", cam_info_file)) {
      info_from_file_.reset(new sensor_msgs::CameraInfo);
      std::string camera_name;
      if (!camera_calibration_parsers::readCalibration(cam_info_file, camera_name, *info_from_file_)) {
        ROS_FATAL("Couldn't load calibration from file [%s]", cam_info_file.c_str());
        exit(1);
      }
      info_from_file_->header.frame_id = "high_def_optical_frame";
      //info_from_file_->header.frame_id = "high_def_optical_frame_offset";
    }

    table_pub_ = nh_.advertise<billiards_msgs::TableState>("/table_state_vision", 0);

    local_nh.param("show_images", show_images_, true);
    if (show_images_) {
      makeWindow("smoothed");
      makeWindow("binarized");
      makeWindow("blobs");

      cvStartWindowThread();
    }
    else {
      // In non-GUI mode, publish the display images on topics
      smoothed_pub_  = it_.advertise("/ball_detector/smoothed", 1);
      binarized_pub_ = it_.advertise("/ball_detector/binarized", 1);
      blobs_pub_     = it_.advertise("/ball_detector/blobs", 1);
      // For benefit of rviz
      info_copy_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/ball_detector/camera_info", 1);
    }

    ReconfigureServer::CallbackType f = boost::bind(&BallDetectorNode::configCb, this, _1, _2);
    reconfigure_server_.setCallback(f);

    local_nh.param("action_mode", action_mode_, false);
    if (action_mode_) {
      ROS_INFO("Running as an action");
      as_.reset( new ActionServer(nh_, "vision_get_table_state") );
      as_->registerGoalCallback( boost::bind(&BallDetectorNode::goalCb, this) );
      as_->registerPreemptCallback( boost::bind(&BallDetectorNode::preemptCb, this) );
    }
    else {
      ROS_INFO("Running continuously");
      subscribeToCamera();
    }

    ROS_INFO("We're online!");
  }

  void timeoutCb(const ros::TimerEvent& e)
  {
    if (image_sub_.getNumPublishers() == 0 && cam_sub_.getNumPublishers() == 0)
      ROS_INFO("[ball_detector] Aborted, there are no publishers of image topic.");    
    else
      ROS_INFO("[ball_detector] Aborted, there are publishers on image topic, but detection took too long.");

    shutdownCameraSubscription();
    as_->setAborted();
  }

  void goalCb()
  {
    // Accept the new goal (doesn't contain anything so far)
    typedef boost::shared_ptr<const billiards_msgs::GetTableStateGoal> GoalPtr;
    GoalPtr goal = as_->acceptNewGoal();
    reference_frame_ = goal->table_frame_id;
    do_geometric_filtering_ = goal->filter_against_table;

    ROS_INFO("[ball_detector] Received new goal. Projecting to %s, geometric filtering %s",
             reference_frame_.c_str(), do_geometric_filtering_ ? "ON" : "OFF");
    
    /// @todo Still need to check isPreemptRequested?

    // Subscribe to camera stream
    ros::Duration(4.0).sleep(); // avoid motion blur from still-moving robot head
    subscribeToCamera();

    // Time out after some unreasonable length of time
    action_timer_ = nh_.createTimer(ros::Duration(45.0), boost::bind(&BallDetectorNode::timeoutCb, this, _1), true);
  }

  void preemptCb()
  {
    ROS_WARN("[ball_detector] Preempted");
    as_->setPreempted();
    action_timer_.stop();
    shutdownCameraSubscription();
  }

  void subscribeToCamera()
  {
    ROS_INFO("Subscribing to camera");
    std::string topic = nh_.resolveName("image");
    if (use_cam_info_) {
      cam_sub_    = it_.subscribeCamera(topic, 1, &BallDetectorNode::imageCb, this);
    }
    else if (info_from_file_) {
      image_sub_ = it_.subscribe(topic, 1, boost::bind(&BallDetectorNode::imageCb, this, _1, info_from_file_));
    }
    else {
      image_sub_ = it_.subscribe(topic, 1, boost::bind(&BallDetectorNode::imageCb, this, _1,
                                                       sensor_msgs::CameraInfoConstPtr()));
    }
  }

  void shutdownCameraSubscription()
  {
    ROS_INFO("Unsubscribing from camera");
    image_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (action_mode_)
      ROS_INFO("[ball_detector] Received image, performing detection");
    
    cv::Mat source = bridge_.imgMsgToCv(image_msg, "bgr8");

    image_mutex_.lock(); /// @todo exception-safe?

    // Smooth to reduce noise
    smoothed_.create(source.rows, source.cols, source.type());
    /// @todo Make smoothing configurable?
    //cv::medianBlur(source, smoothed_, 5);
    cvPyrMeanShiftFiltering(&(IplImage)source, &(IplImage)smoothed_, 2.0, 40.0);

    // Convert to HSV color space
    cv::cvtColor(smoothed_, source_hsv_, CV_BGR2HSV);
    cv::split(source_hsv_, source_channels_);

    // Attempt to binarize pixels into ball vs. non-ball
    // Ball = above minimum saturation OR above a value threshold (for specularities)
    cv::threshold(source_channels_[1], saturation_thresholded_,
                  threshold_saturation_min_, 255.0, CV_THRESH_BINARY);
    cv::threshold(source_channels_[2], value_thresholded_,
                  threshold_value_min_, 255.0, CV_THRESH_BINARY);
    cv::bitwise_or(saturation_thresholded_, value_thresholded_, binarized_);
    
    // Extract connected components
    blobs_ = CBlobResult(&(IplImage)binarized_, NULL, 0);
    blobs_.Filter(blobs_, B_INCLUDE, CBlobGetArea(), B_INSIDE, blob_area_absolute_min_, blob_area_absolute_max_);
    blobs_.Filter(blobs_, B_EXCLUDE, CBlobGetCompactness(), B_GREATER, blob_compactness_);

    // Assign states to each ball for visualization
    int num_blobs = blobs_.GetNumBlobs();
    billiards_msgs::GetTableStateResult result;
    billiards_msgs::TableState& table_state = result.state;
    table_state.balls.reserve(num_blobs);
    std::vector<BallClassification> classifications(num_blobs, UNKNOWN_BALL);
    
    // Perform filtering based on knowledge of table plane
    if (info_msg) {
      cam_model_.fromCameraInfo(info_msg);

      try {
        tf::StampedTransform transform;
        ros::Time cam_time = image_msg->header.stamp; // use image stamp since we might be mocking cam info
        /// @todo Make tf usage optional? Fall back to ball radius approach
        tf_.waitForTransform(reference_frame_, cam_model_.tfFrame(), cam_time, ros::Duration(0.5));
        tf_.lookupTransform(reference_frame_, cam_model_.tfFrame(), cam_time, transform);

        for (int i = 0; i < num_blobs; ++i) {
          // Project ray through ball centers
          CBlob* blob = blobs_.GetBlob(i);
          cv::Point2d uv(CBlobGetXCenter()(*blob), CBlobGetYCenter()(*blob));
          if (use_width_as_height_) // height is uncertain due to shadowing
            uv.y = blob->MinY() + (blob->MaxX() - blob->MinX()) * 0.5;
          cv::Point3d xyz;
          cam_model_.projectPixelTo3dRay(uv, xyz);

#if 0
          // Scale ball distance based on the blob radius. Not so accurate but fine for visualization.
          double r = std::sqrt(blob->Area()/M_PI); // observed radius in pixels
          double R = ball_radius_; // real radius in meters
          double Z = cam_model_.fx() * R / r;
          xyz *= (Z / xyz.z);
          //printf("\tBall ray %d to (%.3f, %.3f, %.3f)\n", i, xyz.x, xyz.y, xyz.z);
          tf::Point ball_pos(xyz.x, xyz.y, xyz.z);
          tf::Point cam_pos(0, 0, 0);
          std::string frame_id = cam_model_.tfFrame();
#else
          // Calculate intersection of the ray with (one ball radius above) the table plane.
          // See http://en.wikipedia.org/wiki/Line-plane_intersection for derivation.
          tf::Point Ia = transform.getOrigin();
          tf::Point Ib = transform * tf::Point(xyz.x, xyz.y, xyz.z);
          tf::Vector3 n(0, 0, 1); // Z up
          double d = -reference_frame_offset_;
          double t = (-d - Ia.dot(n)) / ((Ib - Ia).dot(n));
          tf::Point ball_pos = Ia + (Ib - Ia)*t;
          tf::Point cam_pos = Ia;
          std::string& frame_id = reference_frame_;

          if (do_geometric_filtering_) {
            // If we have a good table frame, can check that position is actually within the table.
            if (filter_on_table_frame_) {
              if (ball_pos.x() < 0.0 || ball_pos.x() > table_length_ ||
                  ball_pos.y() < 0.0 || ball_pos.y() > table_width_) {
                //ROS_INFO("Rejected ball centered at (%d, %d), outside of table", (int)uv.x, (int)uv.y);
                classifications[i] = REJECTED_TABLE;
                continue;
              }
            }
            
            // Reject blobs projected behind the camera (too high in the image)
            tf::Point ball_pos_in_cam = transform.invXform(ball_pos);
            if (ball_pos_in_cam.z() < 0.0) {
              //ROS_INFO("Rejected ball centered at (%d, %d), projects behind camera", (int)uv.x, (int)uv.y);
              classifications[i] = REJECTED_BEHIND;
              continue;
            }

            // Calculate expected area of a ball blob at the projected distance. Reject if detected
            // blob area differs too much.
            double r_expected = cam_model_.fx() * ball_radius_ / ball_pos_in_cam.z();
            double A_expected = M_PI * r_expected * r_expected;
            double A_observed = blob->Area();
            double A_error = std::abs(A_expected - A_observed) / A_expected;
            if (A_error > blob_area_expected_tolerance_) {
              // This is likely to be on the table, probably an occlusion or too low tolerance
              ROS_INFO("Rejected ball centered at (%d, %d), A_error = %f", (int)uv.x, (int)uv.y, A_error);
              classifications[i] = REJECTED_SIZE;
              continue;
            }
          }
#endif
          // Found a ball! Or just skipped the filtering.
          classifications[i] = UNKNOWN_BALL;
          table_state.balls.resize(table_state.balls.size() + 1);
          billiards_msgs::BallState& ball_msg = table_state.balls.back();
          ball_msg.pocketed = false;
          ball_msg.point.header.stamp = cam_time;
          ball_msg.point.header.frame_id = frame_id;
          tf::pointTFToMsg(ball_pos, ball_msg.point.point);
          ball_msg.group_id = 0; // ???
          ball_msg.id = -1; // to be filled in correctly later
        }
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[ball_detector] TF exception:\n%s", ex.what());
      }
    }

    // Classify the balls we've found
    BlobStats stats;
    stats.hue = source_channels_[0];
    stats.saturation = source_channels_[1];
    stats.low_saturation_threshold = low_saturation_threshold_;

    int next_red_id = 1, next_yellow_id = 9, ball_index = 0;
    
    for (int i = 0; i < blobs_.GetNumBlobs(); ++i) {
      CBlob* blob = blobs_.GetBlob(i);
      BallClassification& blob_class = classifications[i];
      if (blob_class == UNKNOWN_BALL) {
        // Something to classify!
        stats.clear();
        blobs::foreachPixel(*blob, binarized_, boost::ref(stats));
        double ave_sat = (double)stats.sat_accumulator / stats.sat_samples;
        double ave_hue = (double)stats.hue_accumulator / stats.hue_samples;

        int id = -1;
        if (ave_sat < low_saturation_threshold_) {
          /// @todo Check no more than one cue ball!
          blob_class = CUE;
          id = 0;
        }
        else if (ave_hue < red_yellow_hue_threshold_) {
          blob_class = RED;
          id = next_red_id++;
        }
        else {
          blob_class = YELLOW;
          id = next_yellow_id++;
        }
        /// @todo 8-ball :)

        if (!table_state.balls.empty())
          table_state.balls[ball_index++].id = id;
      }
    }

    // Even in action mode, we don't want to give up after finding no balls.
    if (!table_state.balls.empty())
      processTableState(result);

    // Draw blobs over source image, bounding boxes on smoothed image
    source.copyTo(blob_image_);
    IplImage blob_image_ipl = blob_image_;
    for (int i = 0; i < blobs_.GetNumBlobs(); ++i) {
      CBlob* blob = blobs_.GetBlob(i);
      BallClassification blob_class = classifications[i];
      //blobs::draw(*blob, binarized_, blob_image_);

      CvScalar color;
      switch (blob_class) {
        case REJECTED_TABLE:
          color = CV_RGB(0,0,255);
          break;
        case REJECTED_BEHIND:
          color = CV_RGB(0,255,255);
        case REJECTED_SIZE:
          color = CV_RGB(255,0,255);
          break;
        case UNKNOWN_BALL:
          ROS_WARN("A ball was left unknown!");
          color = CV_RGB(0,0,0);
          break;
        case CUE:
          color = CV_RGB(127,127,127);
          break;
        case RED:
          color = CV_RGB(255,0,0);
          break;
        case YELLOW:
          color = CV_RGB(255,255,0);
          break;
        case EIGHT:
          color = CV_RGB(224,224,224);
          break;
      }
      blob->FillBlob(&blob_image_ipl, color);

      if (blob_class >= UNKNOWN_BALL) {
        double min_x = blob->MinX(), min_y = blob->MinY(), max_x = blob->MaxX();
        double max_y = use_width_as_height_ ? min_y + (max_x - min_x) : blob->MaxY();
        cv::Point2d tl(min_x, min_y), tr(max_x, min_y), bl(min_x, max_y), br(max_x, max_y);
        cv::line(smoothed_, tl, tr, CV_RGB(0,128,0), 5);
        cv::line(smoothed_, tr, br, CV_RGB(0,128,0), 5);
        cv::line(smoothed_, br, bl, CV_RGB(0,128,0), 5);
        cv::line(smoothed_, bl, tl, CV_RGB(0,128,0), 5);
        cv::line(smoothed_, tr, bl, CV_RGB(0,128,0), 5);
        cv::line(smoothed_, br, tl, CV_RGB(0,128,0), 5);
      }
    }

    // Must release the mutex before calling cv::imshow, or can deadlock against OpenCV's window mutex.
    image_mutex_.unlock();
    
    // Display images
    showImage(smoothed_, smoothed_pub_, "smoothed");
    showImage(binarized_, binarized_pub_, "binarized");
    showImage(blob_image_, blobs_pub_, "blobs");
    if (info_copy_pub_ && info_msg)
      info_copy_pub_.publish(info_msg);
  }

  void processTableState(const billiards_msgs::GetTableStateResult& result)
  {
    table_pub_.publish(result.state);

    if (!action_mode_) return;

    /// @todo Accumulate several detections for better stability

    as_->setSucceeded(result);
    action_timer_.stop();
    // Shut down camera subscription and wait for the next goal
    shutdownCameraSubscription();
  }

  void configCb(Config& config, uint32_t level)
  {
    threshold_saturation_min_ = config.threshold_saturation_min;
    threshold_value_min_ = config.threshold_value_min;
    blob_area_absolute_min_ = config.blob_area_absolute_min;
    blob_area_absolute_max_ = config.blob_area_absolute_max;
    blob_area_expected_tolerance_ = (double)config.blob_area_expected_tolerance / 100.0;
    blob_compactness_ = config.blob_compactness;

    filter_on_table_frame_ = config.filter_on_table_frame;
    if (config.use_table_frame) {
      reference_frame_ = "/table";
      reference_frame_offset_ = ball_radius_;
    }
    else {
      reference_frame_ = "/base_footprint";
      reference_frame_offset_ = table_height_ + ball_radius_;
    }

    low_saturation_threshold_ = config.low_saturation_threshold;
    red_yellow_hue_threshold_ = config.red_yellow_hue_threshold;

    use_width_as_height_ = config.use_width_as_height;
    
    //threshold_hue_min_ = config.threshold_hue_min;
    //threshold_hue_max_ = config.threshold_hue_max;
  }

  void makeWindow(const std::string& name)
  {
    cv::namedWindow(name, 0);//CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback(name.c_str(), &BallDetectorNode::mouseCb, this);
  }

  void showImage(const cv::Mat& image, const image_transport::Publisher& pub, const std::string& window) {
    if (image.empty()) return;
    if (show_images_)
      cv::imshow(window, image);
    if (pub && pub.getNumSubscribers() > 0) {
      IplImage ipl = image;
      pub.publish(sensor_msgs::CvBridge::cvToImgMsg(&ipl, (image.channels() > 1) ? "bgr8" : "mono8"));
    }
  }

  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_LBUTTONDOWN)
      return;

    BallDetectorNode* _this = (BallDetectorNode*)param;
    boost::lock_guard<boost::mutex> guard(_this->image_mutex_);

    printf("Pixel (%d, %d): H = %d, S = %d, V = %d\n", x, y,
           (int)_this->source_channels_[0].at<uint8_t>(y, x),
           (int)_this->source_channels_[1].at<uint8_t>(y, x),
           (int)_this->source_channels_[2].at<uint8_t>(y, x));

    CBlobGetXYInside inside(cvPoint2D32f(x, y));
    BlobStats stats;
    stats.hue = _this->source_channels_[0];
    stats.saturation = _this->source_channels_[1];
    stats.low_saturation_threshold = _this->low_saturation_threshold_;
    
    for (int i = 0; i < _this->blobs_.GetNumBlobs(); ++i) {
      CBlob* blob = _this->blobs_.GetBlob(i);
      if ( inside(*blob) == 1.0 ) {
        double area = blob->Area();
        double radius = std::sqrt(area/M_PI);
        printf("\tBlob area = %f\n\tCompactness = %f\n\tRadius = %f\n", area, CBlobGetCompactness()(*blob), radius);

        stats.clear();
        blobs::foreachPixel(*blob, _this->binarized_, boost::ref(stats));
        double ave_sat = (double)stats.sat_accumulator / stats.sat_samples;
        double ave_hue = (double)stats.hue_accumulator / stats.hue_samples;
        printf("\tAve saturation = %.1f from %d samples\n", ave_sat, stats.sat_samples);
        printf("\tAve hue = %.1f from %d samples\n", ave_hue, stats.hue_samples);
        
        break;
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector");
  BallDetectorNode node;
  ros::spin();
}


#if 0
class BlobExpectedAreaFilter : public COperadorBlob
{
public:
  BlobExpectedAreaFilter()
  {
  }

  virtual double operator()(const CBlob& blob) const
  {

  }

  virtual const char* GetNom() const { return "BlobExpectedAreaFilter"; }
};
#endif

// Graveyarded thresholding experiments
#if 0
    // Non-ball = above min hue threshold OR below max hue threshold (red wraps around)
    cv::threshold(source_channels_[0], saturation_thresholded_,
                  threshold_hue_max_, 255.0, CV_THRESH_BINARY);
    cv::threshold(source_channels_[0], value_thresholded_,
                  threshold_hue_min_, 255.0, CV_THRESH_BINARY_INV);
    cv::bitwise_or(saturation_thresholded_, value_thresholded_, binarized_);
#endif
    
#if 0
    // Non-ball = same as above
    cv::threshold(source_channels_[0], saturation_thresholded_,
                  /*threshold_saturation_min_*/130.0, 255.0, CV_THRESH_BINARY);
    cv::threshold(source_channels_[0], value_thresholded_,
                  /*threshold_value_min_*/10.0, 255.0, CV_THRESH_BINARY_INV);
    cv::bitwise_or(saturation_thresholded_, value_thresholded_, binarized_);
    // Also require low saturation
    cv::threshold(source_channels_[1], saturation_thresholded_,
                  threshold_saturation_min_, 255.0, CV_THRESH_BINARY_INV);
    cv::bitwise_and(saturation_thresholded_, binarized_, binarized_);
    // Try to separate table from background, then fill in gaps from the balls
    //cv::erode(binarized_, binarized_, cv::Mat(), cv::Point(-1,-1), 15);
    //cv::dilate(binarized_, binarized_, cv::Mat(), cv::Point(-1, -1), 30);
    //cv::erode(binarized_, binarized_, cv::Mat(), cv::Point(-1,-1), 10);
#endif
