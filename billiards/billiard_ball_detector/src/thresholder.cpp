#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <BlobResult.h>

cv::Mat source, smoothed, source_hsv, source_channels[3];
CBlobResult blobs;

void mouseCb(int event, int x, int y, int flags, void* param)
{
  if (event != CV_EVENT_LBUTTONDOWN)
    return;

  printf("H = %d, S = %d, V = %d\n",
         (int)source_channels[0].at<uint8_t>(y, x),
         (int)source_channels[1].at<uint8_t>(y, x),
         (int)source_channels[2].at<uint8_t>(y, x));

  CBlobGetXYInside inside(cvPoint2D32f(x, y));
  for (int i = 0; i < blobs.GetNumBlobs(); ++i) {
    CBlob* blob = blobs.GetBlob(i);
    if ( inside(*blob) == 1.0 ) {
      printf("Blob stats: area = %f, compactness = %f\n", blob->Area(), CBlobGetCompactness()(*blob));
    }
  }
}

void showImage(const std::string& name, const cv::Mat& image)
{
  cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback(name.c_str(), mouseCb, NULL);
  cv::imshow(name, image);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thresholder");
  
  // Load source image
  source = cv::imread(argv[1]);
  //cv::medianBlur(source, smoothed, 5);
  smoothed = cv::Mat(source.rows, source.cols, source.type());
  cvPyrMeanShiftFiltering(&(IplImage)source, &(IplImage)smoothed, 2.0, 40.0);

  // Convert to HSV color space
  cv::cvtColor(smoothed, source_hsv, CV_BGR2HSV);
  cv::split(source_hsv, source_channels);

  // Attempt to binarize pixels into ball vs. non-ball
  cv::Mat thresh1, thresh2, thresholded;
  cv::threshold(source_channels[1], thresh1, 30.0, 255.0, CV_THRESH_BINARY);
  cv::threshold(source_channels[2], thresh2, 250.0, 255.0, CV_THRESH_BINARY);
  cv::bitwise_or(thresh1, thresh2, thresholded);
  //thresholded = thresh1;

  // Extract connected components
  blobs = CBlobResult(&(IplImage)thresholded, NULL, 0);
  blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_INSIDE, 400, 1600);
  blobs.Filter(blobs, B_EXCLUDE, CBlobGetCompactness(), B_GREATER, 5.0);

  cv::Mat blob_image = source.clone();
  IplImage blob_image_ipl = blob_image;
  CBlobGetXCenter x_center;
  CBlobGetYCenter y_center;
  for (int i = 0; i < blobs.GetNumBlobs(); ++i) {
    CBlob* blob = blobs.GetBlob(i);
    blob->FillBlob(&blob_image_ipl, CV_RGB(255,0,0) );
    //printf("(X,Y) = (%f, %f), area = %f, compactness = %f\n", x_center(*blob), y_center(*blob), blob->Area(), CBlobGetCompactness()(*blob));
  }

  // Show all the products
  showImage("image", smoothed);
  showImage("hue", source_channels[0]);
  showImage("saturation", source_channels[1]);
  showImage("value", source_channels[2]);
  showImage("thresholded", thresholded);
  showImage("blobs", blob_image);

  cvStartWindowThread();
  ros::spin();
}
