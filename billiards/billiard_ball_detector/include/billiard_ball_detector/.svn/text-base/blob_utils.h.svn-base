#ifndef BILLIARD_BALL_DETECTOR_BLOB_UTILS_H
#define BILLIARD_BALL_DETECTOR_BLOB_UTILS_H

#include <Blob.h>
#include <opencv/cv.h>
#include <boost/function.hpp>

namespace blobs {

/// @todo Shouldn't really need binarized image for this
void foreachPixel(CBlob& blob, const cv::Mat& binarized,
                  const boost::function<void(cv::Point)>& functor);

void draw(CBlob& blob, const cv::Mat& binarized, cv::Mat image);

} // namespace blobs

#endif
