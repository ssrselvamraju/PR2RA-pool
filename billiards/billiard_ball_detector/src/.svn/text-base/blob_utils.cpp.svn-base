#include <billiard_ball_detector/blob_utils.h>
#include <vector>
#include <algorithm>

namespace blobs {

struct comparePoint : public std::binary_function<CvPoint, CvPoint, bool>
{
  bool operator()(CvPoint a, CvPoint b)
  {
    if (a.y == b.y)
      return a.x < b.x;
    return a.y < b.y;
  }
};

void foreachPixel(CBlob& blob, const cv::Mat& binarized,
                  const boost::function<void(cv::Point)>& functor)
{
  int min_x = blob.MinX() + 0.5, min_y = blob.MinY() + 0.5;
  int max_x = blob.MaxX() + 0.5, max_y = blob.MaxY() + 0.5;

  for (int y = min_y; y < max_y; ++y) {
    for (int x = min_x; x < max_x; ++x) {
      if ( binarized.at<unsigned char>(y, x) > 0 )
        functor( cv::Point(x,y) );
    }
  }
}

struct FillPoint
{
  cv::Mat image;

  void operator()(cv::Point pt)
  {
    image.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
  }
};

void draw(CBlob& blob, const cv::Mat& binarized, cv::Mat image)
{
  FillPoint functor;
  functor.image = image;
  foreachPixel(blob, binarized, functor);
}

#if 0
void draw(CBlob& blob, IplImage* image)
{
  /// @todo This is kinda buggy but should be adequate for now...
  CvSeq* edges = blob.GetExternalContour()->GetContourPoints();

  // Check the blob exists
  if( edges == NULL || edges->total == 0 ) return;

  CvPoint edgeactual, pt1, pt2;
  CvSeqReader reader;
  std::vector<CvPoint> vectorEdges( edges->total );
  std::vector<CvPoint>::iterator itEdges, itEdgesNext;
  bool dinsBlob;
  int yActual;
  
  // Form a list a points on the contours
  cvStartReadSeq( edges, &reader);
  itEdges = vectorEdges.begin();
  while( itEdges != vectorEdges.end() )
  {
    CV_READ_SEQ_ELEM( edgeactual ,reader);
    *itEdges = edgeactual;
    itEdges++;
  }
  // Sort the points into raster order
  std::sort( vectorEdges.begin(), vectorEdges.end(), comparePoint() );
  
  // Call functor for each point inside the outer contour
  itEdges = vectorEdges.begin();
  itEdgesNext = vectorEdges.begin() + 1;
  dinsBlob = true;
  while( itEdges != (vectorEdges.end() - 1))
  {
    yActual = (*itEdges).y;
    
    if( ( (*itEdges).x != (*itEdgesNext).x ) &&
        ( (*itEdgesNext).y == yActual )
      )
    {
      if( dinsBlob )
      {
        pt1 = *itEdges;
        pt2 = *itEdgesNext;

        //cvLine( image, pt1, pt2, CV_RGB(255,0,0) );
      }
      dinsBlob = !dinsBlob;
    }
    itEdges++;
    itEdgesNext++;
    if( (*itEdges).y != yActual ) dinsBlob = true;
  }
}
#endif

} // namespace blobs
