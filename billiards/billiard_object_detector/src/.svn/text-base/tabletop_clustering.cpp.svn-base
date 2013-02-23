/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Marius Muja and Matei Ciocarlie

#include "tabletop_object_detector/tabletop_clustering.h"

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/projections.h>

using std::vector;
using std::max;

namespace billiard_object_detector {

//! A helper class for point cloud clustering
class TabletopClustering::NNGridIndexer
{
  float xmin, xmax, ymin, ymax;
  float xd,yd;
  
  int resolution;
  
  float* grid;
  
public:
  NNGridIndexer(const sensor_msgs::PointCloud& cloud)
  {
    xmin = cloud.points[0].x;
    xmax = cloud.points[0].x;
    ymin = cloud.points[0].y;
    ymax = cloud.points[0].y;
    for (size_t i=1;i<cloud.get_points_size();++i) {
      if (cloud.points[i].x<xmin) xmin = cloud.points[i].x;
      if (cloud.points[i].x>xmax) xmax = cloud.points[i].x;
      if (cloud.points[i].y<ymin) ymin = cloud.points[i].y;
      if (cloud.points[i].y>ymax) ymax = cloud.points[i].y;
    }
    
    resolution = 600;
    xd = (xmax-xmin)/(resolution-1);
    yd = (ymax-ymin)/(resolution-1);
    
    grid = new float[resolution*resolution];
    memset(grid,0,resolution*resolution*sizeof(float));
    
    for (size_t i=0;i<cloud.get_points_size();++i) {
      geometry_msgs::Point32 p = cloud.points[i];
      
      int x = int((p.x-xmin)/xd+0.5);
      int y = int((p.y-ymin)/yd+0.5);
      
      float *ptr = grid+x*resolution+y;
      *ptr = max(p.z,*ptr);
    }
  }
  
  ~NNGridIndexer()
  {
    delete[] grid;
  }
  
  
  int computeMean(const geometry_msgs::Point32& p, float radius, geometry_msgs::Point32& result)
  {
    int xc = int((p.x-xmin)/xd+0.5);
    int yc = int((p.y-ymin)/yd+0.5);
    
    int xoffs = int((radius/xd)+0.5);
    int yoffs = int((radius/yd)+0.5);
    
    float xmean = 0;
    float ymean = 0;
    int count = 0;
    
    for (int x=xc-xoffs;x<=xc+xoffs;++x) {
      for (int y=yc-yoffs;y<=yc+yoffs;++y) {
	if (x<0 || x>=resolution) continue;
	if (y<0 || y>=resolution) continue;
	if (double((x-xc)*(x-xc))/(xoffs*xoffs)+double((y-yc)*(y-yc))/(yoffs*yoffs)>=1) continue;
	if (grid[x*resolution+y]==0) continue;
	
	xmean += x;
	ymean += y;
	count ++;
      }
    }
    
    
    if (count==0) return 0;
    
    xmean /= count;
    ymean /= count;
    
    result.x = xmean*xd+xmin;
    result.y = ymean*yd+ymin;
    
    return count;
  }
  
};

/*! Defines default node parameters.
*/
TabletopClustering::TabletopClustering()
{
  min_points_per_cluster_ = 1000;
  min_points_per_seed_ = 10;
  max_clustering_iterations_ = 7;
  min_object_distance_ = 0.03;
  min_object_height_ = 0.015;
  max_object_height_ = 0.35;
  cluster_radius_ = 0.10 / 2.0;
}

/*! All the functionality of this class in a single call:

  - finds and filters table plane

  - transforms objects point cloud into table coordinate system

  - clusters the result based on XY projections and gets rid of points under the table

  Most of the heavy lifting is in other functions; this is a convenient wrapper. The thing that
  it does itself is transforming the points into the table coordinate system.

 */
void TabletopClustering::findTableTopObjectClusters(const sensor_msgs::PointCloud& inputCloud,
						    vector<sensor_msgs::PointCloud>& outputClusters,
						    sensor_msgs::PointCloud& table_points,
						    tf::Transform& table_plane_trans)
{
  // find the table plane
  sensor_msgs::PointCloud objects_pc;
  sensor_msgs::PointCloud plane_pc;
  std::vector<double> table_plane_coeffs;
  filterTablePlane(inputCloud, table_plane_coeffs, objects_pc, plane_pc);
  objects_pc.header = plane_pc.header = inputCloud.header;

  //transform the objects into the table frame
  table_plane_trans = getPlaneTransform(table_plane_coeffs);
  //this should not really be a listener, as we don't need all the overhead...
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, objects_pc.header.stamp, 
                                        objects_pc.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  listener.transformPointCloud("table_frame", objects_pc, objects_pc);
  listener.transformPointCloud("table_frame", plane_pc, table_points);

  // cluster objects; also gets rid of points under the table
  findTabletopClusters(objects_pc, outputClusters);  
}

/*! First, it crops the input point cloud based on some arbitrary hard-coded limts.
  Then, it fits a plane to the result. Finally, it separates the input point cloud into
  a plane cloud and an outlier cloud, which is assumed to contain the objects on the plane.
 */
void TabletopClustering::filterTablePlane(const sensor_msgs::PointCloud& pc, 
					  vector<double>& coefficients, 
					  sensor_msgs::PointCloud& object_cloud, 
					  sensor_msgs::PointCloud& plane_cloud)
{
  sensor_msgs::PointCloud filtered_cloud;
  sensor_msgs::PointCloud filtered_outside;
  
  filterByZBounds(pc, 0.1, 1.2 , filtered_cloud, filtered_outside );
  
  vector<int> indices(filtered_cloud.get_points_size());
  for (size_t i = 0; i<filtered_cloud.get_points_size(); ++i) 
  {
    indices[i] = i;
  }
  
  vector<int> inliers;
  double dist_thresh = 0.01; // in meters
  int min_points = 200;
  
  fitSACPlane(filtered_cloud, indices, inliers, coefficients, dist_thresh, min_points);
  
  cloud_geometry::getPointCloud(filtered_cloud, inliers, plane_cloud);
  cloud_geometry::getPointCloudOutside(filtered_cloud, inliers, object_cloud);
}

/*! Calls a RANSAC fiter to fit a plane to a given set of points */
bool TabletopClustering::fitSACPlane (const sensor_msgs::PointCloud& points, const vector<int> &indices, // input
				      vector<int> &inliers, vector<double> &coeff,  // output
				      double dist_thresh, int min_points_per_model)
{
  // Create and initialize the SAC model
  sample_consensus::SACModelPlane model;
  sample_consensus::RANSAC sac(&model, dist_thresh);
  sac.setMaxIterations (100);
  model.setDataSet ((sensor_msgs::PointCloud*)&points, indices);
  
  // Search for the best plane
  if (sac.computeModel ()) 
  {
    // Compute the model coefficients
    sac.computeCoefficients (coeff);
    // Refine them using least-squares
    sac.refineCoefficients (coeff);
    
    // Get the list of inliers
    model.selectWithinDistance (coeff, dist_thresh, inliers);
    
    if ((int)inliers.size()<min_points_per_model) 
    {
      return false;
    }
    
    geometry_msgs::Point32 viewpoint;
    viewpoint.x = 0;
    viewpoint.y = 0;
    viewpoint.z = 0;
    // Flip the plane normal towards the viewpoint
    cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.points.at(inliers[0]), viewpoint);
    
    ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (), 
	      coeff[0], coeff[1], coeff[2], coeff[3]);
  }
  else 
  {
    ROS_ERROR ("Could not compute a planar model for %d points.", (int)indices.size());
    return false;
  }
  return true;
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform TabletopClustering::getPlaneTransform(std::vector<double> coeffs)
{
  ROS_ASSERT(coeffs.size() > 3);
  double a = coeffs[0], b = coeffs[1], c = coeffs[2], d = coeffs[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  /*
  btVector3 x(-b, a, 0);
  x = x.normalized();
  btVector3 y = z.cross(x).normalized();
  */
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}


/*! Crops a point cloud based on the z value of the points. Returns 2 clouds, one that contains the
  points inside the cropped area and one with the points outside.
*/
void TabletopClustering::filterByZBounds(const sensor_msgs::PointCloud& pc, double zmin, double zmax, 
					 sensor_msgs::PointCloud& filtered_pc, 
					 sensor_msgs::PointCloud& filtered_outside)
{
  vector<int> indices_remove;
  for (size_t i = 0;i<pc.get_points_size();++i) 
  {
    if (pc.points[i].z>zmax || pc.points[i].z<zmin) 
    {
      indices_remove.push_back(i);
    }
  }
  cloud_geometry::getPointCloudOutside (pc, indices_remove, filtered_pc);
  cloud_geometry::getPointCloud(pc, indices_remove, filtered_outside);
}

/*! Main clustering function. Takes in a point cloud of points, then finds all the clusters and 
  their centers. Uses mean shift clustering.
  
  Clustering is done based only on the x and y coordinates of the points. This is assumed to 
  work for tabletop objects where the point cloud has already been transformed in the table
  coordinate system, with the z axis aligned with the table normal. This is thus the equivalent
  of clustering based on the projections of the points onto the table.

  After the centers are computed, calls a separate function that prunes them and creates the
  clusters themselves by assigning points from the cloud to each center.
*/
void TabletopClustering::findTabletopClusters(const sensor_msgs::PointCloud& cloud, 
					      vector<sensor_msgs::PointCloud>& clusters)
{
  if (cloud.get_points_size()==0) return;
  vector<geometry_msgs::Point32> centers;
  
#if 0
  // initialize centers using kmeans
  const int NUM_CLUSTERS = 30;
  
  int count = cloud.get_points_size();
  CvMat* points = cvCreateMat( count, 2, CV_32FC1 );
  CvMat* labels = cvCreateMat( count, 1, CV_32SC1 );
  CvMat* centers_ = cvCreateMat( NUM_CLUSTERS, 2, CV_32FC1 );
  
  for (int i=0;i<count;++i) {
    float* ptr = (float*)(points->data.ptr + i * points->step);
    ptr[0] = cloud.points[i].x;
    ptr[1] = cloud.points[i].y;
  }
  
  cvKMeans2(points, NUM_CLUSTERS, labels, 
	    cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.005 ),
	    5,0,0,centers_);
  
  float step = cluster_radius_;
  NNGridIndexer index(cloud);
  //vector<int> indices(cloud.get_points_size());
  vector<geometry_msgs::Point32> centers(NUM_CLUSTERS);
  vector<geometry_msgs::Point32> means(NUM_CLUSTERS);
  geometry_msgs::Point32 p;
  double total_dist = 0;
  for (int i=0;i<NUM_CLUSTERS;++i) {
    float* ptr = (float*)(centers_->data.ptr + i * centers_->step);
    p.x = ptr[0];
    p.y = ptr[1];
    centers[i] = p;
    geometry_msgs::Point32 mean;
    int count = index.computeMean(p, step, mean);
    means[i]= mean;
    total_dist += dist2D(mean, p);
  }
  
  cvReleaseMat(&points);
  cvReleaseMat(&labels);
  cvReleaseMat(&centers_);
  
#else
  // initialize centers in a regular grid

  float xmin = cloud.points[0].x;
  float xmax = cloud.points[0].x;
  float ymin = cloud.points[0].y;
  float ymax = cloud.points[0].y;
  
  for (size_t i=1;i<cloud.get_points_size();++i) 
  {
    if (cloud.points[i].x<xmin) xmin = cloud.points[i].x;
    if (cloud.points[i].x>xmax) xmax = cloud.points[i].x;
    if (cloud.points[i].y<ymin) ymin = cloud.points[i].y;
    if (cloud.points[i].y>ymax) ymax = cloud.points[i].y;
  }
  
  float step = cluster_radius_;
  
  NNGridIndexer index(cloud);
  
  // getting the initial centers
  vector<geometry_msgs::Point32> means;
  // layout initial clusters in a grid
  double total_dist = 0;
  for (double x = xmin; x<xmax; x+=step/2) 
  {
    for (double y = ymin; y<ymax; y+=step/2) 
    {
      geometry_msgs::Point32 p;
      p.x = x;
      p.y = y;      
      geometry_msgs::Point32 mean;
      int found = index.computeMean(p, step, mean);      
      if (found>min_points_per_seed_) 
      {
	centers.push_back(p);
	means.push_back(mean);
	total_dist += dist2D(mean, p);
      }
    }
  }
  
#endif
  
  int iter = 0;
  // mean-shift
  bool odd = true;
  while (total_dist>0.001) 
  {    
    if (odd) 
    {
      total_dist = meanShiftIteration(cloud, index, means, centers, step);
    }
    else 
    {
      total_dist = meanShiftIteration(cloud, index, centers, means, step);
    }
    odd = !odd;
    iter++;
    
    if (iter>max_clustering_iterations_) break;
  }
  
  filterClusters(cloud, centers, clusters);
}

/*! A single iteration of mean shift clustering; called from findTabletopClusters */
double TabletopClustering::meanShiftIteration(const sensor_msgs::PointCloud& pc, NNGridIndexer& index, 
					      vector<geometry_msgs::Point32>& centers, 
					      vector<geometry_msgs::Point32>& means, float step)
{
  double total_dist = 0;
  for (size_t i=0;i<centers.size();++i) 
  {  
    geometry_msgs::Point32 mean;
    int count = index.computeMean(centers[i], step, mean);
    if (count==0) 
    {
      ROS_WARN("Got empty cluster, this should not happen\n");
    }
    double dist = dist2D(mean, centers[i]);
    total_dist += dist;
    means[i] = mean;
  }
  return total_dist;
}

/*! Does some more cleanup and filtering of clusters, based on heuristics such as overlap, height, 
  number of points, etc. Gets rid of points under the table. Then, assembles the clouds themselves 
  by assigning to each cluster the points that are within the clustering radius. 
*/
void TabletopClustering::filterClusters(const sensor_msgs::PointCloud& cloud, 
					const vector<geometry_msgs::Point32>& centers, 
					vector<sensor_msgs::PointCloud>& clusters)
{
  std::list<Cluster> cluster_list;

  // prepare clusters
  for (size_t j=0;j<centers.size();++j) 
  {
    Cluster cluster;
    cluster.seeds_.push_back(centers[j]);
    cluster.height_ = 0.0;
    cluster_list.push_back(cluster);
  }
    
  // merge clusters based on seeds
  ROS_DEBUG("Starting with %u clusters", (unsigned int) cluster_list.size());
  std::list<Cluster>::iterator it = cluster_list.begin();
  while(it != cluster_list.end())
  {
    bool merge = false;
    std::list<Cluster>::iterator it1 = it;
    it1++;
    while (it1 != cluster_list.end() )
    {
      if ( clusterSeedDistance(*it, *it1) < ((cluster_radius_ + min_object_distance_) * 
					     (cluster_radius_ + min_object_distance_)) )
      {
	merge = true;
	mergeClusters(*it, *it1);
	it1 = cluster_list.erase(it1);
      }
      else
      {
	it1 ++;
      }
    }
    if (!merge) it++;
  }
  
  // populate clusters
  ROS_DEBUG("Populating %u clusters", (unsigned int) cluster_list.size());
  for(size_t i=0;i<cloud.get_points_size();++i) 
  {
    //get rid of points under the table
    if (cloud.points[i].z < 0) continue;
    //get rid of points too high
    if (max_object_height_ >= 0.0 && cloud.points[i].z > max_object_height_) continue;

    bool inserted = false;
    for (it = cluster_list.begin(); it!=cluster_list.end(); it++)
    {
      for (size_t j=0; j<it->seeds_.size(); j++)
      {
	if (dist2D(cloud.points[i], it->seeds_[j]) < cluster_radius_*cluster_radius_) 
	{
	  it->cloud_.points.push_back( cloud.points[i] );
	  it->height_ = max( it->height_, (double)cloud.points[i].z );
	  inserted = true;
	  break;
	}
      }
    }
  }

  //prune clusters
  it = cluster_list.begin();
  while (it != cluster_list.end() )
  {
    if ( it->height_ < min_object_height_)
    {
      it = cluster_list.erase(it);
    }
    else if (it->cloud_.points.size() < min_points_per_cluster_)
    {
      it = cluster_list.erase(it);
    }
    else
    {
      it++;
    }
  }

  //return results
  ROS_DEBUG("Returning %u clusters", (unsigned int) cluster_list.size());
  clusters.clear();
  for (it = cluster_list.begin(); it!=cluster_list.end(); it++)
  {
    clusters.push_back( it->cloud_ );
  }  
}

/*! Also sets the new center of merge_into to the average of the two centers and
  its height to the max of the two heights.
*/
void TabletopClustering::mergeClusters(Cluster &merge_into, const Cluster &merge_from)
{
  size_t into_original_size = merge_into.cloud_.points.size();
  for (size_t i=0; i<merge_from.cloud_.points.size(); i++)
  {
    bool duplicate = false;
    for (size_t j=0; j<into_original_size; j++)
    {
      if ( merge_from.cloud_.points[i].x == merge_into.cloud_.points[j].x &&
	   merge_from.cloud_.points[i].y == merge_into.cloud_.points[j].y &&
	   merge_from.cloud_.points[i].z == merge_into.cloud_.points[j].z )
      {
	duplicate = true;
	break;
      }
    }
    if (!duplicate)
    {
      merge_into.cloud_.points.push_back( merge_from.cloud_.points[i] );
    }
  }
  merge_into.seeds_.insert( merge_into.seeds_.end(), merge_from.seeds_.begin(), merge_from.seeds_.end() );
  merge_into.height_ = std::max(merge_into.height_, merge_from.height_);
}

double TabletopClustering::clusterSeedDistance(const Cluster &c1, const Cluster &c2)
{
  double dist = std::numeric_limits<double>::max();
  for (size_t i=0; i<c1.seeds_.size(); i++)
  {
    for (size_t j=0; j<c2.seeds_.size(); j++)
    {
      dist = std::min( dist, dist2D(c1.seeds_[i], c2.seeds_[j]) );
    }
  }
  return dist;
}

} //namespace
