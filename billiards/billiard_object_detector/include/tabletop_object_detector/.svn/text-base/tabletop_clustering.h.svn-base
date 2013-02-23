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

#ifndef _TABLETOP_DETECTOR_H_
#define _TABLETOP_DETECTOR_H_

#include <vector>
#include <algorithm>

//tf, for transforming points to table reference system
//no transforms are actually broadcasted
#include <tf/transform_listener.h>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"

namespace billiard_object_detector {

//! Clusters objects on a table into individual point clouds
/*! Receives as input a point cloud which is assumed to be of a table with a number of objects sitting
  on it. It then proceeds through the following stages:
  
  - find the dominant plane, which is assumed to be the table. This plane is then removed from the 
  point cloud. Its coordinates can alse be returned to the caller.

  - transforms the points into the table reference frame, with the z axis aligned with the table normal

  - clusters the results using mean shift clustering.

  Output is a point cloud for each independent "object" found like this, a set of centers of the 
  projected clusters, as well as the coefficients of the table plane.

  NOTE: the output clusters are in the table reference frame. However, this class does NOT broadcast
  this frame (as to avoid side effects). Be careful using the resulting clusters, suggested use is 
  to broadcast the table frame, named however you want to, and set the headers of the clusters 
  accordingly.
*/
class TabletopClustering
{
private:  
  //! Indicates if debug information should be displayed
  bool display_;
  
  //various parameters for operation
  //! Min points for generating a seed center with uniform seeding
  int min_points_per_seed_;
  //! Maximum number of iterations of mean shift allowed
  int max_clustering_iterations_;
  //! Final clusters with fewer than this number of points will be pruned
  unsigned int min_points_per_cluster_;
  //! Min distance between two objects - clusters with centers closer than this will be merged
  double min_object_distance_;
  //! Clusters shorter than this will be pruned
  double min_object_height_;
  //! Points higher above the table than this will be pruned. Use < 0 for no pruning.
  double max_object_height_;
  //! Points closer than this distance to a cluster center will get assigned to that cluster
  double cluster_radius_;

  //! A helper class for mean shift clustering
  class NNGridIndexer;

  //! A helper class for merging clusters
  struct Cluster {
    //! The actual points in this cluster
    sensor_msgs::PointCloud cloud_;
    //! The seeds used to populate this cluster
    std::vector<geometry_msgs::Point32> seeds_;
    //! The max z value in this cluster
    double height_;
  };

  //! Main function for finding table plane and computing point cloud without it
  void filterTablePlane(const sensor_msgs::PointCloud& pc, std::vector<double>& coefficients, 
			sensor_msgs::PointCloud& object_cloud, sensor_msgs::PointCloud& plane_cloud);
  //! Fits a RANSAC plane to a point cloud
  bool fitSACPlane (const sensor_msgs::PointCloud& points, const std::vector<int> &indices,
		    std::vector<int> &inliers, std::vector<double> &coeff,
		    double dist_thresh, int min_points_per_model);
  //! Crops a point cloud along the z axis
  void filterByZBounds(const sensor_msgs::PointCloud& pc, double zmin, double zmax, 
		       sensor_msgs::PointCloud& filtered_pc, sensor_msgs::PointCloud& filtered_outside);
  //! Projects a point cloud onto a plane and computes a new cloud with the projected points
  void projectToPlane(const sensor_msgs::PointCloud& objects, const std::vector<double>& plane, 
		      sensor_msgs::PointCloud& projected_objects);
  //! Gets a tf_pose into a plane's coordinate system, so that the z axis is normal to the plane
  tf::Transform getPlaneTransform(std::vector<double> coeffs);
  //! Helper function for clustering, note that distances are defined in the XY plane
  template<typename T>
    static double dist2D(const T& a, const T& b)
    {
      return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
    }
  
  //! Main function for clustering, based on projections on the XY plane. Does mean shift, then cleanup
  void findTabletopClusters(const sensor_msgs::PointCloud& cloud, 
			    std::vector<sensor_msgs::PointCloud>& clusters);
  //! A single mean shift iteration
  double meanShiftIteration(const sensor_msgs::PointCloud& pc, NNGridIndexer& index, 
			    std::vector<geometry_msgs::Point32>& centers, 
			    std::vector<geometry_msgs::Point32>& means, float step);
  //! Cleanup of clusters based on heuristics
  void filterClusters(const sensor_msgs::PointCloud& cloud, 
		      const std::vector<geometry_msgs::Point32>& centers, 
		      std::vector<sensor_msgs::PointCloud>& clusters);
  
  //! Merges two clusters together by putting points from merge_from into merge_into
  void mergeClusters(Cluster &merge_into, const Cluster &merge_from);    

  //! Returns the minimum distance between two seeds of these clusters
  double clusterSeedDistance(const Cluster &c1, const Cluster &c2);
 public:
  
  //! Initializes parameters to default values
  TabletopClustering();
  //! Just a stub for now
  ~TabletopClustering(){}

  //! Offers all the functionality of this class in a single call
  void findTableTopObjectClusters(const sensor_msgs::PointCloud& inputCloud, 
				  std::vector<sensor_msgs::PointCloud>& outputClusters,
				  sensor_msgs::PointCloud& table_points,
				  tf::Transform& table_plane_trans);
};

} //namespace

#endif
