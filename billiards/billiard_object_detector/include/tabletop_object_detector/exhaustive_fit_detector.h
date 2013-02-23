/*********************************************************************
 * Software License Agreement (BSD License)
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

#ifndef _EXHAUSTIVE_FIT_DETECTOR_
#define _EXHAUSTIVE_FIT_DETECTOR_

#include <boost/filesystem.hpp>

#include <boost/ptr_container/ptr_vector.hpp>

#include <sensor_msgs/PointCloud.h>

#include "tabletop_object_detector/model_fitter.h"
#include "tabletop_object_detector/mesh_loader.h"

//model database
#include "model_database/model_database.h"
#include "model_database/database_scaled_model.h"

namespace billiard_object_detector {

//! Given a point cloud, computes the fit against multiple meshes and chooses the best ones
/*! Is templated on the type of individual fitter to use; the individual fitter must
  be able to store a mesh or point cloud inside of it, then later compute the fit to
  an input cloud. The individual fitter is designed to inherit from ModelToCloudFitter,
  defined inside of model_fitter.h
  
  This class just initializes a whole bunch of individual fitters, then, when given a 
  new cloud, tries to fit all of them. It will then return the fits with the best scores.  
*/
template <class Fitter>
class ExhaustiveFitDetector
{
 private:
  //! Stores the individual model to cloud fitters, each initialized with a model
  std::vector<Fitter*> templates;
  //! Loads one template from a file and, is loading succeeds, adds it to the back of templates
  bool loadTemplate(const std::string& filename);

 public:
  //! Just a stub; does not load models
  ExhaustiveFitDetector() {}
  //! Deletes any loaded models
  ~ExhaustiveFitDetector();
  //! Loads all the mesh files in the directory pointed to by \a path
  void loadTemplateModels(const std::string& path);
  //! Loads all the models that are in the model database
  void loadDatabaseModels(const model_database::ModelDatabase &database, bool reduced_set);
  //! Main fitting function; fits all meshes against \a cloud and sorts the fits
  /*! Fits the point cloud \a cloud against all the models in the internal list.
    It always stores the list with at most \a numResults best fits, sorted by 
    their score. At the end, it returns this list.
    \param rotate true if we search for the optimal rotation as well
  */
  template <class PointCloudType>
  std::vector<ModelFitInfo> fitBestModels(const PointCloudType& cloud, int numResults)
  {
    std::vector<ModelFitInfo> fit_results;
    if (numResults <= 0) return fit_results;
    
    for (size_t i=0; i<templates.size(); ++i) 
    {
      ModelFitInfo current = templates[i]->template fitPointCloud<PointCloudType>(cloud);
      if ((int)fit_results.size() < numResults) 
      {
	fit_results.push_back(current);
	std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
      } 
      else
      {
	if (fit_results.back().getScore() > current.getScore())
	{
	  fit_results.back() = current;
	  std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
	}
      }
    } 
    return fit_results;
  }
};

template <class Fitter>
ExhaustiveFitDetector<Fitter>::~ExhaustiveFitDetector()
{
  for (size_t i=0;i<templates.size();++i) {
    delete templates[i];
  }
}

template <class Fitter>
bool ExhaustiveFitDetector<Fitter>::loadTemplate(const std::string& filename)
{
  ROS_DEBUG("Loading mesh %s", filename.c_str());
  PLYModelLoader model_loader;
  geometric_shapes_msgs::Shape mesh;  
  model_loader.readFromFile(filename,mesh);
  if (mesh.vertices.empty()) 
  {
    ROS_ERROR("Failed to load mesh from file %s", filename.c_str());
    return false;
  }

  Fitter* fitter = new Fitter();
  fitter->initializeFromMesh(mesh);
  templates.push_back(fitter);  
  return true;
}

/*! Loads all the files with extension .ply in the directory \a path. */
template <class Fitter>
void ExhaustiveFitDetector<Fitter>::loadTemplateModels(const std::string& path)
{
  boost::filesystem::path template_dir(path);
  
  if (!boost::filesystem::is_directory(template_dir)) {
    ROS_ERROR("Cannot load templates, %s is not a directory", template_dir.leaf().c_str());
    return;
  }
  
  boost::filesystem::directory_iterator dir_iter(template_dir), dir_end;
  for(;dir_iter != dir_end; ++dir_iter) {
    if (boost::filesystem::extension(*dir_iter)==".ply") {
      if (loadTemplate(dir_iter->string())) {
        ROS_INFO("Loaded file %s",dir_iter->string().c_str());
      }
    }
  }  
}

/*! Loads all the models in the Model Database. In order to do that, it asks the
  database for a list of models, then asks for the path to the geometry file for
  each model. Then it initializes a IterativeDistanceFitter for each of them, and also sets
  the database model id correctly for each model so that we later know what model
  each instance of IterativeDistanceFitter refers to.

  WARNING: for the moment, it only uses the database models with the "orthographic"
  acquisition method. Those models are rotationally symmetric (which is what most fitters
  operating under this class are capable of handling) plus they do not have "filled insides"
  which makes them easier to grasp.
*/
template <class Fitter>
void ExhaustiveFitDetector<Fitter>::loadDatabaseModels(const model_database::ModelDatabase &database, 
						       bool reduced_set)
{
  boost::ptr_vector<model_database::DatabaseScaledModel> model_ids;
  bool result;
  if (!reduced_set)
  {
    //change here to get other (or all) models
    //result = database.getScaledModelsByAcquisition(model_ids, "orthographic");
    result = database.getScaledModelsIcraExperimentSet(model_ids);
  }
  else
  {
    result = database.getScaledModelsReducedExperimentSet(model_ids);
  }
  if (!result)
  {
    ROS_ERROR("Database error: could not retrieve list of models");
    return;
  }
  if (model_ids.empty()) 
  {
    ROS_ERROR("Empty model list retrieved from database");
    return;
  }

  //use the model_root as specified in database
  std::string model_root;
  if (!database.getModelRoot(model_root)) 
  {
    ROS_ERROR("Database error: could not retrieve model root");
    return;
  }
  //or set it here to point to the correct directory on your local machine
  //model_root = "/home/matei/data/model_database/";

  ROS_INFO("Object detector: loading object models");
  for(size_t i=0; i<model_ids.size(); i++) 
  {
    std::string geometry_path = model_ids[i].geometry_path_.get();
    geometry_path = model_root + geometry_path;
    if (loadTemplate(geometry_path)) 
    {
      //set the model ID in the template so that we can use it later
      templates.back()->setModelId( model_ids[i].id_.get() );
      ROS_INFO("  Loaded database model with id %d", model_ids[i].id_.get());
    }
  }
  ROS_INFO("Object detector: loading complete");
}

} //namespace

#endif
