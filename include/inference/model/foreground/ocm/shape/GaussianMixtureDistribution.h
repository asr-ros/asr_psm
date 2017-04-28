/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Package includes
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Pose.h>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {

  /**
   * A gaussian mixture distribution maintaining a list of weighted gaussian kernels.
   */
  class GaussianMixtureDistribution {
  public:
    
    /**
    * Constructor.
    * 
    * @param pDimension The number of dimensions of the distribution.
    */
    GaussianMixtureDistribution(unsigned int pDimension);
    
    /**
    * Destructor.
    */
    ~GaussianMixtureDistribution();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     * @param pNode The name of the node that contains the distribution.
     */
    void load(boost::property_tree::ptree& pPt, std::string pNode);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param pVisualizer The visualizer for the secondary scene object represented by this node.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> pVisualizer);
    
    /**
     * Evaluates the gaussian mixture distribution for the given evidence and visualizes the result.
     * 
     * @param pVisualizer The visualizer for the secondary scene object represented by this node.
     * @param pPose The pose to evaluate the gaussian mixture distribution with.
     */
    void visualize(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> pVisualizer,
		   boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Evaluate the gaussian mixture distribution with the given pose.
     * 
     * @param pPose The pose to evaluate the gaussian mixture distribution with.
     */
    double evaluate(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
  private:
    
    /**
     * Converts the given pose into a 7D eigen Vector.
     * 
     * @param pPose The pose to convert to a 7d vector.
     * @return The 7d vector representing the pose.
     */
    Eigen::VectorXd getVectorFromObject(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
  private:
    
    /**
     * The number of dimensions of the distribution.
     */
    unsigned int mDimension;
    
    /**
     * The gaussian kernels (and weights) forming the gaussian mixture distribution.
     */
    std::vector<GaussianKernel> mKernels;
  };
}
