/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/shape/GaussianMixtureDistribution.h"

namespace ProbabilisticSceneRecognition {
  
  GaussianMixtureDistribution::GaussianMixtureDistribution(unsigned int pDimension)
  : mDimension(pDimension)
  {
  }
  
  GaussianMixtureDistribution::~GaussianMixtureDistribution()
  {
  }
  
  void GaussianMixtureDistribution::load(boost::property_tree::ptree& pPt, std::string pNode)
  {
    // Load the gaussian kernels.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pPt.get_child(pNode))
    {
      // Only access the 'distribution' child nodes.
      if(!std::strcmp(v.first.c_str(), "kernel"))
    mKernels.push_back(PSMInference::GaussianKernel(mDimension, v.second));
    }
    
    // Sum up all mixture weights.
    double sum = 0.0;
    BOOST_FOREACH(PSMInference::GaussianKernel kernel, mKernels)
      sum += kernel.getWeight();
    
    // Check, if the mixture weights summed up before are one.
    if(sum < 0.999 || sum > 1.001)
      throw std::invalid_argument("Unable to procees loading. The weights of the mixture of gaussian do not sum up to one (" + boost::lexical_cast<std::string>(sum) + ").");
  }
  
  void GaussianMixtureDistribution::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> pVisualizer)
  {
    // Iterate over all gaussian kernels and add them to the visualizer.
    for(PSMInference::GaussianKernel kernel : mKernels)
      kernel.initializeVisualizer(pVisualizer);
  }
  
  double GaussianMixtureDistribution::evaluate(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    double result = 0.0;
    
    // Copy evidence into an 7D eigen vector.
    Eigen::VectorXd evidence = getVectorFromObject(pPose);
    
    // Marginalize over all kernels to get the probability.
    // Normalization is already done by the mixture weights in the kernel.
    BOOST_FOREACH(PSMInference::GaussianKernel kernel, mKernels)
      result += kernel.evaluate(evidence);
      
    // The result is a marginalisation over all gaussian kernels. 
    return result;
  }
  
  void GaussianMixtureDistribution::visualize(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer,
					      boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    // Reset the certainty to zero, so that we can use the value from the best fitting kernel.
    mVisualizer->resetCertainty();
    
    // Iterate over all kernels and take the best score.
    for(PSMInference::GaussianKernel kernel : mKernels)
      kernel.visualize(mVisualizer, getVectorFromObject(pPose));
  }
  
  Eigen::VectorXd GaussianMixtureDistribution::getVectorFromObject(boost::shared_ptr<ResourcesForPsm::Pose> pPose)
  {
    Eigen::VectorXd evidence(mDimension);
    
    if(mDimension == 3)
    {
      Eigen::Vector3d position = pPose->getPosition();
      evidence[0] = position[0];
      evidence[1] = position[1];
      evidence[2] = position[2];
    } else if (mDimension == 4) {
      Eigen::Quaternion<double> orientation = pPose->getOrientation();
      evidence[0] = orientation.w();
      evidence[1] = orientation.x();
      evidence[2] = orientation.y();
      evidence[3] = orientation.z();
    } else {
      throw std::invalid_argument("Unable to evaluate gaussian mixture distribution. It has an invalid number of dimensions (" + boost::lexical_cast<std::string>(mDimension) + ").");
    }
    return evidence;
  }
  
}
