/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {
 
  GaussianKernel::GaussianKernel()
  {
  }
  
  GaussianKernel::~GaussianKernel()
  {
  }
  
  void GaussianKernel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer)
  {
    mVisualizer->appendKernel(boost::shared_ptr<Eigen::Vector3d>(new Eigen::Vector3d(mMean->block<3,1>(0,0))),
			      boost::shared_ptr<Eigen::Matrix3d>(new Eigen::Matrix3d(mCovariance->block<3,3>(0,0))));
  }
  
  bool GaussianKernel::compare(const GaussianKernel pKernel)
  {
    bool result = true;
    
    // Compare size of mean vectors.
    result &= (mMean->size() == pKernel.mMean->size());
    
    // Compare the mean vectors.
    for(int i = 0; i < mMean->size(); i++)
      result &= ((*mMean)(i) == (*pKernel.mMean)(i));
    
    // Compare size of covariance matrices..
    result &= (mCovariance->rows() == pKernel.mCovariance->rows());
    result &= (mCovariance->cols() == pKernel.mCovariance->cols());
    
    // Compare the covariance matrices.
    for(int i = 0; i < mCovariance->rows(); i++)
      for(int j = 0; j < mCovariance->cols(); j++)
	result &= ((*mCovariance)(i,j) == (*pKernel.mCovariance)(i,j));
      
    // If both kernels are equal, this should be still true.
    return result;
  }
  
  void GaussianKernel::save(boost::property_tree::ptree& pPt)
  { 
    // Create a seperate tree for this kernel.
    boost::property_tree::ptree subtree;
    
    // Write weight to XML.
    subtree.add("<xmlattr>.weight", mWeight);
    
    // Serialize mean vector and write it to XML.
    std::string meanString;
    SerializationHelper::convertVectorToString(mMean, meanString);
    subtree.add("<xmlattr>.mean", meanString);
    
    // Serialize covariance matrix and write it to XML.
    std::string covarianceString;
    SerializationHelper::convertMatrixToString(mCovariance, covarianceString);
    subtree.add("<xmlattr>.covariance", covarianceString);
        
    // Add subtree to main tree.
    pPt.add_child("kernel", subtree);
  }
  
}
