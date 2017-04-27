/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

namespace ProbabilisticSceneRecognition {
 
  GaussianMixtureModel::GaussianMixtureModel()
  {
  }
  
  GaussianMixtureModel::~GaussianMixtureModel()
  {
  }
  
  void GaussianMixtureModel::addKernel(const GaussianKernel& pKernel)
  {
    // Check, if a kernel of this type already exists.
    for(GaussianKernel kernel : mKernels)
      if(kernel.compare(pKernel))
	return;
    
    // Kernel not already in distribution? Take it!
    mKernels.push_back(pKernel);
  }
  
  void GaussianMixtureModel::normalizeWeights()
  {
    double sum = 0.0;
    
    // Calculate the sum over all kernels.
    for(unsigned int i = 0; i < mKernels.size(); i++)
      sum += mKernels[i].mWeight;
    
    // Normalize the kernels.
    for(unsigned int i = 0; i < mKernels.size(); i++)
      mKernels[i].mWeight /= sum;
  }
  
  unsigned int GaussianMixtureModel::getNumberOfKernels()
  {
    return mKernels.size();
  }
  
  void GaussianMixtureModel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer)
  {
    for(GaussianKernel kernel : mKernels)
      kernel.initializeVisualizer(mVisualizer);
  }
  
  void GaussianMixtureModel::save(boost::property_tree::ptree& pPt, std::string pNode)
  {
    // Create a seperate tree for this gmm.
    boost::property_tree::ptree subtree;
    
    BOOST_FOREACH(GaussianKernel kernel, mKernels)
      kernel.save(subtree);
      
    // Add subtree to main tree.
    pPt.add_child(pNode, subtree);
  }
  
}
