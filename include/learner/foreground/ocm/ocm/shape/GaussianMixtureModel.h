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

// Global includes
#include <vector>

// Package includes
#include <pl.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "learner/foreground/ocm/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {
 
  /**
   * This class is a wrapper for a gaussian mixture model (GMM).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class GaussianMixtureModel {
  public:

    /**
     * Constructor.
     */
    GaussianMixtureModel();
    
    /**
     * Destructor.
     */
    ~GaussianMixtureModel();
    
    /**
     * Adds a kernel to the distribution.
     * 
     * @param pKernel The kernel to add.
     */
    void addKernel(const GaussianKernel& pKernel);
    
    /**
     * Normalizes the weights to sum up to one. This is necessary after removing duplicate kernels.
     */
    void normalizeWeights();
    
    /**
     * Returns the number of kernels.
     * 
     * @return The number of kernels.
     */
    unsigned int getNumberOfKernels();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mSuperior);
    
    /**
     * Saves the content to XML.
     * 
     * @param pPt Datastructure for handling XML operations.
     * @param pNode The name of the XML node that should contain the distribution.
     */
    void save(boost::property_tree::ptree& pPt, std::string pNode);

  private:
    
    /**
     * The gaussian kernels the GMM is made of.
     */
    std::vector<GaussianKernel> mKernels;
  };
}
