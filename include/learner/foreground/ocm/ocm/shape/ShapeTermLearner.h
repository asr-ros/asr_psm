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
#include <boost/shared_ptr.hpp>

#include <Pose.h>

// Local includes
#include "learner/foreground/ocm/ocm/OcmTree.h"
#include "learner/foreground/ocm/ocm/OcmModel.h"
#include "learner/foreground/ocm/ocm/TermLearner.h"

#include "learner/foreground/ocm/ocm/shape/GMMParameterEstimator.h"

#include "helper/MathHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for the shape term of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ShapeTermLearner : public TermLearner {
  public:
    
    /**
     * Constructor.
     */
    ShapeTermLearner();
    
    /**
     * Destructor.
     */
    ~ShapeTermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
   void learn(boost::shared_ptr<OcmModel> pModel);
   
  private:
    
    /**
     * Learns the term parameters.
     * 
     * @param pNode OCM parent node that provides raw data and containers for the parameters to learn.
     */
   void learn(boost::shared_ptr<OcmTree> pNode);
    
    /**
     * Learns the shape term parameters for a parent/child pair.
     * 
     * @param pParent The parent node the pose should be learned relative to.
     * @param pChild The child node to assign the learned pose to.
     */
    void learnNodePose(boost::shared_ptr<OcmTree> pParent, boost::shared_ptr<OcmTree> pChild);
    
  private:
    
    /**
     * The minmal and maximal number of kernels.
     */
    int mNumberKernelsMin, mNumberKernelsMax;
    
    /**
     * The number of runs per kernel.
     */
    int mRunsPerKernel;
    
    /**
     * For every sample, n noised samples are generated to make orientation and position learning more tolerant.
     */
    int mNumberOfSyntheticSamples;
    
    /**
     * The intervals (position and orientation) for the sample relaxiation.
     */
    double mIntervalPosition, mIntervalOrientation;
    
    /**
     * Path to the orientation plots.
     */
    std::string mPathOrientationPlots;
    
    /**
     * Interface to the private namespace of the ros node.
     */
    ros::NodeHandle mPrivateNamespaceHandle;

    /**
     * The number of attempts per run to find a valid model until the learner gives up.
     */
    int mAttemptsPerRun;
  };
}
