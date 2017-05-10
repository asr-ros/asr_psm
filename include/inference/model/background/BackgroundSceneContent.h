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
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/SceneContent.h"

#include "inference/model/background/PowerSetBackgroundInferenceAlgorithm.h"
#include "inference/model/background/SummarizedBackgroundInferenceAlgorithm.h"
#include "inference/model/background/MultipliedBackgroundInferenceAlgorithm.h"
#include "inference/model/background/MaximumBackgroundInferenceAlgorithm.h"

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This subclass of the SceneContent class is a wrapper for storing the resources required by a background scene. The background scene is based onthe same calculations as the foreground scene, but the information here is assumed equal distributed.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class BackgroundSceneContent : public SceneContent {
  public:
    
    /**
     * Constructor.
     */
    BackgroundSceneContent();
    
    /**
     * Destructor.
     */
    ~BackgroundSceneContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Initializes the inference algorithms. The algorithm that should be used is determined by the given string.
     * 
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    void initializeInferenceAlgorithms(std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior);
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void update(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger);
    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);
  };
}
