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
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

// Package includes
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/InferenceAlgorithm.h"

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract class for modelling a specific type of scene (e.g. fore- or background scene).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneContent {
  public:

    /**
     * Constructor.
     */
    SceneContent();
    
    /**
     * Destructor.
     */
    virtual ~SceneContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Initializes the inference algorithms. The algorithm that should be used is determined by the given string.
     * 
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    virtual void initializeInferenceAlgorithms(std::string pAlgorithm) = 0;
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior) = 0;
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    virtual void update(std::vector<asr_msgs::AsrObject> pEvidenceList, std::ofstream& pRuntimeLogger) = 0;
    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    virtual void update(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph) = 0;
    
    /**
     * Returns the probability for the scene modelled by this class.
     * 
     * @return Probability for this scene.
     */
    double getSceneProbability();
    
  protected:
    
    /**
     * Sets the inference algorithm.
     * 
     * @param pAlgorithm The algorithm used to infer the scene probability.
     */
    void setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm> pAlgorithm);
    
    /**
     * Loads the data used by the inference algorithm from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void loadInferenceAlgorithm(boost::property_tree::ptree& pPt);
    
    /**
     * Executes the inference process.
     * 
     * @param pEvidenceList The evidence found.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void doInference(std::vector<asr_msgs::AsrObject> pEvidenceList, std::ofstream& pRuntimeLogger);
    
  private:
    
    /**
     * The algorithm used for inference.
     */
    boost::shared_ptr<InferenceAlgorithm> mAlgorithm;
    
  };
}
