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
#include <fstream>
#include <iostream>

// Package includes
#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrSceneGraph.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>
#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/SceneIdentifier.h"
#include "inference/model/SceneContent.h"

#include "inference/model/foreground/ForegroundSceneContent.h"	

#include "inference/model/background/BackgroundSceneContent.h"

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class describes a scene. It's a wrapper for the for the scene content which contains the concrete realization of the scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneDescription {
  public:
    
    /**
     * Constructor.
     */
    SceneDescription();
    
    /**
     * Destructor.
     */
    ~SceneDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     * @param pAlgorithm The name of the inference algorithm.
     */
    void load(boost::property_tree::ptree& pPt, std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void update(std::vector<asr_msgs::AsrObject> pEvidenceList);

    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);
    
    /**
     * Calculates the probability of the scene.
     */
    void calculateSceneProbaility();
    
    /**
     * Returns the scene's metadata.
     * 
     * @return The metadata of the scene.
     */
    boost::shared_ptr<SceneIdentifier> getSceneIdentifier();
        
    /**
     * Sets the scene content of this scene.
     * 
     * @param pIdentifier The identifier holding the scenes metadata.
     */
    void setSceneIdentifier(boost::shared_ptr<SceneIdentifier> pIdentifier);
    
  private:
    
    /**
     * handle for the file that will contain the results from the runtime test.
     */
    std::ofstream mRuntimeFile;
    
    /**
     * A wrapper for the scene's metadata.
     */
    boost::shared_ptr<SceneIdentifier> mIdentifier;
    
    /**
     * A container holding the resources required by the scene.
     * A background scene will hold other resources than a foreground scene.
     */
    boost::shared_ptr<SceneContent> mContent;
    
    /**
     * Coordinates the primary scene object visualization.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mVisualizer;
  };
}
