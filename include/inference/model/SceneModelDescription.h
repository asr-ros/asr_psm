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
#include <ros/ros.h>
#include <ros/console.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <asr_msgs/AsrObject.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "inference/model/ObjectEvidence.h"
#include "inference/model/SceneIdentifier.h"
#include "inference/model/SceneDescription.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Model for probabilistic scene recognition.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneModelDescription {    
  public:

    /**
     * Constructor.
     */
    SceneModelDescription();
    
    /**
     * Destructor.
     */
    ~SceneModelDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPathToFile Path to the XML file that contains the modelin serialized form.
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    void loadModelFromFile(std::string pPathToFile, std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Integrated evidence abound objects found by the detection systems into the model.
     * Evidences are accumulated until an update is requested.
     * 
     * @param pObject AsrObject message containing data about the evidence.
     */
    void integrateEvidence(const boost::shared_ptr<const asr_msgs::AsrObject>& pObject);
    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void integrateSceneGraph(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);
    
    /**
     * Update the model based on the accumulated evidence.
     */
    void updateModel();
    
    /**
     * Returns a list of all scenes containing their name and probability.
     * 
     * @param pSceneList The scene list including names and probabilities of all scenes.
     */
    void getSceneListWithProbabilities(std::vector<SceneIdentifier>& pSceneList);
    
  private:
    
    /**
     * An intelligent container for the object evidences. It states whether a given evidence is an update of an already known object or a new one.
     */
    ObjectEvidence mObjectEvidence;
    
    /**
     * Used for forwarding the evidences.
     * Put this here so we don't need to build a new one every time we got new evidence.
     */
    std::vector<asr_msgs::AsrObject> mEvidenceList;
    
    /**
     * A list containing the background and foreground elements.
     */
    std::vector<boost::shared_ptr<SceneDescription> > mScenes;
  };
}
