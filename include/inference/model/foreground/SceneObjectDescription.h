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
#include <chrono>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>
#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

#include <fstream>

// Local includes
#include "inference/model/foreground/SceneObjectContent.h"

#include "inference/model/foreground/ocm/OcmSceneObjectContent.h"

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class models a single instance of a scene object. A scene object contains information about the object as well as the scene. One could describe it as an object in the context of a scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneObjectDescription {    
  public:

    /**
     * Constructor.
     */
    SceneObjectDescription();

    /**
     * Destructor.
     */
    ~SceneObjectDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
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
    void update(std::vector<asr_msgs::AsrObject> pEvidenceList, std::ofstream& pRuntimeLogger);

    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);
    
    /**
     * Returns the probability for the scene object modelled by this class.
     * 
     * @return Probability for this scene object.
     */
    double getSceneObjectProbability();
    
    /**
     * Returns the a priori probability of the scene object.
     * 
     * @return The a priori probability of the scene object.
     */
    double getSceneObjectPriori();
    
    /**
     * Returns the description of the scene object.
     * 
     * @return The description of the scene object.
     */
    std::string getDescription();
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    void setBestStatus(bool pStatus);
    
  private:
    
    /**
     * A priori probability of the scene object.
     */
    double mPriori;
    
    /**
     * The type of the content wrapped in this class.
     */
    std::string mType;
    
    /**
     * A short description of the scene object (e.g. Cup, Blue Plate, ...).
     */
    std::string mDescription;
    
    /**
     * A wrapper for the model that states how the scene object is modelled.
     */
    boost::shared_ptr<SceneObjectContent> mContent;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mVisualizer;
  };
}
