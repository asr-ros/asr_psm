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
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrSceneGraph.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract class to encapsulate the creation and maintenance of a joint probability distribution. The distributions are loaded from XML file. Object evidence will also be handles by this class, especially the case when new evidence was found and the model needs rebuilding.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class TermEvaluator {
  public:

    /**
     * Constructor.
     */
    TermEvaluator();
    
    /**
     * Destructor.
     */
    virtual ~TermEvaluator();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    virtual void handleSceneGraph(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph) = 0;
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior) = 0;
    
    /**
     * Calculates the probability for a hypothesis with the given assignments.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pAssignments Assignments of parts to slots.
     * @return Probability as determinded by the term wrapped here.
     */
    virtual double calculateProbabilityForHypothesis(std::vector<asr_msgs::AsrObject> pEvidenceList, std::vector<unsigned int> pAssignments) = 0;
    
    /**
     * Update the visualizers based on the evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    virtual void visualize(std::vector<asr_msgs::AsrObject> pEvidenceList) = 0;
    
    /**
     * Returns the number of slots of the OCM (equals the number of distributions).
     * 
     * @return The number of slots of the OCM.
     */
    virtual unsigned int getNumberOfSlots() = 0;
  };
}
