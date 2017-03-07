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

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract subclass of the also abstract InferenceAlgorithm class. It provides the calculations for a background scene object. This is basically a foreground scene object which is evaluated under the assumption of equal distribution.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class BackgroundInferenceAlgorithm : public InferenceAlgorithm {
  public:

    /**
     * Constructor.
     */
    BackgroundInferenceAlgorithm();
    
    /**
     * Destructor.
     */
    ~BackgroundInferenceAlgorithm();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Executes the inference based on the given evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    virtual void doInference(std::vector<asr_msgs::AsrObject> pEvidenceList, std::ofstream& pRuntimeLogger) = 0;
    
    /**
     * Returns the probability calculated by the inference process.
     * 
     * @return Probability for this scene.
     */
    virtual double getProbability() = 0;
    
  protected:
    
    /**
     * Calculates the probability of a single background scene object.
     * There are multiple background scene objects, but they all have the same probability.
     * 
     * @param pNumberOfEvidence The number of objects to consider for the background calculation.
     * @param pNumberOfSlots The total number of slots.
     * @return The probability of a single background scene object.
     */
    double calculateProbabilityOfBackgroundSceneObject(unsigned int pNumberOfEvidence, unsigned int pNumberOfSlots);
    
  private:
    
    /**
     * The number of all possible object instances in the/our world.
     */
    unsigned int mNumberOfObjectClasses;
    
    /**
     * The volume of the space we're operating in.
     */
    double mVolumeOfWorkspace;
    
  };
}
