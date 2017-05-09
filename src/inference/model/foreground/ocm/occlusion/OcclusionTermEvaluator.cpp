/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/occlusion/OcclusionTermEvaluator.h"

namespace ProbabilisticSceneRecognition {
 
  OcclusionTermEvaluator::OcclusionTermEvaluator()
  : TermEvaluator()
  {
    // Initialize pointer.
    mTable.reset(new ProbabilityTable());
  }
  
  OcclusionTermEvaluator::~OcclusionTermEvaluator()
  {
  }
  
  void OcclusionTermEvaluator::load(boost::property_tree::ptree& pPt)
  {
    // Load the probability table.
    mTable.reset(new ProbabilityTable(pPt.get_child("occlusion")));
  }
  
  /*void OcclusionTermEvaluator::handleSceneGraph(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph)
  {
    // Scene graph is no of interest here.
  }*/
  
  void OcclusionTermEvaluator::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Here is no visualization required.
  }

  double OcclusionTermEvaluator::calculateProbabilityForHypothesis(std::vector<asr_msgs::AsrObject> pEvidenceList, std::vector<unsigned int> pAssignments)
  {
    /**************************************************************************************************
     * Build an entry of the conditional joint distribution P(h) bases on the given hypothesis
     **************************************************************************************************/  
    
    // The occlusion probability based on the given hypothesis.
    // It will be calculated in this function.
    double result = 1.0;
   
    // If no part was assigned to the root node, we don't need to continue (because the model doesn't allow cases like this).
    if(pAssignments[0] > 0)
    {
      // The number of filled slots.
      unsigned int numberOfFilledSlots = 0;
      
      // Evaluate all slots with associated objects. Multiply their probability to the hypothesis probability.
      for(unsigned int slot = 0; slot < mTable->getNumberOfRows(); slot++)
      { 
	// Get the information whether or not a part is associated with the slot.
	unsigned int part = std::min(pAssignments[slot], (unsigned int) 1);
	
	// Get the probability for the given occlusion status of the given slot.
	result *= mTable->getProbability(slot, part);
	
	// Used to calculate the number of filled slots.
	if(pAssignments[slot] > 0)
	  numberOfFilledSlots++;
      }
    } else {
      // A hypothesis without an assigned root object is invalid and will therefore be scored with the impossible event.
      result = 0.0;
    }
    
    // Return the probability for the given hypothesis.
    return result;
  }

  void OcclusionTermEvaluator::visualize(std::vector<asr_msgs::AsrObject> pEvidenceList)
  {
    // No visualization to update.
  }
  
  unsigned int OcclusionTermEvaluator::getNumberOfSlots()
  {
    return mTable->getNumberOfRows();
  }
  
}
