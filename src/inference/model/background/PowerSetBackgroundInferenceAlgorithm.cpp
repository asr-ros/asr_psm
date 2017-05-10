/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/background/PowerSetBackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  PowerSetBackgroundInferenceAlgorithm::PowerSetBackgroundInferenceAlgorithm()
  : BackgroundInferenceAlgorithm()
  , mProbability(0.0)
  {
  }
  
  PowerSetBackgroundInferenceAlgorithm::~PowerSetBackgroundInferenceAlgorithm()
  {
  }

  void PowerSetBackgroundInferenceAlgorithm::doInference(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    /**************************************************************************************
    * What we do here:
    * 
    * This are basically the same calculations like in the foreground scene. We calculate
    * the probability of the scene based on the scene objects, exactly like in the
    * foreground scene.
    * 
    * However, the probability of the scene objects are calculated under an
    * equal distribution. We assume that there's a scene object for every detected object.
    * 
    * The objects are combined in a power set approach. First the power set of all object
    * combinations is calculated. For every combination the scene objects are evaluated.
    * The results of each combination (= subset in power set) are summarized to form
    * the scene probability.
    ***************************************************************************************/
    
    // Reset the scene probability. We MARGINALZE here, so we set it to ZERO.
    mProbability = 0.0;
    
    // The number of all hypotheses.
    unsigned int numberOfHypotheses = 0;
    
    // The number of scene objects equals the number of evidences.
    unsigned int numberOfSceneObjects = pEvidenceList.size();
    
    // We iterate over the power set over the scene objects.
    for(unsigned int i = 1; i < (1 << numberOfSceneObjects); ++i)
    {
      // String containing the subset for debugging.
      std::string subset;
      
      // The score for this subset. We MULTIPLY the scene object scores, to we initialize it with ONE.
      double hypothesisProbability = 1.0;
      
      // The number of elements in this subset.
      unsigned int numberOfElementsInSubset = 0;
      
      // Here we determine the number of elements in the subset of the power set.
      for(unsigned int j = 0; j < numberOfSceneObjects; ++j)
      {
	// Is element in subset?
	if((1 << j) & i)
	  numberOfElementsInSubset++;
      }
      
      // Here we determine and evaluate the subset of the power set.
      for(unsigned int j = 0; j < numberOfSceneObjects; ++j)
      {
	// Is element in subset?
	if((1 << j) & i)
	{
	  // Add the current subset number to the debug message string.
	  subset += "(" + boost::lexical_cast<std::string>(j) + ") ";
	  
	  // Update score based on the score of an equal distributed background object and an appropriate prior.
	  // The prior is a binary uniform distribution over the cases 'object there' and 'object not there'.
	  hypothesisProbability *= calculateProbabilityOfBackgroundSceneObject(numberOfElementsInSubset, numberOfElementsInSubset) * 0.5;
	  
	  ROS_INFO_STREAM(" > Multiplying with subset score: '" << calculateProbabilityOfBackgroundSceneObject(numberOfElementsInSubset, numberOfElementsInSubset) << "' with priori '0.5'.");
	}
      }
      
      // Debug message.
      ROS_INFO_STREAM("Score for scene object subset '" << subset << "' is '" << hypothesisProbability << "'.");
      
      // Add hypothesis to scene probability. We score the empty subset with zero.
      if(i > 0)
	mProbability += hypothesisProbability;
      
      // +1 for the number of hypotheses.
      numberOfHypotheses++;
    }
    
    // Apply normalization term by dividing by the number of all hypotheses.
    if(numberOfHypotheses > 0)
      mProbability /= numberOfHypotheses;
  }
  
  double PowerSetBackgroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}
