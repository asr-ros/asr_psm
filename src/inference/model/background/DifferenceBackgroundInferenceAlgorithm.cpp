/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/background/DifferenceBackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  DifferenceBackgroundInferenceAlgorithm::DifferenceBackgroundInferenceAlgorithm()
  : BackgroundInferenceAlgorithm()
  , mProbability(0.0)
  {
  }
  
  DifferenceBackgroundInferenceAlgorithm::~DifferenceBackgroundInferenceAlgorithm()
  {
  }

  void DifferenceBackgroundInferenceAlgorithm::doInference(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)
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
    * The scene objects are evaluated and the results are summarized to form
    * the scene probabilty.
    ***************************************************************************************/

    // The number of scene objects equals the number of evidences.
    unsigned int numberOfSceneObjects = pEvidenceList.size();

    // No need to find the maximum, all scores are the same.
    mProbability = 1.0;
    for(int i = 0; i < numberOfSceneObjects; i++){
        mProbability *= 0.1;                                                            //TODO: Parameter überdenken?
    }
    //mProbability = calculateProbabilityOfBackgroundSceneObject(numberOfSceneObjects, numberOfSceneObjects);
  }
  
  double DifferenceBackgroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}
