/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/occlusion/OcclusionTermLearner.h"

namespace ProbabilisticSceneRecognition {
 
  OcclusionTermLearner::OcclusionTermLearner()
  : TermLearner()
  {    
  }
  
  OcclusionTermLearner::~OcclusionTermLearner()
  {
  }
  
  void OcclusionTermLearner::learn(boost::shared_ptr<OcmModel> pModel)
  {
    // Get the number of slots in the model.
    unsigned int numberOfSlots = pModel->getNumberOfSlots();
    
    // Create the occlusion table.
    pModel->mOcclusionTable.reset(new ProbabilityTable(numberOfSlots, 2));
    
    // The objects may be there or not. We can't extract this information from the data at this point
    // (no tracking available), so we set it to always there.
    for(unsigned int i = 0; i < numberOfSlots; i++)
    {
      //pModel->mOcclusionTable->add(i, 0, 1);		// Nicht da
      pModel->mOcclusionTable->add(i, 1, 1);		// Da
    }
    
    // Normalize the probability table. Not necessary here, but just to be sure in case of changes...
    pModel->mOcclusionTable->normalize();
  }
  
}
