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