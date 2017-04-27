#include "inference/model/background/MaximumBackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  MaximumBackgroundInferenceAlgorithm::MaximumBackgroundInferenceAlgorithm()
  : BackgroundInferenceAlgorithm()
  , mProbability(0.0)
  {
  }
  
  MaximumBackgroundInferenceAlgorithm::~MaximumBackgroundInferenceAlgorithm()
  {
  }
  
  void MaximumBackgroundInferenceAlgorithm::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
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
    mProbability = calculateProbabilityOfBackgroundSceneObject(numberOfSceneObjects, numberOfSceneObjects);
  }
  
  double MaximumBackgroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}