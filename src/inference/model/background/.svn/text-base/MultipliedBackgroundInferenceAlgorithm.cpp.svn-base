#include "inference/model/background/MultipliedBackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  MultipliedBackgroundInferenceAlgorithm::MultipliedBackgroundInferenceAlgorithm()
  : BackgroundInferenceAlgorithm()
  , mProbability(0.0)
  {
  }
  
  MultipliedBackgroundInferenceAlgorithm::~MultipliedBackgroundInferenceAlgorithm()
  {
  }
  
  void MultipliedBackgroundInferenceAlgorithm::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
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
    * The scene objects are evaluated and the results are multiplied to form
    * the scene probabilty.
    ***************************************************************************************/
    
    // The number of scene objects equals the number of evidences.
    unsigned int numberOfSceneObjects = pEvidenceList.size();
    
    // The probability is the sum of the scores for every background scene object.
    mProbability = pow(calculateProbabilityOfBackgroundSceneObject(numberOfSceneObjects, numberOfSceneObjects), numberOfSceneObjects);
  }
  
  double MultipliedBackgroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}