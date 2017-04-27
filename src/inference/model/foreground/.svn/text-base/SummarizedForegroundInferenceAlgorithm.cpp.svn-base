#include "inference/model/foreground/SummarizedForegroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  SummarizedForegroundInferenceAlgorithm::SummarizedForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects)
  : ForegroundInferenceAlgorithm(pSceneObjects)
  , mProbability(0.0)
  {
  }
  
  SummarizedForegroundInferenceAlgorithm::~SummarizedForegroundInferenceAlgorithm()
  {
  }
  
  void SummarizedForegroundInferenceAlgorithm::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Reset the scene probability. We MARGINALZE here, so we set it to ZERO.
    mProbability = 0.0;

    ROS_INFO_STREAM("Calculating scene probability.");
    
    // Iterate over all scene objects, evaluate them and summarize the results
    for(boost::shared_ptr<SceneObjectDescription> sceneObject : *mSceneObjects)
    {
      // Update the scene objects with the new evidence.
      sceneObject->update(pEvidenceList, pRuntimeLogger);
      
      // Add the score of the scene object to the scene probabilty.
      mProbability += sceneObject->getSceneObjectProbability() * sceneObject->getSceneObjectPriori();
      
      ROS_INFO_STREAM(" > Adding to scene probability: '" << sceneObject->getSceneObjectProbability() << "' with priori '" << sceneObject->getSceneObjectPriori() << "'.");
      
      // TODO Think about a normalization term.
    }
  }
  
  double SummarizedForegroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}