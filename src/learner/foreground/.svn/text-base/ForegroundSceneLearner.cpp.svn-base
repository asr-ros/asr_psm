#include "learner/foreground/ForegroundSceneLearner.h"

namespace ProbabilisticSceneRecognition {
  
  ForegroundSceneLearner::ForegroundSceneLearner(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample)
  : SceneLearner(pExample->identifier)
  {
    addExampleToScene(pExample);
  }
  
  ForegroundSceneLearner::~ForegroundSceneLearner()
  {
  }
  
  void ForegroundSceneLearner::setClusteringParameters(double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation)
  {
    mStaticBreakRatio = pStaticBreakRatio;
    mTogetherRatio = pTogetherRatio;
    mMaxAngleDeviation = pMaxAngleDeviation;
  }
  
}