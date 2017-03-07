#include "learner/foreground/ocm/SceneObjectLearner.h"

namespace ProbabilisticSceneRecognition {

  SceneObjectLearner::SceneObjectLearner(std::string pSceneObjectType)
  : mSceneObjectType(pSceneObjectType)
  {
  }
  
  SceneObjectLearner::~SceneObjectLearner()
  {
  }
  
  bool SceneObjectLearner::hasType(std::string pSceneObjectType)
  {
    return mSceneObjectType.compare(pSceneObjectType) == 0;
  }
  
  void SceneObjectLearner::setClusteringParameters(double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation)
  {
    mStaticBreakRatio = pStaticBreakRatio;
    mTogetherRatio = pTogetherRatio;
    mMaxAngleDeviation = pMaxAngleDeviation;
  }
  
  void SceneObjectLearner::setVolumeOfWorkspace(double pValue)
  { 
    mWorkspaceVolume = pValue;
  }
  
  void SceneObjectLearner::setPriori(double pPriori)
  {
    mPriori = pPriori;
  }
  
}