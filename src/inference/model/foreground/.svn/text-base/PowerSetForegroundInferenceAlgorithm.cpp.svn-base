#include "inference/model/foreground/PowerSetForegroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  PowerSetForegroundInferenceAlgorithm::PowerSetForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects)
  : ForegroundInferenceAlgorithm(pSceneObjects)
  , mProbability(0.0)
  {
  }
  
  PowerSetForegroundInferenceAlgorithm::~PowerSetForegroundInferenceAlgorithm()
  {
  }
  
  void PowerSetForegroundInferenceAlgorithm::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Reset the scene probability. We MARGINALZE here, so we set it to ZERO.
    mProbability = 0.0;
    
    // The number of all hypotheses.
    unsigned int numberOfHypotheses = 0;
    
    ROS_INFO_STREAM("Evaluating primary scene objects.");
    
    // Update the scene objects with the new evidence.
    // This is done before iterating over the probabilities, because it has high costs and must be done only once.
    for(unsigned int i = 0; i < mSceneObjects->size(); i++)
      mSceneObjects->at(i)->update(pEvidenceList, pRuntimeLogger);
    
    ROS_INFO_STREAM("Iterating over existence hypothesis.");
    
    // We iterate over the power set over the scene objects.
    for(unsigned int i = 1; i < (1 << mSceneObjects->size()); ++i)
    {
      // String containing the subset for debugging.
      std::string subset;
      
      // The score for this subset. We MULTIPLY the scene object scores, to we initialize it with ONE.
      double hypothesisProbability = 1.0;
      
      // Here we determine and the subset of the power set and build the debug message string.
      for(unsigned int j = 0; j < mSceneObjects->size(); ++j)
      {
	// Is element in subset?
	if((1 << j) & i)
	{
	  // Add the current subset number to the debug message string.
	  subset += "(" + mSceneObjects->at(j)->getDescription() + ") ";
	}
      }
      
      // Here we determine and evaluate the subset of the power set.
      for(unsigned int j = 0; j < mSceneObjects->size(); ++j)
      {
	// Is element in subset?
	if((1 << j) & i)
	{
	  // Update score.
	  hypothesisProbability *= mSceneObjects->at(j)->getSceneObjectProbability() * mSceneObjects->at(j)->getSceneObjectPriori();
	  ROS_INFO_STREAM(" > Multiplying with subset score: '" << mSceneObjects->at(j)->getSceneObjectProbability() << "' with priori '" << mSceneObjects->at(j)->getSceneObjectPriori() << "'.");
	}
      }
      
      // Debug message.
      ROS_INFO_STREAM(" > Score for scene object subset '" << subset << "' is '" << hypothesisProbability << "'.");
      
      // Add hypothesis to scene probability. We score the empty subset with zero.
      if(i > 0)
	mProbability += hypothesisProbability;
      
      // +1 for the number of hypotheses.
      numberOfHypotheses++;
    }
    
    // Apply normalization term by dividing by the number of all hypotheses.
    mProbability /= numberOfHypotheses;
  }
  
  double PowerSetForegroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}