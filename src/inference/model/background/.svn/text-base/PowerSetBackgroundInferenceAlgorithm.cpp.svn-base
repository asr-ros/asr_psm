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
  
  void PowerSetBackgroundInferenceAlgorithm::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
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