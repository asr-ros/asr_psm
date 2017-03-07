#include "inference/model/background/BackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  BackgroundInferenceAlgorithm::BackgroundInferenceAlgorithm()
  : InferenceAlgorithm()
  {
  }
  
  BackgroundInferenceAlgorithm::~BackgroundInferenceAlgorithm()
  {
  }
  
  void BackgroundInferenceAlgorithm::load(boost::property_tree::ptree& pPt)
  {
    mNumberOfObjectClasses = pPt.get<int>("description.<xmlattr>.objects");
    mVolumeOfWorkspace = pPt.get<double>("description.<xmlattr>.volume");
  }
  
  double BackgroundInferenceAlgorithm::calculateProbabilityOfBackgroundSceneObject(unsigned int pNumberOfEvidence, unsigned int pNumberOfSlots)
  {
    // NOTE We assume here that the number of slots equals the number of evidences, so that all evidences have a place in the calculation.
    
    // The probability of the background scene object.
    double probability = 0.0;
    
    // The number of valid hypothesis, therefore hypotheses where the root node is not occluded and no object is assigned more than one times.
    unsigned int numberOfValidHypotheses = 0;
    
    // The number of evidence is the given number of evidence +1 for the null object.
    unsigned int numberOfEvidence = pNumberOfEvidence + 1;
    
    // Now we iterate over all combinations of hypotheses 'h'.
    for(unsigned int h = 0; h < pow(numberOfEvidence, pNumberOfSlots); h++)
    {
      // A marker that becomes true, when hypothesis contains double assignments and should be dropped.
      bool dropped = false;
      
      // A vector holding the assignments of parts to slots.
      std::vector<unsigned int> assignments;
      
      // The number of filled slots.
      unsigned int numberOfFilledSlots = 0;
      
      // Calculate the assignments of parts to slots. Drop hypotheses with double assignments of parts to slots.
      for(unsigned int s = 0; s < pNumberOfSlots; s++)
      {
	// Calculates which part should be associated with the 's'th slot, based on the 'h'th hypothesis.
	unsigned int part = (h / ((unsigned int) pow(numberOfEvidence, s)) ) % numberOfEvidence;
	
	// If part is not zero-object and is already assigned, we drop this hypothesis and continue with the next one.
	if(part > 0 && std::find(assignments.begin(), assignments.end(), part) != assignments.end())
	  dropped = true;
	
	// Root object is occluded? Drop hypothesis!
	if(s == 0 && part == 0)
	  dropped = true;
	
	// Assign the calculated part to the next slot in the row.
	// Even if we don't use the vector we have to do it so we can drop double assignments.
	assignments.push_back(part);
	
	// Used to calculate the number of filled slots.
	if(part > 0)
	  numberOfFilledSlots++;
      }
      
      // Calculate the background probability.
      if(!dropped)
      {
	// Calculate the variables requried by the poisson distribution of the object occlusion term.
	// The plus one is required to fix an anomaly in the distributions.
	double lambda = pNumberOfEvidence + 1;
      
	double faculty = 1;
	for(unsigned int i = 2; i <= numberOfFilledSlots; i++)
	  faculty *= i;
	
	// Calculate the probability for the background based on (in order of appearance):
	// - an equal distributed shape over all three position dimensions and four orientation dimensions for every slot
	// - an equal distributed appearance
	// - an equal distributed occlusion term
	// - hypothesis weight by poisson distribution
	probability += pow(1.0 / (mVolumeOfWorkspace * 2.0 * pow(M_PI, 2.0)), numberOfFilledSlots) 
		    * pow(1.0 / mNumberOfObjectClasses, numberOfFilledSlots)
		    * pow(0.5, numberOfFilledSlots)
		    * (pow(lambda, numberOfFilledSlots) / faculty) * exp(-lambda);
	
	// +1 for this valid hypothesis.
	numberOfValidHypotheses++;
      }
    }
    // Apply normalization term by dividing by the number of VALID (!) hypotheses.
    if(numberOfValidHypotheses > 0)
      probability /= numberOfValidHypotheses;
    
    // return the probability for the background scene object.
    return probability;
  }
  
}