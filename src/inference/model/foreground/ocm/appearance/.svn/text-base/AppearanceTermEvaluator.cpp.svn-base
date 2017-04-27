#include "inference/model/foreground/ocm/appearance/AppearanceTermEvaluator.h"

namespace ProbabilisticSceneRecognition {
  
  AppearanceTermEvaluator::AppearanceTermEvaluator()
  : TermEvaluator()
  {
    // Initialize pointer.
    mMappedTable.reset(new MappedProbabilityTable());
  }
  
  AppearanceTermEvaluator::~AppearanceTermEvaluator()
  {
  }
  
  void AppearanceTermEvaluator::load(boost::property_tree::ptree& pPt)
  {
    // Load mapped probability table.
    mMappedTable->load(pPt.get_child("appearance"));
  }
  
  void AppearanceTermEvaluator::handleSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    // Scene graph is no of interest here.
  }
  
  void AppearanceTermEvaluator::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Here is no visualization required.
  }
  
  double AppearanceTermEvaluator::calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments)
  {
    /**************************************************************************************************
     * Build an entry of the conditional joint distribution P(A|h) bases on the given hypothesis
     **************************************************************************************************/  
    
    // The number of filled slots.
    unsigned int numberOfFilledSlots = 0;
    
    double probability = 1.0;
    
    // If no part was assigned to the root node, we don't need to continue (because the model doesn't allow cases like this).
    // In this case the probability stays zero, because that case it IMPOSSIBLE!
    if(pAssignments[0] > 0)
    {
      // Evaluate all slots with associated objects. Multiply their probability to the hypothesis probability.
      for(unsigned int slot = 0; slot < mMappedTable->getNumberOfRows(); slot++)
      {
	// Get the part assigned to the current slot.
	unsigned int part = pAssignments[slot];
	
	// Ignore assignments of the zero-object, because later we anyway would have to marginalize it out.
	// If a part with a larger index than zero is assigned, we associate the part with the probability table for the slot.
	if(part > 0)
	{
	  // Get the value from the probability table for the given slot and multiply it to the hypothesis probability.
	  // Subtract one from the part number, because the first element in the evidence vector is adressed by zero.
	  probability *= mMappedTable->getProbability(slot, pEvidenceList[part - 1].type);
	  
	  // Used to calculate the number of filled slots.
	  numberOfFilledSlots++;
	}
      }
    } else {
      // A hypothesis without an assigned root object is invalid and will therefore be scored with the impossible event.
      probability = 0.0;
    }
    
    // Return the probability for the given hypothesis.
    return probability;
  }
  
  void AppearanceTermEvaluator::visualize(std::vector<pbd_msgs::PbdObject> pEvidenceList)
  {
    // No visualization to update.
  }
  
  unsigned int AppearanceTermEvaluator::getNumberOfSlots()
  {
    return mMappedTable->getNumberOfRows();
  }
  
}