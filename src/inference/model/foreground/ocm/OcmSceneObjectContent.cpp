/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/OcmSceneObjectContent.h"

namespace ProbabilisticSceneRecognition {
 
  OcmSceneObjectContent::OcmSceneObjectContent()
  : SceneObjectContent()
  , mProbability(0.0)
  {
    mEvaluators.push_back(boost::shared_ptr<TermEvaluator>(new ShapeTermEvaluator()));
    mEvaluators.push_back(boost::shared_ptr<TermEvaluator>(new AppearanceTermEvaluator()));
    mEvaluators.push_back(boost::shared_ptr<TermEvaluator>(new OcclusionTermEvaluator));
  }
  
  OcmSceneObjectContent::~OcmSceneObjectContent()
  {
    
  }
  
  void OcmSceneObjectContent::load(boost::property_tree::ptree& pPt)
  {
    // Load number of slots from XMl file.
    mNumberOfSlots = pPt.get<unsigned int>("slots.<xmlattr>.number");
    
    // Check, if the number of slots is valid.
    if(mNumberOfSlots <= 0)
      throw std::invalid_argument("Unable to procees loading. OCM: number of slots must be > 0.");
    
    // Command the factories to load their data from the XML file.
    BOOST_FOREACH(boost::shared_ptr<TermEvaluator> evaluator, mEvaluators)
    {
      // Load term evaluator data.
      evaluator->load(pPt);
      
      // Check, if the model managed by the term evaluator has the right number of slots.
      if(mNumberOfSlots != evaluator->getNumberOfSlots())
	throw std::invalid_argument("Unable to procees loading. Divergent number of slots between the OCM distributions.");
    }
  }
  
  void OcmSceneObjectContent::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Store a pointer to the visualizer.
    mVisualizer = mSuperior;
    
    // Forward visualizer to the term evaluators.
    for(unsigned int i = 0; i < mEvaluators.size(); i++)
      mEvaluators[i]->initializeVisualizer(mSuperior);
  }

  void OcmSceneObjectContent::update(std::vector<asr_msgs::AsrObject> pEvidenceList)
  {
    // Generate object list for debug message
    std::stringstream objs;
    for(unsigned int i = 0; i < pEvidenceList.size(); i++)
    {
      objs << pEvidenceList[i].type;
      
      if(i < pEvidenceList.size() - 1)
	objs<< ", ";
    }
    
    // Show debug message with the found objects.
    ROS_INFO_STREAM(" > There are '" << pEvidenceList.size() << "' objects: " << objs.str() << ".");
    
    // Reset visualizer
    mVisualizer->resetHypothesis();
    
    /**************************************************************************************
    * Command the factories to calculate the probability for every hypothesis.
    * Marginalize over all hypotheses to calculate the probability for this scene object.
    ***************************************************************************************/
    
    // Reset probability. It is a marginalization and we add the probabilities
    // of the hypotheses to it, so this must be ZERO.
    mProbability = 0.0;
    
    // First of all we create all possible hypotheses (=assignments of parts to slots, parts are found objects).
    // Therefore we need their number, which is the size of the hypothesis space '|h|' to the power
    // of the number of slots 's': |h|^s.
    unsigned int numberOfEvidence = pEvidenceList.size() + 1;
    unsigned int numberOfHypotheses = pow(numberOfEvidence, mNumberOfSlots);
    unsigned int numberOfValidHypotheses = 0;
    
    // The sum of all filled slots for normalizing the result.
    unsigned int sumOfFilledSlots = 0;
    
    // Now we iterate over all combinations of hypotheses 'h'.
    for(unsigned int h = 0; h < numberOfHypotheses; h++)
    {
      // A marker that becomes true, when hypothesis contains double assignments and should be dropped.
      bool dropped = false;
      
      // A counter for the number of filled slots.
      unsigned int numberOfFilledSlots = 0;
      
      // A vector holding the assignments of parts to slots.
      std::vector<unsigned int> assignments;
      
      // Calculate the assignments of parts to slots. Drop hypotheses with double assignments of parts to slots.
      for(unsigned int s = 0; s < mNumberOfSlots; s++)
      {
	// Calculates which part should be associated with the 's'th slot, based on the 'h'th hypothesis.
	unsigned int part = (h / ((unsigned int) pow(numberOfEvidence, s)) ) % numberOfEvidence;
	
	if(part > 0)
	{
	  numberOfFilledSlots++;
	  
	  // If part is not zero-object and is already assigned, we drop this hypothesis and continue with the next one.
	  if(std::find(assignments.begin(), assignments.end(), part) != assignments.end())
	    dropped = true;
	}
	
	// Assign the calculated part to the next slot in the row.
	assignments.push_back(part);
      }
      
      // If the root node is occluded, we drop this hypothesis and continue with the next one.
      if(assignments[0] == 0)
	dropped = true;
      
      // Tell all factories to calculate the probability for the current hypothesis.
      if(!dropped)
      {
	// Generate a debug message with the results.
	std::stringstream msgResult;
	msgResult << "Assignment (";
	
	// Generate a list of all assignments for the debug message.
	for(unsigned int i = 0; i < assignments.size(); i++)
	{
	  if(assignments[i] == 0)
	     msgResult << "NULL";
	  else
	    msgResult << pEvidenceList[assignments[i] - 1].type;
	  
	  // Add separator.
	  if(i < assignments.size() - 1)
	    msgResult<< ", ";
	}
	
	// Add results to debug message
	msgResult << ") generates results (";
	
	// We multiply the terms of a hypothesis, so this must be ONE!
	double hValue = 1.0;
	
	// Iterate over all factories and multiply their output.
	mVisualizer->resetTermScores();
	for(unsigned int i = 0; i < mEvaluators.size(); i++)
	{
	  // Get the probability from the evaluator.
	  double termScore = mEvaluators[i]->calculateProbabilityForHypothesis(pEvidenceList, assignments);
	  
	  // Add term score to visualizer.
	  mVisualizer->addTermScore(termScore);
	  
	  // Add it to the debug message.
	  msgResult << termScore;
	  
	  // Add separator.
	  if(i < mEvaluators.size() - 1)
	    msgResult<< ", ";
	  
	  // Multiply the probability from the evaluators to the hypothesis probability.
	  hValue *= termScore;
	}
	
	// Finish debug message with hypothesis probability.
	msgResult << "). Hypothesis probability is '" << hValue << "'.";
	
	// Show the debug message composed above.
	if(hValue > 0.0)
	  ROS_DEBUG_STREAM(" > " << msgResult.str());
	
	// Multiply hypotheses with the numerator of the 'hypothesis length' normalization term.
	hValue *= numberOfFilledSlots;
	
	// Update visualizer with current hypothesis score.
	mVisualizer->setHypothesisScore(hValue);
	
	// Add the probability of the hypothesis to the sum.
	mProbability += hValue;
	
	// +1 for this valid hypothesis. A hypothesis is valid, if it scores more than zero.
	// Normalize probability with the denominator of the 'hypothesis length' normalization term.
	if(hValue > std::numeric_limits<double>::epsilon())
	{
	  numberOfValidHypotheses++;
	  sumOfFilledSlots += numberOfFilledSlots;
	}
      }
    }
    ROS_DEBUG_STREAM("THERE ARE " << numberOfValidHypotheses << " hypotheses larger than zero.");
    
    // Apply normalization term by dividing by the number of all VALID (!) hypotheses. 
    // Only apply term if there was one valid hypothesis, otherwise we'll have nan.
    if(numberOfValidHypotheses > 0)
    {
      mProbability /= (/*numberOfValidHypotheses * */sumOfFilledSlots);
      mVisualizer->normalizeHypothesisScore(sumOfFilledSlots);
    }
    
    // The term evaluators shall update their visualizers.
    for(boost::shared_ptr<TermEvaluator> evaluator : mEvaluators)
      evaluator->visualize(pEvidenceList);
    
    // Print scene object probability.
    ROS_DEBUG_STREAM("Probability for scene object before applying priori is " << mProbability << ".");
  }
  
  void OcmSceneObjectContent::update(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph)
  {
    // Forward scene graph to the term evaluators.
    for(unsigned int i = 0; i < mEvaluators.size(); i++)
      mEvaluators[i]->handleSceneGraph(pSceneGraph);
  }
  
  double OcmSceneObjectContent::getSceneObjectProbability()
  {
    return mProbability;
  }
  
  void OcmSceneObjectContent::setBestStatus(bool pStatus)
  {
    mVisualizer->setBestStatus(pStatus);
  }
  
}
