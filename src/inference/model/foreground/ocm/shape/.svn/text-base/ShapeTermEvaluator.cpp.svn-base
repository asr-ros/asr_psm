#include "inference/model/foreground/ocm/shape/ShapeTermEvaluator.h"

namespace ProbabilisticSceneRecognition {
 
  ShapeTermEvaluator::ShapeTermEvaluator()
  : TermEvaluator()
  {
  }
  
  ShapeTermEvaluator::~ShapeTermEvaluator()
  {
  }
  
  void ShapeTermEvaluator::load(boost::property_tree::ptree& pPt)
  {   
    // Load hierarchical shape model
    mHsm.load(pPt);
  }
  
  void ShapeTermEvaluator::handleSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    mHsm.handleSceneGraph(pSceneGraph);
  }
  
  void ShapeTermEvaluator::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Forward visualizer to hierarchical shape model.
    mHsm.initializeVisualizer(mSuperior);
  }
  
  double ShapeTermEvaluator::calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments)
  {
    return mHsm.calculateProbabilityForHypothesis(pEvidenceList, pAssignments);
  }
  
  void ShapeTermEvaluator::visualize(std::vector<pbd_msgs::PbdObject> pEvidenceList)
  {
    mHsm.visualize(pEvidenceList);
  }
  
  unsigned int ShapeTermEvaluator::getNumberOfSlots()
  {
    return mHsm.getNumberOfNodes();
  }
  
}