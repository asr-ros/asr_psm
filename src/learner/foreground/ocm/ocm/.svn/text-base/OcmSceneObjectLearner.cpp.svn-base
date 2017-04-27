#include "learner/foreground/ocm/ocm/OcmSceneObjectLearner.h"

namespace ProbabilisticSceneRecognition {

  OcmSceneObjectLearner::OcmSceneObjectLearner(std::string pSceneObjectType)
  : SceneObjectLearner(pSceneObjectType)
  {
    // Term learners should be specified here.
    mTermLearners.push_back(boost::shared_ptr<TermLearner>(new ShapeTermLearner()));
    mTermLearners.push_back(boost::shared_ptr<TermLearner>(new AppearanceTermLearner()));
    mTermLearners.push_back(boost::shared_ptr<TermLearner>(new OcclusionTermLearner()));
  }
  
  OcmSceneObjectLearner::~OcmSceneObjectLearner()
  {
  }
  
  void OcmSceneObjectLearner::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for primary scene object '" << mSceneObjectType << "'.");
    
    // Create a new coordinator for seondary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticPrimarySceneObjectVisualization(mSceneObjectType));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
   
    // Forward visualizer to the OCM model.
    mOcmModel->initializeVisualizer(mVisualizer);
  }
  
  void OcmSceneObjectLearner::save(boost::property_tree::ptree& pPt)
  {
    // Create a seperate tree.
    boost::property_tree::ptree subtree;
    
    // Add scene object parameters.
    subtree.add("<xmlattr>.name", mSceneObjectType);
    subtree.add("<xmlattr>.type", "ocm");
    subtree.add("<xmlattr>.priori", mPriori);
    
    // Write the OCM tree to XML.
    mOcmModel->save(mWorkspaceVolume, subtree);
    
    // Add subtree to main tree.
    pPt.add_child("object", subtree);
  }
  
  void OcmSceneObjectLearner::learn(std::vector<boost::shared_ptr<const pbd_msgs::PbdSceneGraph> > pExamplesList,
    boost::shared_ptr<SceneModel::TreeNode> pTree)
  {
    if(!pTree)
      throw std::runtime_error("FATAL ERROR: Relation graph generator wasn't able to provide a tree.");
    
    ROS_INFO_STREAM("Rearranging tree for object of type " << mSceneObjectType << ".");
    
    // The tree needs to be rearranged. The node with the given type has to be root node.
    pTree = pTree->setNewRootNodeByType(mSceneObjectType);
    
    // Print tree.
    ROS_INFO_STREAM("Tree rearranged for object of type" << mSceneObjectType << ".");
    std::cout << "------------- TREE:" << std::endl;
    pTree->printTreeToConsole(0);
    std::cout << "---------------------" << std::endl;
    
    // Build the OCM tree.
    mOcmModel = boost::shared_ptr<OcmModel>(new OcmModel(pTree));
    
    // Iterate over all learners and let them calculate the model parameters.
    BOOST_FOREACH(boost::shared_ptr<TermLearner> learner, mTermLearners)
      learner->learn(mOcmModel);
  }
  
}