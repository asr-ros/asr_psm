#include "learner/foreground/ocm/ocm/OcmTree.h"

namespace ProbabilisticSceneRecognition {
  
  OcmTree::OcmTree(const boost::shared_ptr<SceneModel::TreeNode> pRoot)
  {
    // Set the type.
    mType = pRoot->mObjectSet->mObjects[0]->mType;
    
    // Take object observations.
    mObjectSet = pRoot->mObjectSet;
    
    // Create child nodes and copy observations.
    BOOST_FOREACH(boost::shared_ptr<SceneModel::TreeNode> child, pRoot->mChildren)
      mChildren.push_back(boost::shared_ptr<OcmTree>(new OcmTree(child)));
  }
  
  OcmTree::~OcmTree()
  {
  }
  
  void OcmTree::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for secondary scene object '" << mType << "'.");
    
    // Initialize the visualizer for the gaussian mixture distribution.
    mGaussianMixtureModelPosition.initializeVisualizer(mVisualizer);
    
    // Set the pose of the parent object.
    unsigned int lastObservationOffset = mObjectSet->mObjects.size() - 1;
    mSuperior->setPose(boost::shared_ptr<ResourcesForPsm::Pose>(new ResourcesForPsm::Pose(mObjectSet->mObjects[lastObservationOffset])));
    
    // Iterate over all child nodes and append them, too!
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mChildren)
      child->initializeVisualizer(mSuperior, this);
  }
  
  void OcmTree::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior,
				     OcmTree* pParent)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for secondary scene object '" << mType << "'.");
    
    // Create a new coordinator for seondary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSecondarySceneObjectVisualization(mType));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
    
    // Initialize the visualizer for the gaussian mixture distribution.
    mGaussianMixtureModelPosition.initializeVisualizer(mVisualizer);
    
    // Extract the trajectory used for learning to the visualizer.
    std::vector<Eigen::Vector3d> mRelativeSamples;
    std::vector<Eigen::Vector3d> mAbsoluteSamples;
    std::vector<Eigen::Vector3d> mParentSamples;
    for(unsigned int i = 0; i < mObjectSet->mObjects.size(); i++)
    {
      // Extract the positions of child and parent.
      boost::shared_ptr<ResourcesForPsm::Pose> childPose(new ResourcesForPsm::Pose(mObjectSet->mObjects[i]));
      boost::shared_ptr<ResourcesForPsm::Pose> parentPose(new ResourcesForPsm::Pose(pParent->mObjectSet->mObjects[i]));
      
      // All samples and gaussian kernels will be drawn relative to the praent object.
      mVisualizer->setParentPose(parentPose);
      
      // Calculate the relative pose between child and parent.
      boost::shared_ptr<ResourcesForPsm::Pose> relativePoseToParent;
      childPose->convertPoseIntoFrame(parentPose, relativePoseToParent); 
      
      // Add relative sample, absolute sample and corresponding parent pose to the list.
      mAbsoluteSamples.push_back(childPose->getPosition());
      mParentSamples.push_back(parentPose->getPosition());
      mRelativeSamples.push_back(relativePoseToParent->getPosition());
    }
    
    // Forward relative learning samples, absolute learning samples and the corresponding parent poses
    // for the secondary scene object to the visualizer.
    mVisualizer->setAbsoluteLearningSamples(mRelativeSamples, mAbsoluteSamples, mParentSamples);
    
    // Iterate over all child nodes and append them, too!
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mChildren)
      child->initializeVisualizer(mSuperior, this);
  }
  
  void OcmTree::saveShape(boost::property_tree::ptree& pPt)
  {
    // Create a seperate tree.
    boost::property_tree::ptree subtree;
    
    // Add the name of the child.
    subtree.add("<xmlattr>.name", mType);
    
    // Save the pose.
    mGaussianMixtureModelPosition.save(subtree, "position");
    mGaussianMixtureModelOrientation.save(subtree, "orientation");
    
    // Iterate over all children of the root node and save them.
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mChildren)
      child->saveShape(subtree);
    
    // Add subtree to main tree.
    pPt.add_child("child", subtree);
  }
  
  unsigned int OcmTree::getNumberOfNodes()
  {
    unsigned int result = 1;
    
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mChildren)
      result += child->getNumberOfNodes();
    
    return result;
  }
  
}