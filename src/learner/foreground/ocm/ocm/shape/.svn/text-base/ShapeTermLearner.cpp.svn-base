#include "learner/foreground/ocm/ocm/shape/ShapeTermLearner.h"

namespace ProbabilisticSceneRecognition {
 
  ShapeTermLearner::ShapeTermLearner()
  : TermLearner()
  , mPrivateNamespaceHandle("~")
  {    
  }
  
  ShapeTermLearner::~ShapeTermLearner()
  {
  }
  
  void ShapeTermLearner::learn(boost::shared_ptr<OcmModel> pModel)
  {
    // Iterate over all children of the given node.
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, pModel->mRoot->mChildren)
    {
      // Learn the pose.
      learnNodePose(pModel->mRoot, child);
      
      // ProceePositiond with the childs children.
      learn(child);
    }
  }
  
  void ShapeTermLearner::learn(boost::shared_ptr<OcmTree> pNode)
  {
    // Iterate over all children of the given node.
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, pNode->mChildren)
    {
      // Learn the pose.
      learnNodePose(pNode, child);
      
      // Proceed with the childs children.
      learn(child);
    }
  }
  
  void ShapeTermLearner::learnNodePose(boost::shared_ptr<OcmTree> pParent, boost::shared_ptr<OcmTree> pChild)
  {
    // Gets the length of the parent observation trajectory.
    unsigned int trajectoryLength = pParent->mObjectSet->mObjects.size();
    
    // Check that both observation trajectories have the same length.
    // If that's not the case, something went wrong in the Scene Graph Generator.
    if(trajectoryLength != pChild->mObjectSet->mObjects.size())
      throw std::runtime_error("Shape term learner: the observation trajectories of child and parent node don't have the same length. This indicates a bug in the scene_graph_generator.");
    
    // Try to get the minmal number of kernels.
    if(!mPrivateNamespaceHandle.getParam("kernels_min", mNumberKernelsMin))
      throw std::runtime_error("Please specify parameter " + std::string("kernels_min") + " when starting this node.");
    
    // Try to get the maximal number of kernels.
    if(!mPrivateNamespaceHandle.getParam("kernels_max", mNumberKernelsMax))
      throw std::runtime_error("Please specify parameter " + std::string("kernels_max") + " when starting this node.");
    
    // Try to get the number of training runs per kernel.
    if(!mPrivateNamespaceHandle.getParam("runs_per_kernel", mRunsPerKernel))
      throw std::runtime_error("Please specify parameter " + std::string("runs_per_kernel") + " when starting this node.");
    
    // Try to get the number of synthetic samples per kernel.
    if(!mPrivateNamespaceHandle.getParam("synthetic_samples", mNumberOfSyntheticSamples))
      throw std::runtime_error("Please specify parameter " + std::string("synthetic_samples") + " when starting this node.");
    
    // Try to get sample relaxiation interval for the position.
    if(!mPrivateNamespaceHandle.getParam("interval_position", mIntervalPosition))
      throw std::runtime_error("Please specify parameter " + std::string("interval_position") + " when starting this node.");
    
    // Try to get sample relaxiation interval for the orientation.
    if(!mPrivateNamespaceHandle.getParam("interval_orientation", mIntervalOrientation))
      throw std::runtime_error("Please specify parameter " + std::string("interval_orientation") + " when starting this node.");
    
    // Try to get the path for the orientation plot.
    if(!mPrivateNamespaceHandle.getParam("orientation_plot_path", mPathOrientationPlots))
      mPathOrientationPlots = "UNDEFINED";
    
    
    
    // Create leaners for position and orientation.
    GMMParameterEstimator learnerPosition(3, mNumberKernelsMin, mNumberKernelsMax, mRunsPerKernel, mNumberOfSyntheticSamples, mIntervalPosition, mIntervalOrientation, mPathOrientationPlots);
    GMMParameterEstimator learnerOrientation(4, mNumberKernelsMin, mNumberKernelsMax, mRunsPerKernel, mNumberOfSyntheticSamples, mIntervalPosition, mIntervalOrientation, mPathOrientationPlots);
    
    // Iterate over both trajectories, calculate the relative pose of child to parent and add it to the learners.
    for(unsigned int i = 0; i < trajectoryLength; i++)
    {
      // Extract the positions of child and parent.
      boost::shared_ptr<ResourcesForPsm::Pose> childPose(new ResourcesForPsm::Pose(pChild->mObjectSet->mObjects[i]));
      boost::shared_ptr<ResourcesForPsm::Pose> parentPose(new ResourcesForPsm::Pose(pParent->mObjectSet->mObjects[i]));
      
      // Calculate the relative pose between child and parent.
      boost::shared_ptr<ResourcesForPsm::Pose> relativePoseToParent;
      childPose->convertPoseIntoFrame(parentPose, relativePoseToParent);
      
      // Add the datum to the learner.
      learnerPosition.addDatum(relativePoseToParent->getPosition());
      learnerOrientation.addDatum(relativePoseToParent->getOrientation());
    }
    
    // Learn GMM over position and add model to child node.
    ROS_INFO("Learning gaussian mixture model over position.");
    learnerPosition.learn();
    learnerPosition.getModel(pChild->mGaussianMixtureModelPosition);
    
    // Learn GMM over orientation.
    ROS_INFO("Learning gaussian mixture model over orientation.");
    learnerOrientation.learn();
    learnerOrientation.getModel(pChild->mGaussianMixtureModelOrientation);
    
    // Export orientation plots.
    if(mPathOrientationPlots.compare("UNDEFINED") != 0)
      learnerOrientation.plotModel();
  }
  
}