/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/shape/ShapeTermLearner.h"

namespace ProbabilisticSceneRecognition {
 
  ShapeTermLearner::ShapeTermLearner(std::string pSceneName)
  : TermLearner()
  , mPrivateNamespaceHandle("~")
  , mSceneName(pSceneName)
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
    unsigned int trajectoryLength = pParent->mObjectSet->objects.size();
    
    // Check that both observation trajectories have the same length.
    // If that's not the case, something went wrong in the Scene Graph Generator.
    if(trajectoryLength != pChild->mObjectSet->objects.size())
      throw std::runtime_error("Shape term learner: the observation trajectories of child and parent node don't have the same length. This indicates a bug in the scene_graph_generator.");
    
    // Try to get the minmal number of kernels.
    getParameter("kernels_min", mNumberKernelsMin);

    // Try to get the maximal number of kernels.
    getParameter("kernels_max", mNumberKernelsMax);

    // Try to get the number of training runs per kernel.
    getParameter("runs_per_kernel", mRunsPerKernel);

    // Try to get the number of synthetic samples per kernel.
    getParameter("synthetic_samples", mNumberOfSyntheticSamples);

    // Try to get sample relaxiation interval for the position.
    getParameter("interval_position", mIntervalPosition);

    // Try to get sample relaxiation interval for the orientation.
    getParameter("interval_orientation", mIntervalOrientation);

    // Try to get the path for the orientation plot.
    if(!mPrivateNamespaceHandle.getParam("orientation_plot_path", mPathOrientationPlots))
      mPathOrientationPlots = "UNDEFINED";

    // Try to get the number of attempts per run.
    getParameter("attempts_per_run", mAttemptsPerRun);
    
    // Create leaners for position and orientation.
    GMMParameterEstimator learnerPosition(3, mNumberKernelsMin, mNumberKernelsMax, mRunsPerKernel, mNumberOfSyntheticSamples, mIntervalPosition, mIntervalOrientation, mPathOrientationPlots, mAttemptsPerRun);
    GMMParameterEstimator learnerOrientation(4, mNumberKernelsMin, mNumberKernelsMax, mRunsPerKernel, mNumberOfSyntheticSamples, mIntervalPosition, mIntervalOrientation, mPathOrientationPlots, mAttemptsPerRun);
    
    // Iterate over both trajectories, calculate the relative pose of child to parent and add it to the learners.
    for(unsigned int i = 0; i < trajectoryLength; i++)
    {
      // Extract the positions of child and parent.
      boost::shared_ptr<ISM::Pose> childPose(pChild->mObjectSet->objects[i]->pose);
      boost::shared_ptr<ISM::Pose> parentPose(pParent->mObjectSet->objects[i]->pose);
      
      // Calculate the relative pose between child and parent.
      boost::shared_ptr<ISM::Pose> relativePoseToParent;
      childPose->convertPoseIntoFrame(parentPose, relativePoseToParent);
      
      // Add the datum to the learner.
      learnerPosition.addDatum(relativePoseToParent->point->getEigen());
      learnerOrientation.addDatum(relativePoseToParent->quat->getEigen());
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

  void ShapeTermLearner::getParameter(std::string pParameterName, double& pParameter)
  {
      ros::NodeHandle localNamespaceHandle("~");
      if (mSceneName != "")
          localNamespaceHandle = ros::NodeHandle("~/" + mSceneName);
      if(!localNamespaceHandle.getParam(pParameterName, pParameter))
      {
          ROS_DEBUG_STREAM("Could not find parameter " << "~/" << mSceneName << "/" << pParameterName << ". Using unspecific namespace instead.");
          if (!mPrivateNamespaceHandle.getParam(pParameterName, pParameter))
              throw std::runtime_error("Please specify parameter " + pParameterName + " when starting this node.");
      }
  }

  void ShapeTermLearner::getParameter(std::string pParameterName, int& pParameter)
  {
      double tempParameter;
      getParameter(pParameterName, tempParameter);
      pParameter = (int) tempParameter;
  }
  
}
