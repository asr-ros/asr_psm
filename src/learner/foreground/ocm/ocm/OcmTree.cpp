/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/OcmTree.h"

namespace ProbabilisticSceneRecognition {
  
  OcmTree::OcmTree(const boost::shared_ptr<SceneModel::TreeNode> pRoot)
  {
    // Set the type.
    mType = pRoot->mObjectSet->objects[0]->type;
    
    // Take object observations.
    mObjectSet = pRoot->mObjectSet;

    mIsReference = pRoot->mIsReference;
    if (mIsReference) mReferenceToID = pRoot->mReferenceTo->mID;
    
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
    unsigned int lastObservationOffset = mObjectSet->objects.size() - 1;
    mSuperior->setPose(*new boost::shared_ptr<ISM::Pose>(new ISM::Pose(*mObjectSet->objects[lastObservationOffset]->pose)));
    
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
    for(unsigned int i = 0; i < mObjectSet->objects.size(); i++)
    {
      // Extract the positions of child and parent.
      boost::shared_ptr<ISM::Pose> childPose(*new boost::shared_ptr<ISM::Pose>(new ISM::Pose(*mObjectSet->objects[i]->pose)));
      boost::shared_ptr<ISM::Pose> parentPose(*new boost::shared_ptr<ISM::Pose>(new ISM::Pose(*pParent->mObjectSet->objects[i]->pose)));
      
      // All samples and gaussian kernels will be drawn relative to the praent object.
      mVisualizer->setParentPose(parentPose);
      
      // Calculate the relative pose between child and parent.
      boost::shared_ptr<ISM::Pose> relativePoseToParent;
      childPose->convertPoseIntoFrame(parentPose, relativePoseToParent); 
      
      // Add relative sample, absolute sample and corresponding parent pose to the list.
      mAbsoluteSamples.push_back(childPose->point->eigen);
      mParentSamples.push_back(parentPose->point->eigen);
      mRelativeSamples.push_back(relativePoseToParent->point->eigen);
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

    // If reference: add ID of referenced
    if (mIsReference) subtree.add("<xmlattr>.references", mReferenceToID);
    
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
    if (mIsReference) return 0; // Do not count references as unique nodes

    unsigned int result = 1;
    
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mChildren)
      result += child->getNumberOfNodes();
    
    return result;
  }
  
}
