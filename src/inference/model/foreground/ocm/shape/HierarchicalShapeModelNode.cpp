/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/shape/HierarchicalShapeModelNode.h"

namespace ProbabilisticSceneRecognition {
  
  HierarchicalShapeModelNode::HierarchicalShapeModelNode(boost::property_tree::ptree& pPt, unsigned int& pID): mWasVisited(false)
  {
    // Initialize shared pointer to gaussian mixture distribution;
    mGaussianMixtureDistributionPosition.reset(new GaussianMixtureDistribution(3));
    mGaussianMixtureDistributionOrientation.reset(new GaussianMixtureDistribution(4));

    // Execute the loading process.
    load(pPt, pID);
  }
  
  HierarchicalShapeModelNode::~HierarchicalShapeModelNode()
  {
  }
  
  void HierarchicalShapeModelNode::load(boost::property_tree::ptree& pPt, unsigned int& pID)
  {
    // Load the name of the object that is represented by this node.
    mSceneObject = pPt.get<std::string>("<xmlattr>.name");

    // Try to load the index of the object referenced by this node. If attribute is not found, node is not a reference.
    boost::optional<std::string> referenceTo = pPt.get_optional<std::string>("<xmlattr>.references");
    if (referenceTo)
    {
        mIsReference = true;
        mReferenceTo = boost::lexical_cast<unsigned int>(*referenceTo);
    }
    else
    {
        mIsReference = false;
        mReferenceTo = pID;
        pID++;
    }
    
    // Load the gaussian mixture distribution associated with this node.
    mGaussianMixtureDistributionPosition->load(pPt, "position");
    mGaussianMixtureDistributionOrientation->load(pPt, "orientation");
    
    // Load the childs of the node, if there are any.
    for(boost::property_tree::ptree::value_type &v: pPt)
    {
      // Only access the 'child' nodes.
      if(!std::strcmp(v.first.c_str(), "child"))
      {
          boost::shared_ptr<HierarchicalShapeModelNode> child(new HierarchicalShapeModelNode(v.second, pID));
          child->setParentObjectType(mSceneObject);
          mChildren.push_back(child);
      }
    }
  }
  
  void HierarchicalShapeModelNode::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for secondary scene object '" << mSceneObject << "'.");
    
    // Create a new coordinator for seondary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSecondarySceneObjectVisualization());

    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);

    // Initialize the visualizer for the gaussian mixture distribution.
    mGaussianMixtureDistributionPosition->initializeVisualizer(mVisualizer);

    // Iterate over all child nodes and append them, too!
    for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->initializeVisualizer(mSuperior);
  }
  
  void HierarchicalShapeModelNode::setAbsoluteParentPose(boost::shared_ptr<ISM::Pose> pPose)
  {
    mAbsoluteParentPose = pPose;
  }

  double HierarchicalShapeModelNode::calculateProbabilityForHypothesis(std::vector<ISM::Object> pEvidenceList, std::vector<unsigned int> pAssignments, unsigned int& pSlotId, bool pCut,
																		std::vector<boost::shared_ptr<ConditionalProbability>>& pConditionalProbabilities)
  {
      double result = 1.0;

      unsigned int oldSlotId = pSlotId;
      if (!mIsReference)
          // Go to the next slot.
          pSlotId++;
      else pSlotId = mReferenceTo;

      std::vector<boost::shared_ptr<HierarchicalShapeModelNode>> children;
      if (!mIsReference) children = mChildren;
      else children = mReferencedNode->getChildren();

      // Subtree already cut?
      if(pCut || pAssignments[pSlotId] == 0)
      {
          if (!mWasVisited)
              pConditionalProbabilities[pSlotId]->setProbability(mParentObject, 1.0);   // only if this node never gets visited through a not cut path, the associated probability remains set to 1.
          // Continue moving down the tree to increment the slot id.
          for(boost::shared_ptr<HierarchicalShapeModelNode> child: children)
              child->calculateProbabilityForHypothesis(pEvidenceList, pAssignments, pSlotId, true, pConditionalProbabilities);
      }
      else
      {
          mWasVisited = true;

          // Extract the pose of the object associates with this node/slot and convert it into the parent frame.
          mAbsolutePose.reset(new ISM::Pose(*pEvidenceList[pAssignments[pSlotId] - 1].pose));
          mAbsolutePose->convertPoseIntoFrame(mAbsoluteParentPose, mRelativePose);

          // Evaluate the relative pose under the the probability distribution describing the scene shape.
          // We remember: the scene shape is defined in the coordinate frame of the parent node.
          double scorePos = mGaussianMixtureDistributionPosition->evaluate(mRelativePose);
          double scoreOri = mGaussianMixtureDistributionOrientation->evaluate(mRelativePose);
          double score = scorePos * scoreOri;

          pConditionalProbabilities[pSlotId]->setProbability(mParentObject, score);

          //ROS_DEBUG_STREAM("Pose fitting report for scene object '" << mSceneObject <<"'. Position is " << scorePos << ", orientation is " << scoreOri << ". Total is " << score << ".");

          // Add score to global result.
          result *= score;

          // Forward the current score to the visualizer. If this is part of the best hypothesis,
          // it will be used for coloring the link to the parent object.
          mVisualizer->setBestPoseCandidate(score);

          // Ok, now we need to give the pose of this object to the children of this node.
          // Using this information and the evidence they're also able to calculate their probabilities.
          for(boost::shared_ptr<HierarchicalShapeModelNode> child: children)
          {
              // Update the tranformation of the child node (from world frame to this nodes frame).
              child->setAbsoluteParentPose(mAbsolutePose);

              // If zero-object was assigned to child, continue iterating down the tree to INCREMENT THE SLOT ID.
              // The returned probability will be one, so it has no influence at all.
              result *= child->calculateProbabilityForHypothesis(pEvidenceList, pAssignments, pSlotId, false, pConditionalProbabilities);
          }

          // Forward position of this primary scene object to visualizer.
          mVisualizer->setBestCandidatePose(mAbsolutePose);
      }

      if (mIsReference) pSlotId = oldSlotId;

    return result;
  }

  void HierarchicalShapeModelNode::visualize(std::vector<ISM::Object> pEvidenceList)
  {
    // Try to find evidence for this scene object.
    for(unsigned int i = 0; i < pEvidenceList.size(); i++)
    {
      // Get the object.
      ISM::Object object = pEvidenceList[i];

      
      // Is this evidence for this scene object (assumed that there is no detection uncertainty)?
      // If yes and the parent node was also detected, then update the position.
      if(mSceneObject.compare(object.type) == 0 && mAbsoluteParentPose)
      {
	// Extract the pose of the object associates with this node/slot and convert it into the parent frame.
    mAbsolutePose.reset(new ISM::Pose(*object.pose));
	mAbsolutePose->convertPoseIntoFrame(mAbsoluteParentPose, mRelativePose);
	
	/********************************************************
	* Now we draw.
	********************************************************/
	
	// Apply last known absolut object position to the visualizer.
	mVisualizer->setLastPose(mAbsolutePose);
	
	// Apply the absolute position of the parent object to the visualizer.
	mVisualizer->setParentPose(mAbsoluteParentPose);
	
	// Based on the assumption that there is no detection uncertainty for object types,
	// evaluate the gaussian mixture and visualize the uncertainty for the detection.
	mGaussianMixtureDistributionPosition->visualize(mVisualizer, mRelativePose);
	
	// For the given child node, set the pose of this node as parent pose in world.
	for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->setAbsoluteParentPose(mAbsolutePose);
      }
    }
    
    // Forward evidence to all child nodes.
    for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->visualize(pEvidenceList);
  }
  
  unsigned int HierarchicalShapeModelNode::getNumberOfNodes()
  {
    if (mIsReference) return 0;     // References not counted as unique nodes

    unsigned int result = 1;
    
    // Let the children of this node count their children.
    for(boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren)
    {
      result += child->getNumberOfNodes();
    }
    return result;
  }

  bool HierarchicalShapeModelNode::isReference(unsigned int& pReferenceTo)
  {
      pReferenceTo = mReferenceTo;
      return mIsReference;
  }

  std::vector<boost::shared_ptr<HierarchicalShapeModelNode>> HierarchicalShapeModelNode::getChildren()
  {
    return mChildren;
  }

  void HierarchicalShapeModelNode::setReferencedNode(boost::shared_ptr<HierarchicalShapeModelNode> pReferencedNode)
  {
    if (mSceneObject != pReferencedNode->getSceneObjectType())
        throw std::runtime_error("Node with type " + mSceneObject + " is trying to reference a node of different type " + pReferencedNode->getSceneObjectType());
    mReferencedNode = pReferencedNode;
  }

  void HierarchicalShapeModelNode::resetVisit()
  {
    mWasVisited = false;
    for (boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren) child->resetVisit();
  }
}
