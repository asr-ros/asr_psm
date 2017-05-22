/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/shape/HierarchicalShapeModel.h"

namespace ProbabilisticSceneRecognition {
 
  HierarchicalShapeModel::HierarchicalShapeModel()
  {
      ros::NodeHandle nodeHandle("~");
      // Try to get the type of the conditional probability algorithm.
      if(!nodeHandle.getParam("conditional_probability_algorithm", mConditionalProbabilityAlgorithm))
      {
         ROS_INFO_STREAM("Could not find ROS parameter conditional_probability_algorithm. Using standard method of using the minimum (value=\"minimum\").");
         mConditionalProbabilityAlgorithm = "minimum";     // for compatability with old launch files.
      }
  }
  
  HierarchicalShapeModel::~HierarchicalShapeModel()
  {
  }
  
  void HierarchicalShapeModel::load(boost::property_tree::ptree& pPt)
  {
    // Load the volume of the space we're working in. It is required for the uniform distribution.
    mWorkspaceVolume = pPt.get<double>("shape.root.<xmlattr>.volume");

    unsigned int id = 1;
    // this, the root, implicitly has id 0.

    // Load the childs of the root node.
    for(boost::property_tree::ptree::value_type &v: pPt.get_child("shape.root"))
    {
      // Only access the 'child' child nodes.
      if(!std::strcmp(v.first.c_str(), "child"))
    mChildren.push_back(boost::shared_ptr<HierarchicalShapeModelNode>(new HierarchicalShapeModelNode(v.second, id)));
    }

    // Assign references based on the IDs:
    std::vector<boost::shared_ptr<HierarchicalShapeModelNode>> nodesToVisit;
    for (boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren)
        nodesToVisit.push_back(child);

    std::vector<boost::shared_ptr<HierarchicalShapeModelNode>> references;
    std::map<unsigned int, boost::shared_ptr<HierarchicalShapeModelNode>> nonReferencesById;
    while(!nodesToVisit.empty())
    {
        // pop first node:
        boost::shared_ptr<HierarchicalShapeModelNode> currentNode = nodesToVisit[0];
        nodesToVisit.erase(nodesToVisit.begin());

        unsigned int referenceTo;
        bool isReference = currentNode->isReference(referenceTo);
        if (!isReference)
        {
            nonReferencesById[referenceTo] = currentNode;
            for (boost::shared_ptr<HierarchicalShapeModelNode> child: currentNode->getChildren())
                nodesToVisit.push_back(child);
        }
        else
        {
            references.push_back(currentNode);
        }
    }

    for (boost::shared_ptr<HierarchicalShapeModelNode> currentReference: references)
    {
        unsigned int referenceTo;
        if (!currentReference->isReference(referenceTo)) throw std::runtime_error("Trying to make a non-reference node a reference.");

        currentReference->setReferencedNode(nonReferencesById[referenceTo]);
    }
  }
  
  void HierarchicalShapeModel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    // Store a copy of the primary scene object visualizer.
    mVisualizer = mSuperior;
    
    // Forward visualizer to the child nodes representing the secondary scene objects.
    for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->initializeVisualizer(mSuperior);
  }

  double HierarchicalShapeModel::calculateProbabilityForHypothesis(std::vector<ISM::Object> pEvidenceList, std::vector<unsigned int> pAssignments)
  {
      for (boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren) child->resetVisit();

    double result = 1.0;

    // If no part was assigned to the root node, we don't need to continue (because the model doesn't allow cases like this).
    // Otherwise go down the rabbit hole and see what you'll find!
    if(pAssignments[0] > 0)
    {
      
      // This helper variable keeps track of the slot we're currently on. It will be given via reference to the nodes.
      // We have a tree structure and do a iteration similar to deep search here to 'serialize the tree'.
      unsigned int slotId = 0;
      
      // Converts the AsrObject assigned to this slot into the Pose data structure.
      // Subtract one from the assignment, because the first evidence is stored at position zero.
      mAbsolutePose.reset(new ISM::Pose(*pEvidenceList[pAssignments[0] - 1].pose));
      
      // Evaluate evidence for root node under uniform distribution (FOR EVERY DIMENSION. Could only be done, it a root object was assigned to the root node.
      // THIS IS NECESSARY! When we have only one object, a scene containing it and a background scene,
      // then the probabilities MUST BE 50/50 (of course assumed that both scenes only consider shape)!
      double rootResult = 1.0 / (mWorkspaceVolume * 2.0 * pow(M_PI, 2.0));
      result *= rootResult;
      
      // vector to hold the conditional probability of each slot:
      std::vector<boost::shared_ptr<ConditionalProbability>> conditionalProbabilities(pAssignments.size());
      // initialize conditional probabilites:
      if (mConditionalProbabilityAlgorithm == "minimum")
          for (unsigned int i = 0; i < conditionalProbabilities.size(); i++)
              conditionalProbabilities[i] = boost::shared_ptr<ConditionalProbability>(new MinimumConditionalProbability());
      else if (mConditionalProbabilityAlgorithm == "multiplied")
          for (unsigned int i = 0; i < conditionalProbabilities.size(); i++)
              conditionalProbabilities[i] = boost::shared_ptr<ConditionalProbability>(new MultipliedConditionalProbability());
      else if (mConditionalProbabilityAlgorithm == "root_of_multiplied")
          for (unsigned int i = 0; i < conditionalProbabilities.size(); i++)
              conditionalProbabilities[i] = boost::shared_ptr<ConditionalProbability>(new RootOfMultipliedConditionalProbability());
      else if (mConditionalProbabilityAlgorithm == "average")
          for (unsigned int i = 0; i < conditionalProbabilities.size(); i++)
              conditionalProbabilities[i] = boost::shared_ptr<ConditionalProbability>(new AverageConditionalProbability());
      else throw std::runtime_error("In HierarchicalShapeModel::calculateProbabilityForHypothesis(): conditional probability algorithm type "
                                    + mConditionalProbabilityAlgorithm + " is invalid. Valid types are minimum.");

      // set probability of reference object (represented by this class):
      conditionalProbabilities[0]->addProbability(rootResult);

      // Iterate over all children, assign the given evidence to them and evaluate
      for(boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren)
      {	
	// Update the transformation from world coordinates to parent coordinates.
    child->setAbsoluteParentPose(mAbsolutePose);
	
	// If zero-object was assigned to child, continue iterating down the tree to set the right slot id,
	// but don't use the probabilities based on the occluded subtree.
    //result *= child->calculateProbabilityForHypothesis(pEvidenceList, pAssignments, slotId, pAssignments[0] == 0);
    result *= child->calculateProbabilityForHypothesis(pEvidenceList, pAssignments, slotId, pAssignments[0] == 0, conditionalProbabilities);
      }

      double slotProduct = 1.0;
      for (boost::shared_ptr<ConditionalProbability> cP: conditionalProbabilities)
          slotProduct *= cP->getProbability();
      // keeping the old result around for comparison:
      /*if (slotProduct != result) std::cout << slotProduct << " (min), " << result << " (mul)" << std::endl;
      else std::cout << slotProduct << "(min&mul)" << std::endl;*/
      result = slotProduct; // overwriting it for output.
      
      // Forward position of this primary scene object to visualizer.
      mVisualizer->setBestPoseCandidate(mAbsolutePose);
    } else {
      // A hypothesis without an assigned root object is invalid and will therefore be scored with the impossible event.
      result = 0.0;
    }
    return result;
  }

  void HierarchicalShapeModel::visualize(std::vector<ISM::Object> pEvidenceList)
  {
    // Get the name of the primary scene object.
    std::string name = mVisualizer->getSceneObjectName();
    
    // Try to find evidence for this scene object.
    for(unsigned int i = 0; i < pEvidenceList.size(); i++)
    {
      // Is this evidence for this scene object?
      if(name.compare(pEvidenceList[i].type) == 0)
      {
	// Extract pose of the object.
    mAbsolutePose.reset(new ISM::Pose(*pEvidenceList[i].pose));
	
	// Forward the absolute pose of the primary scene object to the visualizer.
	mVisualizer->setPose(mAbsolutePose);
	
	// Forward the pose of this node as parent node pose to the child nodes.
	for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->setAbsoluteParentPose(mAbsolutePose);
      }
    }
    
    // Forward visualization command to the child nodes representing the secondary scene objects.
    for(unsigned int i = 0; i < mChildren.size(); i++)
      mChildren[i]->visualize(pEvidenceList);
  }
  
  unsigned int HierarchicalShapeModel::getNumberOfNodes()
  {
    unsigned int result = 1;
    
    // Let the children of this node count their children.
    for(boost::shared_ptr<HierarchicalShapeModelNode> child: mChildren)
    {
      result += child->getNumberOfNodes();
    }
    return result;
  }
  
}
