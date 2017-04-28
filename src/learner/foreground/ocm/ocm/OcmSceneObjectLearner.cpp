/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

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
  
  void OcmSceneObjectLearner::learn(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph> > pExamplesList,
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
