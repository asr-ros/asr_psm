/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/OcmModel.h"

namespace ProbabilisticSceneRecognition {
 
  OcmModel::OcmModel(const boost::shared_ptr<SceneModel::TreeNode> pRoot)
  {
    // Create a new root node for the OCM tree.
    mRoot.reset(new OcmTree(pRoot));
  }
  
  OcmModel::~OcmModel()
  {
  }
  
  void OcmModel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior)
  {
    if(mRoot)
      mRoot->initializeVisualizer(mSuperior);
  }
  
  void OcmModel::save(double pWorkspaceVolume, boost::property_tree::ptree& pPt)
  {    
    // Save the number of slots.
    pPt.add("slots.<xmlattr>.number", mRoot->getNumberOfNodes());
    
    // Save the model.
    saveShape(pWorkspaceVolume, pPt);
    saveAppearance(pPt);
    saveOcclusion(pPt);
  }

    void OcmModel::saveShape(double pWorkspaceVolume, boost::property_tree::ptree& pPt)
    {
      // Create a seperate tree.
      boost::property_tree::ptree subtree;
      
      // Add root node.
      subtree.add("<xmlattr>.volume", pWorkspaceVolume);
      
      // Iterate over all children of the root node and save them.
      BOOST_FOREACH(boost::shared_ptr<OcmTree> child, mRoot->mChildren)
	child->saveShape(subtree);
      
      // Add subtree to main tree.
      pPt.add_child("shape.root", subtree);
    }
    
    void OcmModel::saveAppearance(boost::property_tree::ptree& pPt)
    {
      // Create a seperate tree.
      boost::property_tree::ptree subtree;
      
      // Save the appearance mapping.
      mAppearanceTable->save(subtree);
      
      // Add subtree to main tree.
      pPt.add_child("appearance", subtree);
    }
    
    void OcmModel::saveOcclusion(boost::property_tree::ptree& pPt)
    {
      // Create a seperate tree.
      boost::property_tree::ptree subtree;
      
      // Save the hypothesis mapping.
      mOcclusionTable->save(subtree);
      
      // Add subtree to main tree.
      pPt.add_child("occlusion", subtree);
    }
    
    unsigned int OcmModel::getNumberOfSlots()
    {
      return mRoot->getNumberOfNodes();
    }
    
}
