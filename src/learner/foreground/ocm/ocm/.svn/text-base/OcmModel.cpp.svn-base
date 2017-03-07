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