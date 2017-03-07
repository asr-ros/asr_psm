#include "learner/foreground/ocm/ocm/appearance/AppearanceTermLearner.h"

namespace ProbabilisticSceneRecognition {
 
  AppearanceTermLearner::AppearanceTermLearner()
  : TermLearner()
  {    
  }
  
  AppearanceTermLearner::~AppearanceTermLearner()
  {
  }
  
  void AppearanceTermLearner::learn(boost::shared_ptr<OcmModel> pModel)
  {
    // Create the appearance table.
    pModel->mAppearanceTable.reset(new MappedProbabilityTable());
    
    // Create an index of all object types. Use the secure way and iterate over the types of all observations,
    // not the type information in the node.
    learnMapping(pModel, pModel->mRoot);
    
    // Now initialize the probability table, based on the information gathered before.
    pModel->mAppearanceTable->initializeTable(pModel->getNumberOfSlots());
    
    // Iterate over all the observations of all nodes again and build the table.
    unsigned int slot = 0;
    learnTable(pModel, pModel->mRoot, slot);
    
    // Normalize the table.
    pModel->mAppearanceTable->normalize();
  }
  
  void AppearanceTermLearner::learnMapping(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode)
  {
    // Iterate over all observations for the given node and create the mapping.
    BOOST_FOREACH(boost::shared_ptr<SceneModel::Object> object, pNode->mObjectSet->mObjects)
      pModel->mAppearanceTable->add(object->mType);

    // Iterate over all child nodes and make them learn the mapping.
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, pNode->mChildren)
      learnMapping(pModel, child);
  }
  
  void AppearanceTermLearner::learnTable(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode, unsigned int& pSlot)
  {
    // Iterate over all observations for the given node and count
    BOOST_FOREACH(boost::shared_ptr<SceneModel::Object> object, pNode->mObjectSet->mObjects)
      pModel->mAppearanceTable->add(pSlot, object->mType);
    
    // Add no additional appearances of the default class (=class for unknown objects).
    pModel->mAppearanceTable->setDefaultClassCounter(pSlot, 0);
    
    // Increment slot number
    pSlot++;
    
    // Make the children do their homework.
    BOOST_FOREACH(boost::shared_ptr<OcmTree> child, pNode->mChildren)
      learnTable(pModel, child, pSlot);
  }
  
}