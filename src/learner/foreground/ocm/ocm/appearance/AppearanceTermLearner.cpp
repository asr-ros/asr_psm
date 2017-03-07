/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

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
