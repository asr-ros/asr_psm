/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/background/BackgroundSceneLearner.h"

namespace ProbabilisticSceneRecognition {

  BackgroundSceneLearner::BackgroundSceneLearner()
  : SceneLearner()
  {

  }
  
  BackgroundSceneLearner::~BackgroundSceneLearner()
  {
  }
  
  void BackgroundSceneLearner::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // No need for visualization here.
  }
  
  void BackgroundSceneLearner::save(boost::property_tree::ptree& pPt)
  {
    // Create mSceneName = pExample->mIdentifier;a subtree.
    boost::property_tree::ptree subtree;
    
    // Add scene name and type.
    subtree.add("<xmlattr>.type", "background");
    subtree.add("<xmlattr>.name", "background");
    subtree.add("<xmlattr>.priori", mPriori);
    
    // Add parameters relevant for the background scene.
    subtree.add("description.<xmlattr>.objects", mMaximalNumberOfObjects);
    subtree.add("description.<xmlattr>.volume", mWorkspaceVolume);
    
    // Add subtree to main tree.
    pPt.add_child("psm.scenes.scene", subtree);
  }

  void BackgroundSceneLearner::learn()
  {
    // This is the list we store all object types. The size of the list will be the number of all unique objects.
    std::vector<std::string> index;

    ISM::TracksPtr alltracks(new ISM::Tracks(mExamplesList));
    // Iterate over all examples for all scenes.

    for (auto &track : alltracks->tracks){
      // Iterate over all PbdNodes in the scene (a node represent an object).
      //BOOST_FOREACH(pbd_msgs::PbdNode node, example->scene_elements)
      //{
	// Extract the object type from the first observation.
    std::string type = track->type;
	
	// If the object is not already known, add it to the list of known objects.
	if(std::find(index.begin(), index.end(), type) == index.end())
	  index.push_back(type);
      }
   // }
    
    // This is the number of unique objects.
    mMaximalNumberOfObjects = index.size();
    }

}
