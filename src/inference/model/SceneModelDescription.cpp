/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/SceneModelDescription.h"

namespace ProbabilisticSceneRecognition {

  SceneModelDescription::SceneModelDescription()
  {
  }
  
  SceneModelDescription::~SceneModelDescription()
  {
  }
  
  void SceneModelDescription::loadModelFromFile(std::string pPathToFile, std::string pAlgorithm)
  {
    // Check, if the file containing the model exist.
    if(!boost::filesystem::exists(pPathToFile))
      throw std::invalid_argument("Unable to procees loading. The model file doesn't exist!");
    
    // Status information for the user.
    ROS_INFO_STREAM("Loading scene model from this location: " << pPathToFile);
    
    // Load model file into memory.
    boost::property_tree::ptree pt;
    read_xml(pPathToFile, pt);
    
    // Recursively load all scenes found in the XML file.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("psm.scenes"))
    {
      // Create a new fore- or background scene instance.
      boost::shared_ptr<SceneDescription> scene(new SceneDescription());
      
      // Let it load its parameters...
      scene->load(v.second, pAlgorithm);
      
      // ... and list it.
      mScenes.push_back(scene);
    }
  }
  
  void SceneModelDescription::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // Iterate over all scenes and initialize their visualizers.
    for(boost::shared_ptr<SceneDescription> scene : mScenes)
	scene->initializeVisualizer(mSuperior);
  }

  void SceneModelDescription::integrateEvidence(const boost::shared_ptr<const ISM::Object>& pObject)

  {
    // Add the evidence found to the buffer.
    mObjectEvidence.push(pObject);
  }
  
  void SceneModelDescription::integrateSceneGraph(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph)
  {
    // Forward the scene graph to the scenes.
    BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
      scene->update(pSceneGraph);
  }
  
  void SceneModelDescription::updateModel()
  {
    // Check if new evidences have been found since the last update.
    if(mObjectEvidence.hasWaitingEvidences())
    {
      // Integrate the evidences into what we already know.
      mObjectEvidence.update();
      
      // Get a list of ALL evidence...
      mObjectEvidence.getEvidences(mEvidenceList);
      
      // ... and forward it down the model.
      BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
	scene->update(mEvidenceList);
    }
  }
  
  void SceneModelDescription::getSceneListWithProbabilities(std::vector<SceneIdentifier>& pSceneList)
  {
    double sum = 0.0;
    
    pSceneList.clear();
    
    // Recalculate the probabilities of all scenes and collect their scene identifiers.
    BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
    {      
      // Recalculate probability.
      scene->calculateSceneProbaility();
      
      // Get the scene identifier and sum it up.
      SceneIdentifier i = *(scene->getSceneIdentifier());
      sum += i.mLikelihood;
      
      // Add the identifier to the list.
      pSceneList.push_back(i);
    }
    
    // Normalize the probabilities.
    if(sum > 0.0)
      for(unsigned int i = 0; i < pSceneList.size(); i++)
	pSceneList[i].mLikelihood /= sum;
  }
  
}
