/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ForegroundSceneContent.h"

namespace ProbabilisticSceneRecognition {
 
  ForegroundSceneContent::ForegroundSceneContent()
  : SceneContent()
  {
    mSceneObjects.reset(new std::vector<boost::shared_ptr<SceneObjectDescription> >());
  }
  
  ForegroundSceneContent::~ForegroundSceneContent()
  {
  }
  
  void ForegroundSceneContent::load(boost::property_tree::ptree& pPt)
  {
    // load the inference algorithm.
    loadInferenceAlgorithm(pPt);
    
    // Load all scenen objects which are part of this scene.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pPt)
    {
      // Only access the 'object' child nodes.
      if(!std::strcmp(v.first.c_str(), "object"))
      {
	// Create a new foreground scene instance.
	boost::shared_ptr<SceneObjectDescription> sceneObject(new SceneObjectDescription());
	
	// Let it load its parameters ...
	sceneObject->load(v.second);
	
	// ... and list it.
	mSceneObjects->push_back(sceneObject);
      }
    }
  }
  
  void ForegroundSceneContent::initializeInferenceAlgorithms(std::string pAlgorithm)
  {
    // Select the inference algorithm described by the parameter.
    if(pAlgorithm.compare("powerset") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new PowerSetForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("summarized") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new SummarizedForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("multiplied") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MultipliedForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("maximum") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MaximumForegroundInferenceAlgorithm(mSceneObjects)));
    } else {
      throw std::invalid_argument("Unable to procees loading. The inference algorithm of type '" + pAlgorithm + "' is unknown to the scene of type 'foreground'.");
    }
  }
  
  void ForegroundSceneContent::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // Iterate over all primary scene objects and initialize their visualizers.
    for(unsigned int i = 0; i < mSceneObjects->size(); i++)
      mSceneObjects->at(i)->initializeVisualizer(mSuperior);
  }
  
  void ForegroundSceneContent::update(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Commandy the inference algorithm to execute the inference.
    doInference(pEvidenceList, pRuntimeLogger);
  }
  
  void ForegroundSceneContent::update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    // Forward scene graph to all primary scene objects.
    for(unsigned int i = 0; i < mSceneObjects->size(); i++)
      mSceneObjects->at(i)->update(pSceneGraph);
  }
  
}
