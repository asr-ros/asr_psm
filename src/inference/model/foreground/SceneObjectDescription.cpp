/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/SceneObjectDescription.h"

namespace ProbabilisticSceneRecognition {
  
  SceneObjectDescription::SceneObjectDescription()
  {
  }
  
  SceneObjectDescription::~SceneObjectDescription()
  {
  }
  
  void SceneObjectDescription::load(boost::property_tree::ptree& pPt)
  {
    // Extract scene object name and the type of the content.
    mPriori = pPt.get<double>("<xmlattr>.priori");
    mType = pPt.get<std::string>("<xmlattr>.type");
    mDescription = pPt.get<std::string>("<xmlattr>.name");
    
    ROS_INFO_STREAM("Loading primary scene with type '" << mType << "' and name '" << mDescription << "'.");
    
    // Check, if there was a description specified.
    if(mDescription.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene object specified.");
    
    // Check, if there was a scene object type specified.
    if(mType.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene object '" + mDescription + "' spedified.");
    
    // Check, if there the a priori probability is valid.
    if(mPriori < 0.0 || mPriori > 1.0)
      throw std::invalid_argument("Unable to procees loading. The a priori probability of scene object '" + mDescription + "' is invalid (value: " + boost::lexical_cast<std::string>(mPriori) + ").");
    
    // Create a new scene object content of the given type.
    if(!std::strcmp(mType.c_str(), "ocm")) {
      
      // Create OCM based object content.
      mContent.reset(new OcmSceneObjectContent());
    } else {
      // Warn the user that the specified probabilistic model doesn't exist!
      throw std::invalid_argument("Unable to procees loading. The probabilistic model '" + mType + "' is unknown.");
    }
    
    // Load scenen content.
    mContent->load(pPt);
  }
  
  void SceneObjectDescription::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for primary scene object '" << mDescription << "'.");
    
    // Create a new coordinator for seondary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticPrimarySceneObjectVisualization(mDescription));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
    
    // Forward visualizer to scene object content.
    if(mContent)
      mContent->initializeVisualizer(mVisualizer);
  }

  void SceneObjectDescription::update(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)

  {
    // Debug message.
    ROS_INFO_STREAM("> Evaluating primary scene object '" << mDescription << "'.");
    
    // Get the start time.
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    
    // Forward evidence to the scene object content.
    if(mContent)
      mContent->update(pEvidenceList);
    
    // Get the stop time and dump the results to file.
    std::chrono::duration<float> diff = std::chrono::high_resolution_clock::now() - start;
    if(pRuntimeLogger.is_open())
    {
      pRuntimeLogger << mDescription << "," << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << "\n";
      pRuntimeLogger.flush();
    } else {
      ROS_INFO_STREAM("Evaluating scene object'" << mDescription << "' took " << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << " milliseconds.");
    }
  }
  
  double SceneObjectDescription::getSceneObjectProbability()
  {
    double result = 0.0;
    
    if(mContent)
      result = mContent->getSceneObjectProbability();
    
    return result;
  }
  
  double SceneObjectDescription::getSceneObjectPriori()
  {
    return mPriori;
  }
  
  std::string SceneObjectDescription::getDescription()
  {
    return mDescription;
  }
  
  void SceneObjectDescription::setBestStatus(bool pStatus)
  {
    if(mContent)
      mContent->setBestStatus(pStatus);
  }
  
}
