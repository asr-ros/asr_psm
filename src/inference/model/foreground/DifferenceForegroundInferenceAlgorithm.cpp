/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/DifferenceForegroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
 
  DifferenceForegroundInferenceAlgorithm::DifferenceForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects, std::string pDataBaseName)
  : ForegroundInferenceAlgorithm(pSceneObjects)
  , mProbability(0.0)
  {
      mDataBaseName = pDataBaseName;
  }
  
  DifferenceForegroundInferenceAlgorithm::~DifferenceForegroundInferenceAlgorithm()
  {
  }

  void DifferenceForegroundInferenceAlgorithm::doInference(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    ROS_INFO_STREAM("Calculating scene probability.");
    
    // Iterate over all scene objects, evaluate them and summarize the results.
    mProbability = 0.0;

    ROS_INFO_STREAM("Pulling Objects from database " << mDataBaseName.c_str());
      tableHelper = ISM::TableHelperPtr(new ISM::TableHelper(mDataBaseName));

      for(auto patternName : tableHelper->getRecordedPatternNames())
      {
          ISM::RecordedPatternPtr pattern = tableHelper->getRecordedPattern(patternName);
          for(ISM::ObjectSetPtr objectSet : pattern->objectSets)
          {
              ROS_INFO_STREAM("blabla" << pattern->name);
        //      objectSet->mIdentifier = pattern->name;
     //         mSceneModelLearner->addExample(objectSet);
          }
      }




    /*for(boost::shared_ptr<SceneObjectDescription> sceneObject : *mSceneObjects)
    {
      // Update the scene objects with the new evidence.
      sceneObject->update(pEvidenceList, pRuntimeLogger);
      
      mProbability = std::max(mProbability, sceneObject->getSceneObjectProbability() * sceneObject->getSceneObjectPriori());
    }
    
    // Iterate over all scene objects and mark the best one.
    bool done = false;
    for(boost::shared_ptr<SceneObjectDescription> sceneObject : *mSceneObjects)
    {
      if(sceneObject->getSceneObjectProbability() * sceneObject->getSceneObjectPriori() == mProbability && !done)
      {
	sceneObject->setBestStatus(true);
	done = true;
      } else
	sceneObject->setBestStatus(false);
    }*/
    ROS_INFO_STREAM(" > Difference based scene probability is: '" << mProbability << "'.");
  }
  
  double DifferenceForegroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }
  
}
