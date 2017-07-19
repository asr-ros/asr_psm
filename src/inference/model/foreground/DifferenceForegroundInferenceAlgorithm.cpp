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
      const char *str = mDataBaseName.c_str();
      const char *result = strstr(str, ".sqlite");
      int position = result - str;
      patternName = mDataBaseName.substr(position + 7,mDataBaseName.size());
      mDataBaseName = mDataBaseName.substr(0, position + 7);
  }
  
  DifferenceForegroundInferenceAlgorithm::~DifferenceForegroundInferenceAlgorithm()
  {
  }

  void DifferenceForegroundInferenceAlgorithm::doInference(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    ROS_INFO_STREAM("Calculating scene probability.");



    // Iterate over all scene objects, evaluate them and summarize the results.
    mProbability = 0.0;

    ROS_INFO_STREAM("Pulling Objects from database " << mDataBaseName.c_str() << " for scene '" << patternName.c_str() << "'");
      tableHelper = ISM::TableHelperPtr(new ISM::TableHelper(mDataBaseName));

    for(auto recordedObject : pEvidenceList){
       std::string currentType = recordedObject.type + recordedObject.observedId;
       double maxProbability = 0.0;


          ISM::RecordedPatternPtr pattern = tableHelper->getRecordedPattern(patternName);

          for(ISM::ObjectSetPtr objectSet : pattern->objectSets)
          {
                     ISM::Object testReference;
                     for(auto object : objectSet->objects) {
                         std::string objectType = object->type + object->observedId;
                         if(objectType.compare(currentType) == 0)
                            testReference = *object;
                     }
                     double testProbability = 1.0;

                                                    //TODO: Parameter??

                     for(auto object : objectSet->objects) {
                          std::string objectType = object->type + object->observedId;

                          double innerTestProbability = 0.1;
                          if(objectType.compare(currentType) != 0){
                              ISM::Object innerRecordedObject =findObjectOfType(pEvidenceList, objectType);
                              if(innerRecordedObject.type.compare("") != 0)
                                    innerTestProbability = differenceBetween(recordedObject, innerRecordedObject, testReference, *object);
                          }

                     testProbability *= innerTestProbability;
                     }

                     if(testProbability > maxProbability)
                        maxProbability = testProbability;
            }


       if(maxProbability > mProbability) {
           mProbability = maxProbability;
       }
    }

    ROS_INFO_STREAM(" > Difference based scene probability is: '" << mProbability << "'.");
  }

  double DifferenceForegroundInferenceAlgorithm::differenceBetween(ISM::Object pRoot, ISM::Object pTar, ISM::Object pDiffRoot, ISM::Object pDiffTar) {
      Eigen::Vector3d distance = pRoot.pose->point->eigen - pTar.pose->point->eigen - (pDiffRoot.pose->point->eigen - pDiffTar.pose->point->eigen);
      Eigen::Vector3d pRootEuler = pRoot.pose->quat->eigen.toRotationMatrix().eulerAngles(2, 1, 0);
      normalizeVector3d(pRootEuler);
      Eigen::Vector3d pTarEuler = pTar.pose->quat->eigen.toRotationMatrix().eulerAngles(2, 1, 0);
      normalizeVector3d(pTarEuler);
      Eigen::Vector3d pDiffRootEuler = pDiffRoot.pose->quat->eigen.toRotationMatrix().eulerAngles(2, 1, 0);
      normalizeVector3d(pDiffRootEuler);
      Eigen::Vector3d pDiffTarEuler = pDiffTar.pose->quat->eigen.toRotationMatrix().eulerAngles(2, 1, 0);
      normalizeVector3d(pDiffTarEuler);
      Eigen::Vector3d rotationDistance = pRootEuler - pTarEuler - (pDiffRootEuler - pDiffTarEuler);
      double rotationNorm = sqrt(pow(rotationDistance.x(), 2.0) + pow(rotationDistance.y(), 2.0) + pow(rotationDistance.z(), 2.0));
      double positionNorm = sqrt(pow(distance.x(), 2.0) + pow(distance.y(), 2.0) + pow(distance.z(), 2.0));

      double result = (10.0 - rotationNorm)/10.0 * (10.0 - positionNorm)/10.0;
      if(result < 0) return 0;
      return result;                                                          // Parameter anpassen. Gewichtung? multipliziert vs addiert
  }

  ISM::Object DifferenceForegroundInferenceAlgorithm::findObjectOfType(std::vector<ISM::Object> pList, std::string pTypeAndObservedId) {
         for(ISM::Object object : pList) {
             if(pTypeAndObservedId.compare(object.type + object.observedId) == 0) {
                 return object;
             }
         }
         return *(new ISM::Object());

  }

  
  double DifferenceForegroundInferenceAlgorithm::getProbability()
  {
    return mProbability;
  }

  void DifferenceForegroundInferenceAlgorithm::normalizeVector3d(Eigen::Vector3d input)
  {
    double norm = sqrt(pow(input.x(), 2.0) + pow(input.y(), 2.0) + pow(input.z(), 2.0));
    input = input * (1 / sqrt(norm));
  }
  
}
