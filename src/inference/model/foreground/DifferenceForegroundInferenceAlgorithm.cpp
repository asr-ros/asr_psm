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
          double maxProbability = 0.0;
          ISM::RecordedPatternPtr pattern = tableHelper->getRecordedPattern(patternName);
          for(ISM::ObjectSetPtr objectSet : pattern->objectSets)
          {              // - -- - -- - - - -- - - -- - - -- - - - - - -- - - - object set vorher nach typ sortieren!
             for(auto object : objectSet->objects) {
                 ROS_INFO_STREAM("Current Object to be checked is " << object->type << object->observedId);
                 std::string objectType = object->type + object->observedId;
                 std::string currentType = "";
                 double testProbability = 1.0;
                 double innerMaxProbability = 0.0;
                 ISM::Object currentReferenceObject = findObjectOfType(pEvidenceList, objectType);
                 ISM::Object currentInnerReferenceObject;

                 for(auto innerObject : objectSet->objects) {
                     std::string innerType = innerObject->type + innerObject->observedId;
                     if(objectType.compare(innerType) != 0)
                     {
                        if(currentType.compare("") == 0){
                            currentType = innerType;
                            currentInnerReferenceObject = findObjectOfType(pEvidenceList, currentType);
                            }
                        else if(currentType.compare(innerType) != 0) {
                            currentType = innerType;
                            if(innerMaxProbability != 0.0)
                                testProbability *= innerMaxProbability;
                            innerMaxProbability = 0.0;
                            currentInnerReferenceObject = findObjectOfType(pEvidenceList, currentType);
                        }
                        double innerTestProbability = differenceBetween(currentReferenceObject, currentInnerReferenceObject, *object, *innerObject);
                        if(innerTestProbability > innerMaxProbability)
                            innerMaxProbability = innerTestProbability;
                     }
                 }

                 if(testProbability > maxProbability)
                     maxProbability = testProbability;
             }
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
      return (100.0 - rotationNorm)/100.0 * (100.0 - positionNorm)/100.0;                                       // Parameter anpassen.
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
