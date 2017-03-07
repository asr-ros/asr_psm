/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/SceneIdentifier.h"

namespace ProbabilisticSceneRecognition {
 
  SceneIdentifier::SceneIdentifier()
  {
  }
  
  SceneIdentifier::SceneIdentifier(std::string pType, std::string pDescription)
  : mType(pType)
  , mDescription(pDescription)
  {
  }
  
  SceneIdentifier::SceneIdentifier(const SceneIdentifier& pOther)
  {
    mType = pOther.mType;
    mDescription = pOther.mDescription;
    mPriori = pOther.mPriori;
    mLikelihood = pOther.mLikelihood;
  }
  
  SceneIdentifier::~SceneIdentifier()
  {
  }
  
  void SceneIdentifier::load(boost::property_tree::ptree& pPt)
  {
    mDescription = pPt.get<std::string>("<xmlattr>.name");
    mType = pPt.get<std::string>("<xmlattr>.type");
    mPriori = pPt.get<double>("<xmlattr>.priori");
  }
  
  void SceneIdentifier::save(boost::property_tree::ptree& pPt)
  {
    pPt.add("<xmlattr>.name", mDescription);
    pPt.add("<xmlattr>.type", mType);
    pPt.add("<xmlattr>.priori", mPriori);
  }
    
}
