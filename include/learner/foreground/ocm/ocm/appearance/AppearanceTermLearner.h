/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

#include <trainer/source/Object.h>

// Local includes
#include "learner/foreground/ocm/ocm/OcmTree.h"
#include "learner/foreground/ocm/ocm/OcmModel.h"
#include "learner/foreground/ocm/ocm/TermLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for the appearance term of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AppearanceTermLearner : public TermLearner {
  public:
    
    /**
     * Constructor.
     */
    AppearanceTermLearner();
    
    /**
     * Destructor.
     */
    ~AppearanceTermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
    void learn(boost::shared_ptr<OcmModel> pModel);
    
  private:
    
    /**
     * Learns the mapping for the given node.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     * @param pNode The node to learn the mapping for.
     */
    void learnMapping(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode);
    
    /**
     * Learns the probability table for the given node.
     * 
     * @param pModel The OcmTree model that provides raw data and containers for the parameters to learn.
     * @param pNode The node to learn the probability table for.
     * @param pSlot Maps the given node to a row in the probability table.
     */
    void learnTable(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode, unsigned int& pSlot);
  };
}
