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

// Global includes
#include <map>
#include <string>
#include <vector>

// Local includes
#include <ros/ros.h>
#include <boost/foreach.hpp>

// Package includes
#include <asr_msgs/AsrObject.h>

#include "inference/model/KalmanFilter.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class acts as a container for the object evidence. It states whether a given evidence is an update of an already known object or a new one. It is possible to return the evidences as a vector that can be distributed to the parts of the model that require the evicendes.
   * 
   * The container is able to forgot already seen objects if the specified threshold is exceeded.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ObjectEvidence {
  public:

    /**
     * Constructor.
     */
    ObjectEvidence();
    
    /**
     * Destructor.
     */
    ~ObjectEvidence();
    
    /**
     * Adds an object evidence to the buffer.
     * 
     * @param pObject AsrObject message containing information about the evidence.
     */
    void push(const boost::shared_ptr<const asr_msgs::AsrObject>& pObject);
    
    /**
     * Checks, if new evidence has been added since the last model update.
     * 
     * @return True, if new evidence was added and not yet integrated into the model.
     */
    bool hasWaitingEvidences();
    
    /**
     * Integrates all evidences into the list and checks whether there are new evidences found.
     * This method must be called before the evidence list can be requested.
     */
    void update();
    
    /**
     * Returns a list of all evidences found till now.
     * 
     * @param pEvidences A vector containing all evidences.
     */
    void getEvidences(std::vector<asr_msgs::AsrObject>& pEvidences);
    
  private:
    
    /**
     * Evidences older than this time in milliseconds are erased from the list.
     */
    int mTimeout;
    
    /**
     * A temporary buffer for accumulating evidences.
     */
    std::vector<asr_msgs::AsrObject> mBuffer;
    
    /**
     * A map for storing the object evidences, indexed by the object type and identifier.
     * The tuple (object type, object identifier) is an unique key for every found object.
     */
    std::map<std::string, std::map<std::string, KalmanFilter> > mObjectEvidences;
  };
}
