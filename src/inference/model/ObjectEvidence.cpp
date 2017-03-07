/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/ObjectEvidence.h"

namespace ProbabilisticSceneRecognition {
 
  ObjectEvidence::ObjectEvidence()
  {
    ros::NodeHandle mNodeHandle("~");
    
    // Try to get the timeout for object evidences.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/evidence_timeout", mTimeout))
      throw std::runtime_error("Please specify parameter " + std::string("evidence_timeout") + " when starting this node.");
  }
  
  ObjectEvidence::~ObjectEvidence()
  {
  }

  void ObjectEvidence::push(const boost::shared_ptr<const asr_msgs::AsrObject>& pObject)
  {
    mBuffer.push_back(*pObject);
  }

  bool ObjectEvidence::hasWaitingEvidences()
  {
    return mBuffer.size() > 0;
  }
  
  void ObjectEvidence::update()
  {
    // Iterate over all accumulated evidences and add each one to the list.
    // Also keep book about if a new object has been found.
    BOOST_FOREACH(asr_msgs::AsrObject object, mBuffer)
    {
      // Create iterator for the outer index (object type).
      std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
      
      // Exists an entry with the given object type?
      if((it = mObjectEvidences.find(object.type)) != mObjectEvidences.end())
      {
	// Create iterator for the inner index (object instance).
	std::map<std::string, KalmanFilter>::iterator it2;
	
	// Exists an entry with the given object instance name?
	if((it2 = it->second.find(object.identifier)) != it->second.end())
	{
	  // There exists an entry for type and instance, so we update the associated kalman filter.
	  it2->second.update(object);
	  
	  // Status information for the user.
	  ROS_DEBUG_STREAM("Object Evidence: replaced object (" << object.type << ", " << object.identifier << ").");
	} else {
	  
	  // Create a new kalman filter.
	  it->second.insert(std::pair<std::string, KalmanFilter>(object.identifier, KalmanFilter(object)));
	  
	  // Status information for the user.
	  ROS_DEBUG_STREAM("Object Evidence: object with new identifier found (" << object.type << ", " << object.identifier << ").");
	}
      } else {
	// There was no entry for the given object type and instance.
	// So we first add a map for the instance to the evidences and then an entry to this map.
	std::map<std::string, KalmanFilter> entry;
	entry.insert(std::pair<std::string, KalmanFilter>(object.identifier, KalmanFilter(object)));
	mObjectEvidences.insert(std::pair<std::string, std::map<std::string, KalmanFilter> >(object.type, entry));
	// THIS IS WHY MANY PEOPLE PREFER JAVA...! OR PYTHON ;D!
	
	// Status information for the user.
	ROS_DEBUG_STREAM("Object Evidence: object with new type and identifier found (" << object.type << ", " << object.identifier << ").");
      }
    }
    
    // Search for timed out evidences.
    std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
    for(it = mObjectEvidences.begin(); it != mObjectEvidences.end(); ++it)
    {
      
      // Create iterator for the inner index (object instance).
      std::map<std::string, KalmanFilter>::iterator it2 = it->second.begin();
      if(it2 != it->second.end())
      {
	// Remove timed out evidence.
	if (it2->second.isTimedOut(mTimeout))
	{
      asr_msgs::AsrObject object = it2->second.getObject();
	  
	  it->second.erase(it2++);
	  ROS_DEBUG_STREAM("Removed timed out evidence (" << object.type << ", " << object.identifier << ").");
	} else {
	  ++it2;
	}
      }
    }
    
    // Delete all entries in the buffer.
    mBuffer.clear();
  }
  
  void ObjectEvidence::getEvidences(std::vector<asr_msgs::AsrObject>& pEvidences)
  {    
    // Erase old evidences from history *dramatic drum roll*.
    pEvidences.clear();
    
    // Create iterators for object type and instance indices.
    std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
    std::map<std::string, KalmanFilter>::iterator it2;
    
    // Iterate over the object type index (the outer index).
    for(it = mObjectEvidences.begin(); it != mObjectEvidences.end(); it++)
    {
      // Iterate over the object instance names (the inner index).
      for(it2 = it->second.begin(); it2 != it->second.end(); it2++)
      {
	// Add every evidence to the vector.
	pEvidences.push_back(it2->second.getObject());
      }
    }
  }
  
}
