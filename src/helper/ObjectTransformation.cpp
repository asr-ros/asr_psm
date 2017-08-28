/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/ObjectTransformation.h"

namespace ProbabilisticSceneRecognition {
 
  ObjectTransformation::ObjectTransformation()
  : mTfListener(ros::Duration(5184000.0))
  {
  }
  
  ObjectTransformation::~ObjectTransformation()
  {
  }
  
  void ObjectTransformation::setBaseFrame(std::string pBaseFrame)
  {
    mBaseFrame = pBaseFrame;
  }

  void ObjectTransformation::transform(const boost::shared_ptr<asr_msgs::AsrObject>& pObject)
  {
      if (mBaseFrame == pObject->header.frame_id) return;   // shortcut when both frames are equal.

    // Create everything required for the transformation
    geometry_msgs::PoseStamped input, output;
    input.header = pObject->header;

    if(!pObject->sampledPoses.size()){
      std::cerr << "Got a AsrObject without poses." << std::endl;
      std::exit(1);    
    }
    input.pose = pObject->sampledPoses.front().pose;
    
    // If no transformation from source to target frame possible, drop object.
    if(!mTfListener.waitForTransform(mBaseFrame, input.header.frame_id, pObject->header.stamp, ros::Duration(1.0)))
      throw std::runtime_error("Unable to resolve transformation in target coordinate frame.");
    
    // Do the transformation.
    mTfListener.transformPose(mBaseFrame, input, output);
    
    // Write the results back.
    pObject->header = output.header;

    pObject->sampledPoses.pop_back();
    geometry_msgs::PoseWithCovariance output_pose_with_c;
    output_pose_with_c.pose = output.pose;
    pObject->sampledPoses.push_back(output_pose_with_c);
  }
  
}
