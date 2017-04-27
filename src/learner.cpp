/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// Global includes
#include <cstdlib>

// Package includes
#include <ros/ros.h>

#include <learner/pbd_computation_graph.hpp>

// Local includes
#include "learner/SceneLearningEngine.h"


using namespace ProbabilisticSceneRecognition;


int main(int argc, char* argv[])
{

  ros::init(argc, argv, "js_probabilistic_scene_learner");
  
  // Declaration of the learner for the scene model.
  SceneLearningEngine* learner;

  // Check for wrong configurations and similar.
  try {
    learner = new SceneLearningEngine(pbd_msgs::Topics::SCENE_GRAPHS);

  } catch(std::exception& exception){
    std::cerr << exception.what() << std::endl;
    std::exit(1);
  }

  // Get all PbdSceneGraphs from rosbag files passed as ros parameters before trying to get other messages from the listeners of the node.
  learner->readLearnerInputBags();

  // Check for errors with the resulting scene model
  try {       
    // Calculate parameters of scene model based on scene graphs from rosbag input.
    learner->generateSceneModel();
    
    // Dump model to file and plot its distributions.
    learner->saveSceneModel();
  } catch(std::exception& exception) {
    std::cerr << exception.what() << std::endl;
  }
  
  // Specifies the updates per second.
  ros::Rate rate(30);
  
  // Run main loop until termination.
  while(ros::ok())
  {
    // Visualize the model.
    learner->visualizeSceneModel();
    
    // Sleep for the given time in seconds.
    rate.sleep();
  }
  
  // Get rid of learner object.
  delete learner;

  // Stating that program has run correctly.
  return EXIT_SUCCESS;
}

