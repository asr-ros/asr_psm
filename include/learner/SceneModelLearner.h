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
#include <vector>

// Package includes
#include <ros/ros.h>
#include <ros/console.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <asr_msgs/AsrSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "learner/SceneLearner.h"

#include "learner/background/BackgroundSceneLearner.h"

#include "learner/foreground/ForegroundSceneLearner.h"

#include "learner/foreground/ocm/OcmForegroundSceneLearner.h"

#include <ISM/common_type/ObjectSet.hpp>
namespace ProbabilisticSceneRecognition {
  
  /**
   * Defines the type for the OCM based foreground scene model.
   */
  static const std::string OCM_SCENE_MODEL = "ocm";
  
  /**
   * Learner for the probabilistic scene model.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneModelLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pForegroundSceneModel The type of model that should be used for foreground scene representation.
     * @param pWorkspaceVolume The volume of the workspace in cubic meters.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    SceneModelLearner(std::string pForegroundSceneModel, double pWorkspaceVolume, double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation);
    
    /**
     * Destructor.
     */
    ~SceneModelLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Adds a AsrSceneGraph message to the learner.
     * 
     * @param pExample AsrSceneGraph message containing an example for a scene.
     */
    void addExample(const ISM::ObjectSetPtr pExample);
    
    /**
     * Calculates the model parameters based on the collected evidence.
     */
    void generateSceneModel();
    
    /**
     * Saves the model to an XML file.
     * 
     * @param pPathToFile The CMl file the scene model should be written to.
     */
    void saveSceneModelToFile(std::string pPathToFile);
    
  private:
    
    /**
     * The type of model that should be used for foreground scene representation.
     */
    std::string mForegroundSceneModel;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * Parameters of heuristics used for hierarchical clustering.
     */
    double mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation;
    
    /**
     * A seperate learner for the background model.
     */
    BackgroundSceneLearner mBackgroundSceneLearner;
    
    /**
     * A list of all foreground scene learners. If an AsrSceneGraph message could not be associated with a learner in the list, a new scene learner is automatically added.
     */
    std::vector<boost::shared_ptr<ForegroundSceneLearner> > mSceneLearners;
  };
}
