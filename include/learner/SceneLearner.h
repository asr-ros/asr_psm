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
#include <string>
#include <vector>

// Package includes
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

//local includes
#include <ISM/common_type/ObjectSet.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract learner for a single scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pSceneName The scene of the scene.
     */
    SceneLearner(std::string pSceneName);
    
    /**
     * Constructor.
     *
     */
    SceneLearner();

    /**
     * Destructor.
     */
    virtual ~SceneLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior) = 0;
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    virtual void save(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Learns the scene.
     */
    virtual void learn() = 0;
    
    /**
     * Checks, if the given ISM::ObjectSet contains an example for this scene.
     * 
     * @return True, if the set contains an example for this scene.
     */
    bool isExampleForScene(const ISM::ObjectSetPtr pExample);
    
    /**
     * Adds an ISM::ObjectSet to the learner.
     * 
     * @param pExample ISM::ObjectSet containing an example for the given scene.
     */
    void addExampleToScene(const ISM::ObjectSetPtr pExample);

    /**
     * Sets the volume of the workspace.
     * 
     * @param pValue The volume of the workspace.
     */
    void setVolumeOfWorkspace(double pValue);
    
    /**
     * Sets the a priori probability of the scene.
     * 
     * @param pPriori The a priori probability of the scene.
     */
    void setPriori(double pPriori);
    
  protected:
    
    /**
     * The a priori probability for this scene.
     * Will be set to an equal distribution for all scenes.
     */
    double mPriori;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * The name of the scene. It is required for filtering ISM::ObjetSets and the export to file.
     */
    std::string mSceneName;
    
    /**
     * A list of all examples provided for this scene.
     */
    std::vector<ISM::ObjectSetPtr> mExamplesList;
  };
}
