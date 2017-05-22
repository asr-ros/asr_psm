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

#include <visualization/psm/ProbabilisticSceneVisualization.h>

#include <trainer/PSMTrainer.h>

//local includes
#include <ISM/common_type/ObjectSet.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * An abstract learner for a scene object.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneObjectLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pSceneObjectType The type of the scene object.
     */
    SceneObjectLearner(std::string pSceneObjectType);
    
    /**
     * Destructor.
     */
    virtual ~SceneObjectLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior) = 0;
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    virtual void save(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Learns the scene object.
     * 
     * @param mExamplesList A list of all examples for the scene this scene object belongs to.
     */
    virtual void learn(std::vector<ISM::ObjectSetPtr> pExamplesList, boost::shared_ptr<SceneModel::TreeNode> tree) = 0;
   
    /**
      * Checks, if this scene object has the given type.
      * 
      * @param pSceneObjectType The type to check for.
      */
    bool hasType(std::string pSceneObjectType);
    
    /**
     * Parameters for the clustering algorithm.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    void setClusteringParameters(double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation);
    
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
     * The type of the scene object. Because we don't use any instance information this is also the identity of the scene object.
     */
    std::string mSceneObjectType;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * Parameters of heuristics used for hierarchical clustering.
     */
    double mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation;
  };
}
