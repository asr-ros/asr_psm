/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/OcmForegroundSceneLearner.h"

namespace ProbabilisticSceneRecognition {

  OcmForegroundSceneLearner::OcmForegroundSceneLearner(const ISM::ObjectSetPtr pExample)
  : ForegroundSceneLearner(pExample)
  {
  }
  
  OcmForegroundSceneLearner::~OcmForegroundSceneLearner()
  {
  }
  
  void OcmForegroundSceneLearner::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for scene '" << mSceneName << "'.");
    
    // Create a new coordinator for primary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSceneVisualization(mSceneName));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
    
    // Command scene object learners to initialize their visualizers.
    BOOST_FOREACH(boost::shared_ptr<SceneObjectLearner> learner, mSceneObjectLearners)
      learner->initializeVisualizer(mVisualizer);
  }
  
  void OcmForegroundSceneLearner::save(boost::property_tree::ptree& pPt)
  {
    // Create a subtree.
    boost::property_tree::ptree subtree;
    
    // Add scene name and type.
    subtree.add("<xmlattr>.type", "ocm");
    subtree.add("<xmlattr>.name", mSceneName);
    subtree.add("<xmlattr>.priori", mPriori);
    
    // Command scene object learners to save their content.
    BOOST_FOREACH(boost::shared_ptr<SceneObjectLearner> learner, mSceneObjectLearners)
      learner->save(subtree);
    
    // Add subtree to main tree.
    pPt.add_child("psm.scenes.scene", subtree);
  }
  
  void OcmForegroundSceneLearner::learn()
  {
    /*****************************************************************************************************************
     * What we do here:
     * We create a scene object learner for every unique object in the scene. Objects ARE ONLY IDENTIFIED BY THEIR TYPE,
     * NOT THEIR INSTANCE. That is because at the time of implementation, no instance information was available!
     *****************************************************************************************************************/
    std::vector<std::string> types;
    // First we iterate over all examples for this scene.

      ISM::TracksPtr alltracks(new ISM::Tracks(mExamplesList));
      // Iterate over all examples for all scenes.
      for (auto &track : alltracks->tracks){
	// Get the type of the first observation (we assume here that all obserations are of the same type).
    std::string type = track->type;
	
	// Evalute if there already exists an object with the given type.
	bool isExisting = false;
	BOOST_FOREACH(boost::shared_ptr<SceneObjectLearner> learner, mSceneObjectLearners)
	  isExisting |= learner->hasType(type);
	
	// If no learner was found, create a new one.
	if(!isExisting) {
      mSceneObjectLearners.push_back(boost::shared_ptr<SceneObjectLearner>(new OcmSceneObjectLearner(type, mSceneName)));
      types.push_back(type);
	  ROS_INFO_STREAM("Found a new scene object of type '" << type << "' in scene '" << mSceneName << "'.");
	}

    }
    
    ROS_INFO("Building relation tree.");

    ros::NodeHandle nodeHandle("~");
    std::string trainerType;
    // Try to get the type of the relation tree trainer.
    if(!nodeHandle.getParam("relation_tree_trainer_type", trainerType))
    {
       ROS_INFO_STREAM("Could not find ROS parameter relation_tree_trainer_type. Using standard method of hierarchical clustering (value=\"tree\").");
       trainerType = "tree";     // for compatability with old launch files.
    }

    // Create the relation graph.
    SceneModel::AbstractTrainer trainer;
    if (trainerType == "tree")      // Hierarchical tree, like before, without references.
    {
        SceneModel::PSMTrainer psmtrainer(mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation);
        psmtrainer.addSceneGraphMessages(mExamplesList);
        trainer = psmtrainer;
    }
    else if (trainerType == "fullymeshed")     // Fully meshed topology -> tree with references.
    {
        SceneModel::FullyMeshedTrainer fmtrainer;
        fmtrainer.addSceneGraphMessages(mExamplesList);
        trainer = fmtrainer;
    }
    else if (trainerType == "combinatorial_optimization")     // combinatorial optimization
    {
        // Preparing learners for use in combinatorial optimization, compare below:
        for (boost::shared_ptr<SceneObjectLearner> learner: mSceneObjectLearners)
        {
            learner->setClusteringParameters(mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation);
            learner->setVolumeOfWorkspace(mWorkspaceVolume);
            learner->setPriori(1.0);
        }

        CombinatorialTrainer combinatorialTrainer(mSceneObjectLearners, types);
        combinatorialTrainer.addSceneGraphMessages(mExamplesList);
        trainer = combinatorialTrainer;
    }
    else throw std::runtime_error("Trainer type " + trainerType + " is invalid. Valid types are tree, fullymeshed, combinatorial_optimization.");

    //trainer.addSceneGraphMessages(mExamplesList);
    trainer.loadTrajectoriesAndBuildTree();

    // Print tree.
    ROS_INFO("PSM trainer successfully generated tree.");
    std::cout << "------------- TREE:" << std::endl;
    trainer.getTree()->printTreeToConsole(0);
    std::cout << "---------------------" << std::endl;

    // Now just forward all examples for the scene to the scene object learners.
    BOOST_FOREACH(boost::shared_ptr<SceneObjectLearner> learner, mSceneObjectLearners)
    {
        learner->setClusteringParameters(mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation);
        learner->setVolumeOfWorkspace(mWorkspaceVolume);
        learner->learn(mExamplesList, trainer.getTree());
    }

    // DEPRECATED could be removed!
    // Same problem as in the occurence learner. We don't have tracking, so we don't know anything about the frequency of an object appearing.
    // We assume an equal distribution over all objects.
    BOOST_FOREACH(boost::shared_ptr<SceneObjectLearner> learner, mSceneObjectLearners)
            learner->setPriori(1.0);

  }
  
}
