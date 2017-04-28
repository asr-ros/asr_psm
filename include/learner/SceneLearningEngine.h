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

// Package includes
#include <ros/ros.h>
#include <rosbag/view.h>

#include <boost/shared_ptr.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <asr_msgs/AsrSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "learner/SceneModelLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Class for learning the model required for probabilistic scene recognition.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneLearningEngine {
  public:

    /**
     * Constructor.
     * 
     * @param pPbdSceneGraphTopic Name of the topic AsrSceneGraph messages were published.
     */
    SceneLearningEngine(const std::string& pPbdSceneGraphTopic);

    /**
     * Destructor.
     */
    ~SceneLearningEngine();

    /**
     * Extract AsrSceneGraph messages from all rosbag files given as CLI parameters.
     * Transfer all measurements for each scene into distribution parameter learners.
     */
    void readLearnerInputBags();
    
    /**
     * Open rosbag file and extract AsrSceneGraph messages on input topic (which has been set before).
     * All scene graph that the rosbag file contains, are transfered to the distribution parameter learners.
     * 
     * @param pPbdSceneGraphsBagPath Path of file to be parsed for AsrSceneGraph messages.
     */
    void extractPbdSceneGraphsFromBag(const std::string& pPbdSceneGraphsBagPath);
    
    /**
     * Adds all recorded scene data in a AsrSceneGraph to a system that learns parameters of distributions in the decomposition of the joint distribution in a scene model.
     *
     * @param pSceneGraph Features of all objects in a scene localized over a certain period of time.
     */
    void newSceneGraphCallback(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);

    /**
     * All learnt information is transfered into a scene model description. 
     */
    void generateSceneModel();

    /**
     * Learnt scene model is written to an xml file whose name consists of a scene model identifier set before and a timestamp.
     */
    void saveSceneModel();
    
    /**
     * All distributions in the decomposition of the scene model and the learning data are plotted.
     */
    void visualizeSceneModel();
    
  private:
    
    /**
     * Initializes the scene model with the parameters specified via the node handle.
     * 
     * @param pWorkspaceVolume Volume of the workspace the scene takes place in.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    void initializeSceneModel(double pWorkspaceVolume, double mStaticBreakRatio, double mTogetherRatio, double mMaxAngleDeviation);
    
    /**
     * Initializes the chain responsible for visualization.
     */
    void initializeVisualizationChain();
    
    /************************************
     * ROS
     ************************************/
    
    /**
     * Interface to basic ros node functionality enhanced by the given class.
     */
    ros::NodeHandle mGeneratorHandle;
    
    /**
     * Interface to the private namespace of the ros node.
     */
    ros::NodeHandle mPrivateNamespaceHandle;
        
    /**
     * Listener for AsrSceneGraph messages.
     */
    ros::Subscriber mPbdSceneGraphListener;
    
    /**
     * A list of paths to rosbag files containing AsrSceneGraph messages.
     * The latter can be used for model parameter learning.
     */
    XmlRpc::XmlRpcValue mInputBagFilenames;
        
    /************************************
     * Scene model
     ************************************/
    
    /**
     * Set true to visualize intermediate results instead.
     */
    bool mIntermediateResults;
    
    /**
     * Set true to add timestamps to filename.
     */
    bool mAddTimestampsToFilename;
    
    /**
     * The coordinate frame in which the visualization should take place.
     */
    std::string mBaseFrameId;
    
    /**
     * The visualization is pretty small, this scale factor enlarges it.
     */
    double mScaleFactor;
    
    /**
     * This factor determines the radii of the covariance ellipse.
     */
    double mSigmaMultiplicator;
    
    /**
     * The timestamp for the point in time the learner was started.
     */
    std::string mDateTime;
    
    /**
     * Name of the scene model.
     */
    std::string mSceneModelName;
    
    /**
     * Determines the type of representation of the scene model that should be learned.
     */
    std::string mSceneModelType;
    
    /**
     * Path to the directory the XML file containing the model is stored after learning.
     */
    std::string mSceneModelDirectory;
    
    /**
     * The learner for the probabilistic scene model.
     */
    boost::shared_ptr<SceneModelLearner> mSceneModelLearner;
    
    /**
     * Class for coordinating the scene visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mVisualizer;
  };
}
