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
#include <queue>

// Package includes
#include <ros/ros.h>
#include <rosbag/view.h>

#include <asr_msgs/AsrObject.h>

#include <visualization/gnuplot/GnuplotVisualization.h>
#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "helper/ObjectTransformation.h"

#include "inference/model/SceneIdentifier.h"
#include "inference/model/SceneModelDescription.h"

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Probabilistic scene inference engine in form of a ROS node. This class is basically a ROS wrapper for the inference model. Engine parameters are loaded via node handle, the model is loaded from file.
   *
   */
  class SceneInferenceEngine {
  public:

    /**
     * Constructor.
     */
    SceneInferenceEngine();

    /**
     * Destructor.
     */
    ~SceneInferenceEngine();
    
    /**
     * Updates the inference engine.
     */
    void update();
    
    /**
     * Runs a single update and then terminates.
     */
    void executeInStackMode(); 
    
   /**
     * Collects evidences in form of AsrObject and forwards them to the inference model.
     *
     * @param pObject An observation result for a potential scene element coming from an arbitrary sensor data processing system.
     */
    void newObservationCallback(const boost::shared_ptr<asr_msgs::AsrObject>& pObject);
    
   /**
     * Returns the model.
     * The model is responsible for loading the scene model from file, collect and manage the evidence and do the inference.
     */
    SceneModelDescription getModel() {return mModel;}

	/**
	* Bool to decide if difference based inference shall be used
	*/
	bool difference_based;


    /**
    * The path to the database which contains the samples
    */
    std::string dataBaseName;
    
  private:
    
    /**
     * Loads the probabilistic scene model from XMl file.
     *
     * @param pSceneModelFileName The name of the XMl file containing the scene model that should we used.
     * @param pInferenceAlgorithm The name of the inference algorithm.
     */
    void loadSceneModel(const std::string pSceneModelFileName, const std::string pInferenceAlgorithm);
    
    /**
     * Initializes the chain responsible for visualization.
     * 
     * @param pScale Factor to multiply the kernel with.
     * @param pSigmaMultiplicator Scaling factor for the size of the visualized covariance ellipsoid.
     * @param pFrameId The name of the coordinate frame that should be drawn into.
     */
    void initializeVisualizationChain(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId);

    /**
     * Converts an ISMObject to an AsrObject
     *
     * @param pObject the Object to convert
     */
    boost::shared_ptr<asr_msgs::AsrObject> ISMObjectToAsrObject(boost::shared_ptr<ISM::Object> pObject);

    /**
     * Converts an AsrObject to ISMObject
     *
     * @param pObject the Object to convert
     */
    boost::shared_ptr<ISM::Object> AsrObjectToISMObject(boost::shared_ptr<asr_msgs::AsrObject> pObject);
    
  private:
    
    /**
     * True to show the plot of the scene probabilities.
     */
    bool showPlot;
    
    /**
     * Set true to overwrite the visualization of results and plot the target distributions instead.
     */
    bool mTargetingHelp;
    
    /**
     * Interface to private ros node namespace.
     */
    ros::NodeHandle mNodeHandle;
    
    /**
     * A callback handler listening to objects found by an object detection system.
     */
    ros::Subscriber mObjectListener;
    
    /**
     * A callback handler listening to preprocessed observations that describe the objects in a scene over time
     */
    ros::Subscriber mSceneGraphListener;
    
    /**
     * A buffer for storing evidences.
     */
    std::queue<boost::shared_ptr<asr_msgs::AsrObject> > mEvidenceBuffer;
    
    /**
     * A transformer for objects into the target coordinate frame.
     */
    ObjectTransformation mObjectTransform;
    
    /**
     * The model is responsible for loading the scene model from file, collect and manage the evidence and do the inference.
     */
    SceneModelDescription mModel;
    
    /**
     * Gnuplot visualizer for drawing bar diagrams.
     */
    Visualization::GnuplotVisualization mVisGnuplot;
    
    /**
     * Class for coordinating the scene visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mVisualizer;
  };
}
