/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/SceneLearningEngine.h"

namespace ProbabilisticSceneRecognition {
  
  SceneLearningEngine::SceneLearningEngine(const std::string& pPbdSceneGraphTopic)
  : mPrivateNamespaceHandle("~")
  {
    // Volume of the workspace the scene takes place in.
    double workspaceVolume;
    
    // Parameters of heuristics used for hierarchical clustering.
    double staticBreakRatio, togetherRatio, maxAngleDeviation;
    
    // Try to get the name of the scene model.
    if(!mPrivateNamespaceHandle.getParam("scene_model_name", mSceneModelName))
      throw std::runtime_error("Please specify parameter " + std::string("scene_model_name") + " when starting this node.");
    
    // Try to get the type of the scene model representation.
    if(!mPrivateNamespaceHandle.getParam("scene_model_type", mSceneModelType))
      throw std::runtime_error("Please specify parameter " + std::string("scene_model_type") + " when starting this node.");
    
    // Try to get the directory the model should be stored.
    if(!mPrivateNamespaceHandle.getParam("scene_model_directory", mSceneModelDirectory))
      throw std::runtime_error("Please specify parameter " + std::string("scene_model_directory") + " when starting this node.");
    
    // Get the current date and time for the timestamp.
    std::stringstream ss;
    ss.imbue(std::locale(ss.getloc(), new boost::local_time::local_time_facet("%Y-%m-%d %H:%M:%S")));
    boost::local_time::time_zone_ptr zone_GMT1(new boost::local_time::posix_time_zone("GMT+1"));
    ss << boost::local_time::local_sec_clock::local_time(zone_GMT1);
    mDateTime = ss.str();
    
    // Try to get names of bag files with AsrSceneGraph message input, if any exist.
    if(mPrivateNamespaceHandle.getParam("bag_filenames_list", mInputBagFilenames))
    {
      // Either one string or a list of strings is accepted as input.
      if(mInputBagFilenames.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
	// When not called directly from CLI, we can used a list as input.
      } else {
	// Check whether parameter loaded is really a list of values.
	if(mInputBagFilenames.getType() != XmlRpc::XmlRpcValue::TypeArray)
	  throw std::invalid_argument("CLI option \"bag_filenames_list\" not set with an array.");
	
    // Go through all potential filenames of AsrSceneGraph rosbag files.
	for(int i = 0; i < mInputBagFilenames.size(); i++)
	{
	  // Check whether parameter list element is really a filename. 
	  if(mInputBagFilenames[i].getType() != XmlRpc::XmlRpcValue::TypeString)
	    throw std::invalid_argument("Bag file path no. " + boost::lexical_cast<std::string>(i) + "is no valid string.");
	}
      }
    }
    
    // Try to get the volume of the workspace where the scene takes place.
    if(!mPrivateNamespaceHandle.getParam("workspace_volume", workspaceVolume))
      throw std::runtime_error("Please specify the 'workspace volume' when starting this node.");
    
    // Try to get the volume of the workspace where the scene takes place.
    if(!mPrivateNamespaceHandle.getParam("static_break_ratio", staticBreakRatio))
      throw std::runtime_error("Please specify the 'static_break_ratio' when starting this node.");
    
    // Try to get the volume of the workspace where the scene takes place.
    if(!mPrivateNamespaceHandle.getParam("together_ratio", togetherRatio))
      throw std::runtime_error("Please specify the 'together_ratio' when starting this node.");
    
    // Try to get the volume of the workspace where the scene takes place.
    if(!mPrivateNamespaceHandle.getParam("max_angle_deviation", maxAngleDeviation))
      throw std::runtime_error("Please specify the 'max_angle_deviation' when starting this node.");
    
    // Try to get the name of the scene to be published.
    if(!mPrivateNamespaceHandle.getParam("base_frame_id", mBaseFrameId))
       throw std::runtime_error("Please specify parameter " + std::string("base_frame_id") + " when starting this node.");
    
    // Try to get the visualization scale factor.
    if(!mPrivateNamespaceHandle.getParam("scale_factor", mScaleFactor))
       throw std::runtime_error("Please specify parameter " + std::string("scale_factor") + " when starting this node.");
    
    // Try to get the sigma multiplicator.
    if(!mPrivateNamespaceHandle.getParam("sigma_multiplicator", mSigmaMultiplicator))
       throw std::runtime_error("Please specify parameter " + std::string("sigma_multiplicator") + " when starting this node.");
    
    // Try to get the targeting help flag.
    if(!mPrivateNamespaceHandle.getParam("intermediate_results", mIntermediateResults))
       throw std::runtime_error("Please specify parameter " + std::string("intermediate_results") + " when starting this node.");
    
    // Try to get the add timestamp to filenames flag.
    if(!mPrivateNamespaceHandle.getParam("timestamps", mAddTimestampsToFilename))
       throw std::runtime_error("Please specify parameter " + std::string("timestamps") + " when starting this node.");
    
    // Initialize the scene model.
    initializeSceneModel(workspaceVolume, staticBreakRatio, togetherRatio, maxAngleDeviation);
    
    // Subscribe to the AsrSceneGraph messages pulled from the bag file(s).
    mPbdSceneGraphListener = mGeneratorHandle.subscribe(pPbdSceneGraphTopic, 5, &SceneLearningEngine::newSceneGraphCallback, this);
  }
  
  SceneLearningEngine::~SceneLearningEngine()
  {
    
  }
  
  void SceneLearningEngine::readLearnerInputBags()
  {
    // If only one string is given to node, just use this as path to scene graphs.
    // Otherwise load a bunch of files and process input as it was one file.
    if(mInputBagFilenames.getType() == XmlRpc::XmlRpcValue::TypeString) {
      extractPbdSceneGraphsFromBag(static_cast<std::string>(mInputBagFilenames));
    } else {
      
      // Go through all paths to AsrSceneGraph rosbag files passed to ros node via cli.
      // Extract all AsrSceneGraph messages from rosbag file currently taken into account.
      for(int i = 0; i < mInputBagFilenames.size(); i++) {
	extractPbdSceneGraphsFromBag(static_cast<std::string>(mInputBagFilenames[i]));
      }
    }
  }
  
  void SceneLearningEngine::extractPbdSceneGraphsFromBag(const std::string& pPbdSceneGraphsBagPath)
  {
    ROS_INFO_STREAM("Scene model learner: Extracting AsrSceneGraph messages from rosbag file: " << pPbdSceneGraphsBagPath);

    // Check whether topic name for scene graph has been set before trying to parse rosbag files.
    if(!mPbdSceneGraphListener)
      throw std::logic_error("Cannot parse bag file with AsrSceneGraphs without knowing on which topic they were sent.");
    
    // Set topic representation. When parsing rosbag files this is required for extracting the messages which are representing scene graphs.
    rosbag::TopicQuery pbdSceneGraphIdentifier(mPbdSceneGraphListener.getTopic());

    // Create file handler for rosbag file to be read.
    rosbag::Bag pbdSceneGraphsBag;

    // Get read-only access to messages in given rosbag file, create access infrastructure.
    try {
      pbdSceneGraphsBag.open(pPbdSceneGraphsBagPath, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException& exception) {
      // ROS_ERROR does not work here.
      std::cerr << "Trying to extract AsrSceneGraph messages aborted because of: " << exception.what() << std::endl;
      // Quit this function as no data is to be processed.
      return;
    }

    // Create interface to extract only scene graph messages in given bag file from a previously defined topic.
    rosbag::View pbdSceneGraphView(pbdSceneGraphsBag, pbdSceneGraphIdentifier);

    // Check whether there is any raw data from a scene on the topic where we expect them.
    if(!pbdSceneGraphView.size())
      ROS_WARN_STREAM("No AsrSceneGraph messages exist in " << pPbdSceneGraphsBagPath << " on topic " << mPbdSceneGraphListener.getTopic() << ".");

    // Get access to all scene graphs in bag file to transfer them to parameter learner for scene model.
    for(rosbag::View::iterator sceneGraphIterator = pbdSceneGraphView.begin(); sceneGraphIterator != pbdSceneGraphView.end(); sceneGraphIterator++) {

      // Get interface compliant to AsrSceneGraph message on rosbag item currently taken into account.
      asr_msgs::AsrSceneGraph::ConstPtr currentSceneGraph = sceneGraphIterator->instantiate<asr_msgs::AsrSceneGraph>();

      // Success check for extraction.
      if(currentSceneGraph != NULL)
      // And add all object measurements in scene graph to parameter learners.
	newSceneGraphCallback(currentSceneGraph);
    }

    // Clean up.
    pbdSceneGraphsBag.close();
  }
  
  void SceneLearningEngine::newSceneGraphCallback(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph)
  {
    // Some error checking
    if(!pSceneGraph)
      throw std::invalid_argument("Cannot read from an non existing AsrSceneGraph message.");
    
    if(!mSceneModelLearner)
      throw std::logic_error("Cannot add data to non existing instance of the scene model.");

    // Status information
    ROS_INFO_STREAM("Scene model learner: Receiving AsrSceneGraph message with scene type " << pSceneGraph->identifier << ".");

    // Check whether scene graph input is conform to scenes definition.
    try {
      mSceneModelLearner->addExample(pSceneGraph);
    } catch (std::domain_error& domainError) {
      // Being here should see, we got a scene as input to learn, that is not mentioned in the definition of scenes this scene model should learn.
      std::cerr << "Warning - AsrSceneGraph ignored: " << domainError.what() << std::endl;
      // Do nothing more than ignoring this scene.
    } catch (std::exception& exception) {
      // ROS_ERROR does not work here.   
      std::cerr << exception.what() << std::endl;
      // Here something bad happened, so we abort processing.
      std::exit(1);
    }
  }

  void SceneLearningEngine::generateSceneModel()
  {
    // Some error checking.
    if(!mSceneModelLearner)
      throw std::logic_error("Cannot save a non-existing scene model and plot its distributions.");
    
    // Calculates the scene model.
    mSceneModelLearner->generateSceneModel();
    
    // Initialize the visualizer.
    initializeVisualizationChain();
  }

  void SceneLearningEngine::saveSceneModel()
  {
    // Some error checking.
    if(!mSceneModelLearner)
      throw std::logic_error("Cannot save a non-existing scene model and plot its distributions.");

    // User information.
    ROS_INFO_STREAM("Scene model learner: Storing scene model '" << mSceneModelName << "'.");
    
    // Build the path to the model file based on the directory, the scene model name and the timestamp (if requried).
    std::string pathToFile = mSceneModelDirectory + "/";
    
    if(mAddTimestampsToFilename)
      pathToFile += mDateTime + "-";
    
    pathToFile += mSceneModelName + ".xml";
    
    // Write scene model to file.
    mSceneModelLearner->saveSceneModelToFile(pathToFile);
  }
  
  void SceneLearningEngine::visualizeSceneModel()
  {
    if(mVisualizer)
    {
      if(mIntermediateResults)
	mVisualizer->drawInLearningTrajectoryMode();
      else
	mVisualizer->drawInLearningMode();
    }
  }
  
  void SceneLearningEngine::initializeSceneModel(double pWorkspaceVolume, double mStaticBreakRatio, double mTogetherRatio, double mMaxAngleDeviation)
  {
    mSceneModelLearner.reset(new SceneModelLearner(mSceneModelType, pWorkspaceVolume, mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation));
  }
  
  void SceneLearningEngine::initializeVisualizationChain()
  {
    // Status information for the user.
    ROS_INFO("Initializing visualization mechanism.");
    
    // Create a new coordinator for scene visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSceneModelVisualization());
    
    // Order the model to initialize the visualizers.
    mSceneModelLearner->initializeVisualizer(mVisualizer);
    
    // Set drawing parameters.
    mVisualizer->setDrawingParameters(mScaleFactor, mSigmaMultiplicator, mBaseFrameId);
  }
  
}
