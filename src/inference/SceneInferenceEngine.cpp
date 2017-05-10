/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/SceneInferenceEngine.h"

namespace ProbabilisticSceneRecognition {
  
  SceneInferenceEngine::SceneInferenceEngine()
  : mNodeHandle("~")
  {
    // The ROS topic to listen to.
    std::string pPbdObjectTopic;
    
    // The ROS topic for scene graph messages to listen to.
    std::string pPbdSceneGraphTopic;
    
    // Name of the XML file containing the scnene model.
    std::string sceneModelFileName;
    
    // The name of the algorithm that should be used for the inference.
    std::string inferenceAlgorithm;
    
    // A list of paths to rosbag files containing AsrSceneGraph messages.
    XmlRpc::XmlRpcValue mInputBagFilenames;
    
    // The frame to transform the object poses to.
    // Also the coordinate frame in which the visualization should take place.
    std::string baseFrameId;
    
    // The visualization is pretty small, this scale factor enlarges it.
    double scaleFactor;
    
    // This factor determines the radii of the covariance ellipse.
    double sigmaMultiplicator;
    
    // Try to get the clearance for plotting the scene probabilties.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/plot", showPlot))
      throw std::runtime_error("Please specify parameter " + std::string("plot") + " when starting this node.");
    
    // Try to get the name of the topic to listen to for new evidences.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/object_topic", pPbdObjectTopic))
      throw std::runtime_error("Please specify parameter " + std::string("object_topic") + " when starting this node.");
    
    // Try to get the name of the topic to listen to for scene graph messages.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/scene_graph_topic", pPbdSceneGraphTopic))
      throw std::runtime_error("Please specify parameter " + std::string("scene_graph_topic") + " when starting this node.");
    
    // Try to get the file name of the XML file from which the scene model should be read.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/scene_model_filename", sceneModelFileName))
      throw std::runtime_error("Please specify parameter " + std::string("scene_model_filename") + " when starting this node.");
    
    // Try to get names of bag files with AsrSceneGraph message input, if any exist.
    if(mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/bag_filenames_list", mInputBagFilenames))
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

    // Try to get the name of the scene to be published.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/base_frame_id", baseFrameId))
       throw std::runtime_error("Please specify parameter " + std::string("base_frame_id") + " when starting this node.");
    
    // Try to get the visualization scale factor.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/scale_factor", scaleFactor))
       throw std::runtime_error("Please specify parameter " + std::string("scale_factor") + " when starting this node.");
    
    // Try to get the sigma multiplicator.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/sigma_multiplicator", sigmaMultiplicator))
       throw std::runtime_error("Please specify parameter " + std::string("sigma_multiplicator") + " when starting this node.");
    
    // Try to get the targeting help flag.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/targeting_help", mTargetingHelp))
       throw std::runtime_error("Please specify parameter " + std::string("targeting_help") + " when starting this node.");
    
    // Try to get the name of the inference algorithm.
    if(!mNodeHandle.getParam("/js_probabilistic_scene_inference_engine/inference_algorithm", inferenceAlgorithm))
       throw std::runtime_error("Please specify parameter " + std::string("inference_algorithm") + " when starting this node.");
    
    // Initialize the transformations of objects into the given frame.
    mObjectTransform.setBaseFrame(baseFrameId);
    
    // Initialize the scene model with the parameters given on startup of this node.
    loadSceneModel(sceneModelFileName, inferenceAlgorithm);
    
    // Initialize the visualization chain.
    initializeVisualizationChain(scaleFactor, sigmaMultiplicator, baseFrameId);
    
    // Tell node how to react on messages from objects that could belong to scenes being looked for.
    mObjectListener = mNodeHandle.subscribe(pPbdObjectTopic, 100, &SceneInferenceEngine::newObservationCallback, this);
    
    // Register callback handlers for scene graphs which contain the raw data used for learning.
    mSceneGraphListener = mNodeHandle.subscribe(pPbdSceneGraphTopic, 5, &SceneInferenceEngine::newSceneGraphCallback, this);
    
    // Read the learning data from bag file.
    readLearnerInputBags(mInputBagFilenames);
  }
  
  SceneInferenceEngine::~SceneInferenceEngine()
  {
  }
  
  void SceneInferenceEngine::update()
  {
    // Status information for the user.
    ROS_DEBUG_STREAM("Updating inference engine.");
    
    /********************************************************************
     * Integrate the collected evidence into the model.
     ********************************************************************/
    
    // Process evidences!
   while(!mEvidenceBuffer.empty())
    {
      // Get the first entry.

      boost::shared_ptr<asr_msgs::AsrObject> evidence = boost::shared_ptr<asr_msgs::AsrObject>(new asr_msgs::AsrObject());//HERE: mEvidenceBuffer.front();

      
      // Remove the entry processed from the queue.
      mEvidenceBuffer.pop();
      
      // Status information for the user.
      ROS_INFO_STREAM("Object of type '" << evidence->type << "' found.");
      
      try{
	// Try to transform evidence into target coordinate system.
	mObjectTransform.transform(evidence);
      }
      catch(std::exception& exception){
	// No transformation found, dropping object!
	ROS_ERROR("Unable to resolve transformation in target coordinate frame. Dropping object.");
	continue;
      }
      //ISM::Pose helperPose = new ISM::Pose(*evidence->sampledPoses.front()->getPosition(), *evidence->sampledPoses.front()->getOrientation());
      //boost::shared_ptr<ISM::Object> convertedEvidence = new ISM::Object("" + evidence->type, helperPose,"" +  evidence->identifier,"" +  evidence->meshResourcePath, "" + evidence->providedBy);
      // Forward the new evidence to the model.
      boost::shared_ptr<ISM::Object> convertedEvidence = boost::shared_ptr<ISM::Object>(new ISM::Object());
      mModel.integrateEvidence(convertedEvidence); //HERE: evidence
    }
    
    // Update the model with the evidence collected until now.
    mModel.updateModel();
    
    /********************************************************************
     * Integrate the learning data loaded from bag file.
     ********************************************************************/

    // Process scene graphs!
    while(!mSceneGraphBuffer.empty())
    {
      // Get the first entry.
      boost::shared_ptr<const asr_msgs::AsrSceneGraph> sceneGraph = mSceneGraphBuffer.front();
      
      // Remove the entry processed from the queue.
      mSceneGraphBuffer.pop();
      
      // Status information for the user.
      ROS_INFO_STREAM("SceneGraph of type '" << sceneGraph->identifier << "' found.");
      
      // Forward evidence to the model.
      mModel.integrateSceneGraph(sceneGraph);
    }
    
    /********************************************************************
     * Do the inference and show the results.
     ********************************************************************/

    // Get the results and show them.
    std::vector<SceneIdentifier> pSceneList;
    mModel.getSceneListWithProbabilities(pSceneList);
    
    printf("===========================================");
    printf("This are the scene probabilities:\n");
    for(SceneIdentifier i : pSceneList)
      printf(" -> %s (%s): %f (%f)\n", i.mDescription.c_str(), i.mType.c_str(), i.mLikelihood, i.mPriori);
    
    // Show plot of scene probabilities?
    if(showPlot)
    {
      // List all scenes to get the labels required for the bar diagram and the associated probabilities.
      std::vector<std::string> barLabels;
      std::map<std::string, float> currentData;
      for(SceneIdentifier i : pSceneList)
      {
	barLabels.push_back(i.mDescription);
	currentData.insert(std::pair<std::string, float>(i.mDescription, i.mLikelihood));
      }
      
      // Update visualizer with new data and visualize!
      mVisGnuplot.updateBarChartValues(currentData);
      mVisGnuplot.sendBarChartToGnuplot(true);
    }
    
    /********************************************************************
     * Visualize the scene.
     ********************************************************************/

    // Do the visualization.
    if(mTargetingHelp)
      mVisualizer->drawInTargetingMode();
    else
      mVisualizer->drawInInferenceMode();
  }
  
  void SceneInferenceEngine::executeInStackMode()
  {
    // Try to get the bag path. We read it here so it doesn't throw any errors in online mode.
    std::string bagPath;
    if(!mNodeHandle.getParam("bag_path", bagPath))
      throw std::runtime_error("Please specify parameter " + std::string("bag_path") + " when starting this node.");
    
    ROS_INFO_STREAM("Extracting AsrObject messages from rosbag file: " << bagPath);

    // Check whether topic name for scene graph has been set before trying to parse rosbag files.
    if(!mObjectListener)
      throw std::logic_error("Cannot parse bag file with AsrSceneGraphs without knowing on which topic they were sent.");
    
    // Set topic representatio. When parsing rosbag files this is required for extracting the messages which are representing scene graphs.
    rosbag::TopicQuery pbdSceneGraphIdentifier(mObjectListener.getTopic());

    // Create file handler for rosbag file to be read.
    rosbag::Bag pbdObjectsBag;

    // Get read-only access to messages in given rosbag file, create access infrastructure.
    try {
      pbdObjectsBag.open(bagPath, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException& exception) {
      // ROS_ERROR does not work here.
      std::cerr << "Trying to extract AsrObject messages aborted because of: " << exception.what() << std::endl;
      // Quit this function as no data is to be processed.
      return;
    }

    // Create interface to extract only scene graph messages in given bag file from a previously defined topic.
    rosbag::View pbdSceneGraphView(pbdObjectsBag, pbdSceneGraphIdentifier);

    // Check whether there is any raw data from a scene on the topic where we expect them.
    if(!pbdSceneGraphView.size())
      ROS_WARN_STREAM("No AsrObject messages exist in " << bagPath << " on topic " << mObjectListener.getTopic() << ".");

    // Get access to all scene graphs in bag file to transfer them to parameter learner for scene model.
    for(rosbag::View::iterator sceneGraphIterator = pbdSceneGraphView.begin(); sceneGraphIterator != pbdSceneGraphView.end(); sceneGraphIterator++) {

      // Get interface compliant to AsrObject message on rosbag item currently taken into account.
      boost::shared_ptr<asr_msgs::AsrObject> currentObject = sceneGraphIterator->instantiate<asr_msgs::AsrObject>();

      
      // Update the model, if an object was found.
      if(currentObject != NULL)
      {
	try{
	  // Try to transform evidence into target coordinate system.
	  mObjectTransform.transform(currentObject);
	}
	catch(std::exception& exception){
	  // No transformation found, dropping object!
	  ROS_INFO("Unable to resolve transformation in target coordinate frame. Dropping object.");
	  continue;
	}
	
	// Forward the new evidence to the model.
    mModel.integrateEvidence(boost::shared_ptr<ISM::Object>(new ISM::Object())); //HERE: currentObject
	
	// Update the model with the evidence collected until now.
	mModel.updateModel();
      }
    }
    
    // Clean up.
    pbdObjectsBag.close();
  }

  void SceneInferenceEngine::loadSceneModel(const std::string pSceneModelFileName, const std::string pInferenceAlgorithm)
  {
    // Message for the user.
    ROS_INFO_STREAM("Initializing inference engine.");
    
    // Load the model from file. That's it. Now it's ready for operation!
    mModel.loadModelFromFile(pSceneModelFileName, pInferenceAlgorithm);
  }
  
  void SceneInferenceEngine::readLearnerInputBags(XmlRpc::XmlRpcValue pInputBagFilenames)
  {
    // If only one string is given to node, just use this as path to scene graphs.
    // Otherwise load a bunch of files and process input as it was one file.
    if(pInputBagFilenames.getType() == XmlRpc::XmlRpcValue::TypeString) {
      extractPbdSceneGraphsFromBag(static_cast<std::string>(pInputBagFilenames));
    } else {
      
      // Go through all paths to AsrSceneGraph rosbag files passed to ros node via cli.
      // Extract all AsrSceneGraph messages from rosbag file currently taken into account.
      for(int i = 0; i < pInputBagFilenames.size(); i++)
	extractPbdSceneGraphsFromBag(static_cast<std::string>(pInputBagFilenames[i]));
    }
  }
  
  void SceneInferenceEngine::extractPbdSceneGraphsFromBag(const std::string& pPbdSceneGraphsBagPath)

  {
    ROS_INFO_STREAM("Extracting AsrSceneGraph messages from rosbag file: " << pPbdSceneGraphsBagPath);


    // Check whether topic name for scene graph has been set before trying to parse rosbag files.
    if(!mSceneGraphListener)
      throw std::logic_error("Cannot parse bag file with AsrSceneGraphs without knowing on which topic they were sent.");
    
    // Set topic representatio. When parsing rosbag files this is required for extracting the messages which are representing scene graphs.
    rosbag::TopicQuery pbdSceneGraphIdentifier(mSceneGraphListener.getTopic());

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
      ROS_WARN_STREAM("No AsrSceneGraph messages exist in " << pPbdSceneGraphsBagPath << " on topic " << mSceneGraphListener.getTopic() << ".");

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
  
  void SceneInferenceEngine::initializeVisualizationChain(const double pScale, const float pSigmaMultiplicator, const std::string pFrameId)
  {
    // Status information for the user.
    ROS_INFO("Initializing visualization mechanism.");
    
    // Create a new coordinator for scene visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSceneModelVisualization());
    
    // Order the model to initialize the visualizers.
    mModel.initializeVisualizer(mVisualizer);
    
    // Set drawing parameters.
    mVisualizer->setDrawingParameters(pScale, pSigmaMultiplicator, pFrameId);
    
        // Get the results and show them.
    std::vector<SceneIdentifier> pSceneList;
    mModel.getSceneListWithProbabilities(pSceneList);
    
    // Show plot of scene probabilities? Initialize here!
    if(showPlot)
    {
      // List all scenes to get the labels required for the bar diagram and the associated probabilities.
      std::vector<std::string> barLabels;
      std::map<std::string, float> currentData;
      for(SceneIdentifier i : pSceneList)
      {
	barLabels.push_back(i.mDescription);
	currentData.insert(std::pair<std::string, float>(i.mDescription, i.mLikelihood));
      }
      
      // Initialize the bar diagram, insert the values and visualize!
      mVisGnuplot.initAnimatedBarChart(barLabels, "Scene Probability", "Probability", std::pair<float, float>(0.0, 1.0));
    }
  }
    

  void SceneInferenceEngine::newObservationCallback(const boost::shared_ptr<asr_msgs::AsrObject>& pObject)

  {
    // Buffers the evidence to keep callback time as short as possible.
    mEvidenceBuffer.push(boost::shared_ptr<ISM::Object>(new ISM::Object)); // HERE: pObject
  }
  
  void SceneInferenceEngine::newSceneGraphCallback(const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph)
  {
    // Buffers the scene graph to keep callback time as short as possible.
    mSceneGraphBuffer.push(pSceneGraph);
  }
  
}
