#include "inference/model/foreground/SceneObjectDescription.h"

namespace ProbabilisticSceneRecognition {
  
  SceneObjectDescription::SceneObjectDescription()
  {
  }
  
  SceneObjectDescription::~SceneObjectDescription()
  {
  }
  
  void SceneObjectDescription::load(boost::property_tree::ptree& pPt)
  {
    // Extract scene object name and the type of the content.
    mPriori = pPt.get<double>("<xmlattr>.priori");
    mType = pPt.get<std::string>("<xmlattr>.type");
    mDescription = pPt.get<std::string>("<xmlattr>.name");
    
    ROS_INFO_STREAM("Loading primary scene with type '" << mType << "' and name '" << mDescription << "'.");
    
    // Check, if there was a description specified.
    if(mDescription.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene object specified.");
    
    // Check, if there was a scene object type specified.
    if(mType.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene object '" + mDescription + "' spedified.");
    
    // Check, if there the a priori probability is valid.
    if(mPriori < 0.0 || mPriori > 1.0)
      throw std::invalid_argument("Unable to procees loading. The a priori probability of scene object '" + mDescription + "' is invalid (value: " + boost::lexical_cast<std::string>(mPriori) + ").");
    
    // Create a new scene object content of the given type.
    if(!std::strcmp(mType.c_str(), "ocm")) {
      
      // Create OCM based object content.
      mContent.reset(new OcmSceneObjectContent());
    } else {
      // Warn the user that the specified probabilistic model doesn't exist!
      throw std::invalid_argument("Unable to procees loading. The probabilistic model '" + mType + "' is unknown.");
    }
    
    // Load scenen content.
    mContent->load(pPt);
  }
  
  void SceneObjectDescription::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for primary scene object '" << mDescription << "'.");
    
    // Create a new coordinator for seondary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticPrimarySceneObjectVisualization(mDescription));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
    
    // Forward visualizer to scene object content.
    if(mContent)
      mContent->initializeVisualizer(mVisualizer);
  }
  
  void SceneObjectDescription::update(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Debug message.
    ROS_INFO_STREAM("> Evaluating primary scene object '" << mDescription << "'.");
    
    // Get the start time.
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    
    // Forward evidence to the scene object content.
    if(mContent)
      mContent->update(pEvidenceList);
    
    // Get the stop time and dump the results to file.
    std::chrono::duration<float> diff = std::chrono::high_resolution_clock::now() - start;
    if(pRuntimeLogger.is_open())
    {
      pRuntimeLogger << mDescription << "," << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << "\n";
      pRuntimeLogger.flush();
    } else {
      ROS_INFO_STREAM("Evaluating scene object'" << mDescription << "' took " << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << " milliseconds.");
    }
  }
  
  void SceneObjectDescription::update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    if(mContent)
      mContent->update(pSceneGraph);
  }
  
  double SceneObjectDescription::getSceneObjectProbability()
  {
    double result = 0.0;
    
    if(mContent)
      result = mContent->getSceneObjectProbability();
    
    return result;
  }
  
  double SceneObjectDescription::getSceneObjectPriori()
  {
    return mPriori;
  }
  
  std::string SceneObjectDescription::getDescription()
  {
    return mDescription;
  }
  
  void SceneObjectDescription::setBestStatus(bool pStatus)
  {
    if(mContent)
      mContent->setBestStatus(pStatus);
  }
  
}