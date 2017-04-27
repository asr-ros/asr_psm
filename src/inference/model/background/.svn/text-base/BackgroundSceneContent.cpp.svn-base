#include "inference/model/background/BackgroundSceneContent.h"

namespace ProbabilisticSceneRecognition {
 
  BackgroundSceneContent::BackgroundSceneContent()
  : SceneContent()
  {
  }
  
  BackgroundSceneContent::~BackgroundSceneContent()
  {
  }
  
  void BackgroundSceneContent::load(boost::property_tree::ptree& pPt)
  {
    loadInferenceAlgorithm(pPt);
  }
  
  void BackgroundSceneContent::initializeInferenceAlgorithms(std::string pAlgorithm)
  {
    // Select the inference algorithm described by the parameter.
    if(pAlgorithm.compare("powerset") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new PowerSetBackgroundInferenceAlgorithm()));
    } else if(pAlgorithm.compare("summarized") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new SummarizedBackgroundInferenceAlgorithm()));
    } else if(pAlgorithm.compare("multiplied") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MultipliedBackgroundInferenceAlgorithm()));
    } else if(pAlgorithm.compare("maximum") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MaximumBackgroundInferenceAlgorithm()));
    } else {
      throw std::invalid_argument("Unable to procees loading. The inference algorithm of type '" + pAlgorithm + "' is unknown to the scene of type 'background'.");
    }
  }
  
  void BackgroundSceneContent::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // No need for visualization here!
  }
  
  void BackgroundSceneContent::update(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Command the inference algorithm to execute the inference.
    doInference(pEvidenceList, pRuntimeLogger);
  }
  
  void BackgroundSceneContent::update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    // Scene graph is no of interest here.
  }
  
}