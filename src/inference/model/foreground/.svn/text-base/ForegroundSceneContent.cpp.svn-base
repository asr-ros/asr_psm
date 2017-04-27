#include "inference/model/foreground/ForegroundSceneContent.h"

namespace ProbabilisticSceneRecognition {
 
  ForegroundSceneContent::ForegroundSceneContent()
  : SceneContent()
  {
    mSceneObjects.reset(new std::vector<boost::shared_ptr<SceneObjectDescription> >());
  }
  
  ForegroundSceneContent::~ForegroundSceneContent()
  {
  }
  
  void ForegroundSceneContent::load(boost::property_tree::ptree& pPt)
  {
    // load the inference algorithm.
    loadInferenceAlgorithm(pPt);
    
    // Load all scenen objects which are part of this scene.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pPt)
    {
      // Only access the 'object' child nodes.
      if(!std::strcmp(v.first.c_str(), "object"))
      {
	// Create a new foreground scene instance.
	boost::shared_ptr<SceneObjectDescription> sceneObject(new SceneObjectDescription());
	
	// Let it load its parameters ...
	sceneObject->load(v.second);
	
	// ... and list it.
	mSceneObjects->push_back(sceneObject);
      }
    }
  }
  
  void ForegroundSceneContent::initializeInferenceAlgorithms(std::string pAlgorithm)
  {
    // Select the inference algorithm described by the parameter.
    if(pAlgorithm.compare("powerset") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new PowerSetForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("summarized") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new SummarizedForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("multiplied") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MultipliedForegroundInferenceAlgorithm(mSceneObjects)));
    } else if(pAlgorithm.compare("maximum") == 0) {
      setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm>(new MaximumForegroundInferenceAlgorithm(mSceneObjects)));
    } else {
      throw std::invalid_argument("Unable to procees loading. The inference algorithm of type '" + pAlgorithm + "' is unknown to the scene of type 'foreground'.");
    }
  }
  
  void ForegroundSceneContent::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior)
  {
    // Iterate over all primary scene objects and initialize their visualizers.
    for(unsigned int i = 0; i < mSceneObjects->size(); i++)
      mSceneObjects->at(i)->initializeVisualizer(mSuperior);
  }
  
  void ForegroundSceneContent::update(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Commandy the inference algorithm to execute the inference.
    doInference(pEvidenceList, pRuntimeLogger);
  }
  
  void ForegroundSceneContent::update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    // Forward scene graph to all primary scene objects.
    for(unsigned int i = 0; i < mSceneObjects->size(); i++)
      mSceneObjects->at(i)->update(pSceneGraph);
  }
  
}