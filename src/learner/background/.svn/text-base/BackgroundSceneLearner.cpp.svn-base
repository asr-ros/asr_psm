#include "learner/background/BackgroundSceneLearner.h"

namespace ProbabilisticSceneRecognition {

  BackgroundSceneLearner::BackgroundSceneLearner()
  : SceneLearner("background")
  {
    
  }
  
  BackgroundSceneLearner::~BackgroundSceneLearner()
  {
  }
  
  void BackgroundSceneLearner::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // No need for visualization here.
  }
  
  void BackgroundSceneLearner::save(boost::property_tree::ptree& pPt)
  {
    // Create a subtree.
    boost::property_tree::ptree subtree;
    
    // Add scene name and type.
    subtree.add("<xmlattr>.type", "background");
    subtree.add("<xmlattr>.name", "background");
    subtree.add("<xmlattr>.priori", mPriori);
    
    // Add parameters relevant for the background scene.
    subtree.add("description.<xmlattr>.objects", mMaximalNumberOfObjects);
    subtree.add("description.<xmlattr>.volume", mWorkspaceVolume);
    
    // Add subtree to main tree.
    pPt.add_child("psm.scenes.scene", subtree);
  }

  void BackgroundSceneLearner::learn()
  {
    // This is the list we store all object types. The size of the list will be the number of all unique objects.
    std::vector<std::string> index;

    // Iterate over all examples for all scenes.
    BOOST_FOREACH(boost::shared_ptr<const pbd_msgs::PbdSceneGraph> example, mExamplesList)
    {
      // Iterate over all PbdNodes in the scene (a node represent an object).
      BOOST_FOREACH(pbd_msgs::PbdNode node, example->scene_elements)
      {	
	// Extract the object type from the first observation.
	std::string type = node.track[0].type;
	
	// If the object is not already known, add it to the list of known objects.
	if(std::find(index.begin(), index.end(), type) == index.end())
	  index.push_back(type);
      }
    }
    
    // This is the number of unique objects.
    mMaximalNumberOfObjects = index.size();
  }
    
}