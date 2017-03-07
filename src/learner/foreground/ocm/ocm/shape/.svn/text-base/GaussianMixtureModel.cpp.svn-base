#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

namespace ProbabilisticSceneRecognition {
 
  GaussianMixtureModel::GaussianMixtureModel()
  {
  }
  
  GaussianMixtureModel::~GaussianMixtureModel()
  {
  }
  
  void GaussianMixtureModel::addKernel(const GaussianKernel& pKernel)
  {
    // Check, if a kernel of this type already exists.
    for(GaussianKernel kernel : mKernels)
      if(kernel.compare(pKernel))
	return;
    
    // Kernel not already in distribution? Take it!
    mKernels.push_back(pKernel);
  }
  
  void GaussianMixtureModel::normalizeWeights()
  {
    double sum = 0.0;
    
    // Calculate the sum over all kernels.
    for(unsigned int i = 0; i < mKernels.size(); i++)
      sum += mKernels[i].mWeight;
    
    // Normalize the kernels.
    for(unsigned int i = 0; i < mKernels.size(); i++)
      mKernels[i].mWeight /= sum;
  }
  
  unsigned int GaussianMixtureModel::getNumberOfKernels()
  {
    return mKernels.size();
  }
  
  void GaussianMixtureModel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer)
  {
    for(GaussianKernel kernel : mKernels)
      kernel.initializeVisualizer(mVisualizer);
  }
  
  void GaussianMixtureModel::save(boost::property_tree::ptree& pPt, std::string pNode)
  {
    // Create a seperate tree for this gmm.
    boost::property_tree::ptree subtree;
    
    BOOST_FOREACH(GaussianKernel kernel, mKernels)
      kernel.save(subtree);
      
    // Add subtree to main tree.
    pPt.add_child(pNode, subtree);
  }
  
}