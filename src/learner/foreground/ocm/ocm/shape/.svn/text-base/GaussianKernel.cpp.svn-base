#include "learner/foreground/ocm/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {
 
  GaussianKernel::GaussianKernel()
  {
  }
  
  GaussianKernel::~GaussianKernel()
  {
  }
  
  void GaussianKernel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer)
  {
    mVisualizer->appendKernel(boost::shared_ptr<Eigen::Vector3d>(new Eigen::Vector3d(mMean->block<3,1>(0,0))),
			      boost::shared_ptr<Eigen::Matrix3d>(new Eigen::Matrix3d(mCovariance->block<3,3>(0,0))));
  }
  
  bool GaussianKernel::compare(const GaussianKernel pKernel)
  {
    bool result = true;
    
    // Compare size of mean vectors.
    result &= (mMean->size() == pKernel.mMean->size());
    
    // Compare the mean vectors.
    for(int i = 0; i < mMean->size(); i++)
      result &= ((*mMean)(i) == (*pKernel.mMean)(i));
    
    // Compare size of covariance matrices..
    result &= (mCovariance->rows() == pKernel.mCovariance->rows());
    result &= (mCovariance->cols() == pKernel.mCovariance->cols());
    
    // Compare the covariance matrices.
    for(int i = 0; i < mCovariance->rows(); i++)
      for(int j = 0; j < mCovariance->cols(); j++)
	result &= ((*mCovariance)(i,j) == (*pKernel.mCovariance)(i,j));
      
    // If both kernels are equal, this should be still true.
    return result;
  }
  
  void GaussianKernel::save(boost::property_tree::ptree& pPt)
  { 
    // Create a seperate tree for this kernel.
    boost::property_tree::ptree subtree;
    
    // Write weight to XML.
    subtree.add("<xmlattr>.weight", mWeight);
    
    // Serialize mean vector and write it to XML.
    std::string meanString;
    SerializationHelper::convertVectorToString(mMean, meanString);
    subtree.add("<xmlattr>.mean", meanString);
    
    // Serialize covariance matrix and write it to XML.
    std::string covarianceString;
    SerializationHelper::convertMatrixToString(mCovariance, covarianceString);
    subtree.add("<xmlattr>.covariance", covarianceString);
        
    // Add subtree to main tree.
    pPt.add_child("kernel", subtree);
  }
  
}