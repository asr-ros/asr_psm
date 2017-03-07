#include "inference/model/foreground/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {
 
  GaussianKernel::GaussianKernel(unsigned int pDimension, boost::property_tree::ptree& pPt)
  : mDimension(pDimension)
  {
    // Initialize pointers.
    mMean.reset(new Eigen::VectorXd(pDimension));
    mCovariance.reset(new Eigen::MatrixXd(pDimension, pDimension));
    
    mPositionMean.reset(new Eigen::Vector3d());
    mPositionCovariance.reset(new Eigen::Matrix3d());
    
    // Load weight, mean and covariance.
    load(pPt);
  }
  
  GaussianKernel::~GaussianKernel()
  {
    
  }
  
  void GaussianKernel::load(boost::property_tree::ptree& pPt)
  {
    // Load the weight.
    mWeight = pPt.get<double>("<xmlattr>.weight");
    
    // Load the mean vector.
    std::string meanAsString = pPt.get<std::string>("<xmlattr>.mean");
    SerializationHelper::convertStringToVector(mDimension, meanAsString, mMean);
    
    // Load the covariance matrix.
    std::string covarianceAsString = pPt.get<std::string>("<xmlattr>.covariance");
    SerializationHelper::convertStringToMatrix(mDimension, covarianceAsString, mCovariance);
    
    // Extract the parts of mean and covariance relevant for the position.
    *mPositionMean = mMean->block<3,1>(0,0);
    *mPositionCovariance = mCovariance->block<3,3>(0,0);
  }
  
  void GaussianKernel::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer)
  {
    mVisualizer->appendKernel(mPositionMean, mPositionCovariance);
  }
  
  void GaussianKernel::visualize(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer, Eigen::VectorXd pEvidence)
  {
    mVisualizer->setDetectionCertainty(evaluate(pEvidence));
  }
  
  double GaussianKernel::evaluate(Eigen::VectorXd pEvidence)
  {
    return mWeight * exp(-sqrt((double) ((pEvidence - *mMean).transpose() * mCovariance->inverse() * (pEvidence - *mMean))));
  }
  
  double GaussianKernel::getWeight()
  {
    return mWeight;
  }
  
}
