#include "learner/foreground/ocm/ocm/shape/GMMParameterEstimator.h"

namespace ProbabilisticSceneRecognition {
 
  GMMParameterEstimator::GMMParameterEstimator(unsigned int pNumberDimensions, unsigned int pNumberKernelsMin, unsigned int pNumberKernelsMax, unsigned int pNumberOfRuns, unsigned int pNumberOfSyntheticSamples, double pIntervalPosition, double pIntervalOrientation, std::string pPathOrientationPlots)
  : mNumberDimensions(pNumberDimensions)
  , mNumberKernelsMin(pNumberKernelsMin)
  , mNumberKernelsMax(pNumberKernelsMax)
  , mNumberKernels(pNumberKernelsMax - pNumberKernelsMin + 1)
  , mNumberOfRuns(pNumberOfRuns)
  , mNumberOfSyntheticSamples(pNumberOfSyntheticSamples)
  , mIntervalPosition(pIntervalPosition)
  , mIntervalOrientation(pIntervalOrientation)
  , mPathOrientationPlots(pPathOrientationPlots)
  {
  }
  
  GMMParameterEstimator::~GMMParameterEstimator()
  {
  }
  
  void GMMParameterEstimator::addDatum(Eigen::Vector3d pSample)
  {
    // Check, if leaner was properly configured.
    if(mNumberDimensions != 3)
      throw std::invalid_argument("Cannot add a 3d sample to a learner configured for " + boost::lexical_cast<std::string>(mNumberDimensions) + "d samples.");
    
    // Add the original sample.
    std::vector<double> datum;
    datum.push_back(pSample[0]);
    datum.push_back(pSample[1]);
    datum.push_back(pSample[2]);
    mData.push_back(datum);
    
    if(mNumberOfSyntheticSamples > 0)
    {
      // Generate random generators for all three axes.
      std::random_device rd;
      std::default_random_engine generator(rd());
      std::uniform_real_distribution<double> distribution(-mIntervalPosition, mIntervalPosition);
      
      // Generate a number of random samples distributed around the given seed sample.
      for(unsigned int i = 0; i < mNumberOfSyntheticSamples; i++)
      {
	std::vector<double> datum;
	
	datum.push_back(pSample[0] + distribution(generator));
	datum.push_back(pSample[1] + distribution(generator));
	datum.push_back(pSample[2] + distribution(generator));

	mData.push_back(datum);
      }
    }
  }
  
  void GMMParameterEstimator::addDatum(Eigen::Quaternion<double> pSample)
  {
    // Check, if learner was properly configured.
    if(mNumberDimensions != 4)
      throw std::invalid_argument("Cannot add a 4d sample to a learner configured for " + boost::lexical_cast<std::string>(mNumberDimensions) + "d samples.");
    
    // Add the original sample.
    std::vector<double> datum;
    datum.push_back(pSample.w());
    datum.push_back(pSample.x());
    datum.push_back(pSample.y());
    datum.push_back(pSample.z());
    mData.push_back(datum);
    
    if(mNumberOfSyntheticSamples > 0)
    {
      // Extract euler angles from the given seed sample.
      Eigen::Vector3d ea = pSample.toRotationMatrix().eulerAngles(0, 1, 2);
      
      // Generate random generators for all three axes.
      std::random_device rd;
      std::default_random_engine generator(rd());
      std::uniform_real_distribution<double> distribution(-(mIntervalOrientation * M_PI) / 180.0, (mIntervalOrientation * M_PI) / 180.0);
      
      // Generate a number of random samples distributed around the given seed sample.
      for(unsigned int i = 0; i < mNumberOfSyntheticSamples; i++)
      {
	std::vector<double> datum;
	
	// Create rotation matrix, add noise to rotation.
	Eigen::Matrix3d m(Eigen::AngleAxisd(ea[0] + distribution(generator), Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(ea[1] + distribution(generator), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(ea[2] + distribution(generator), Eigen::Vector3d::UnitZ()));
	
	// Convert rotation matrix back to quaternion and add the noised sample to the list.
	Eigen::Quaternion<double> noisedSample(m);
	datum.push_back(noisedSample.w());
	datum.push_back(noisedSample.x());
	datum.push_back(noisedSample.y());
	datum.push_back(noisedSample.z());
	
	// Add datum to list.
	mData.push_back(datum);
      }
    }
  }
  
  void GMMParameterEstimator::learn()
  {
    // Define the containers that store the GMMS and metadata for all numbers of kernels.
    std::vector<unsigned int> nparams;
    std::vector<plFloat> llk;
    std::vector<plFloat> bic;
    std::vector<plJointDistribution> model;
    
    // Reserve memory for the data structures.
    nparams.resize(mNumberKernels * mNumberOfRuns);
    llk.resize(mNumberKernels * mNumberOfRuns);
    bic.resize(mNumberKernels * mNumberOfRuns);
    model.resize(mNumberKernels * mNumberOfRuns);
    
    // Open a wormhole to the temporary CSV file defined in TEMP_FILE.
    std::ofstream out(TEMP_FILE.c_str());
    if(!out) {
      std::cerr << "GMM learner: Failed to open '" << TEMP_FILE << "' for output of csv file." << std::endl;
      exit(-1);
    }
    
    // Write the header of the CSV file.
    out << "C";
    for(unsigned int i = 0; i < mNumberDimensions; i++)
      out << ";X" << boost::lexical_cast<std::string>(i);
    out << std::endl;
    
    // Dump the data to csv file.
    for(unsigned int i = 0; i < mData.size(); i++)
    {
      // Get the datum.
      std::vector<double> datum = mData[i];
      
      // Dump a single datum to csv.
      for(unsigned int j = 0; j < mNumberDimensions; j++)
	out << ";" << datum[j];
      out << std::endl;
    }
    out.close();
    
    // Learn a GMM for every number of kernels up to the maximal size.
    for(unsigned int i = 0; i < mNumberKernels; i++)
    {
      bool learningCycleCompleted = false;
      
      // generate 100 models and pick the best.
      for(unsigned int run = 0; run < mNumberOfRuns; run++)
      {
	ROS_INFO_STREAM("Learning run " << run + 1 << ".");
	
	// Repeat this calculation as long as ProBT throws a "degenerated distribution" error,
	// but maximal 100 times.
	for(int timer = 100; timer > 0; timer--)
	{
	  try {
	    unsigned int offset  = i * mNumberOfRuns + run;
	    
	    runExpectationMaximization(TEMP_FILE, mNumberKernelsMin + i, nparams[offset], llk[offset], bic[offset], model[offset]);
	    learningCycleCompleted = true;
	    break;
	  } catch (plError e) {
	    std::cout << "ProBT lerner error. Repeating cycle." << std::endl;
	  }
	}
      }
      
      // Terminate if ProBT failed to learn a proper model.
      if(!learningCycleCompleted) {
	ROS_ERROR("Learning cycle incomplete. GMM learning failed! Please restart learner!");
	exit (EXIT_FAILURE);
      } else {
	ROS_INFO("Learning cycle completed.");
      }
    }
    
    // Determine the index of the GMM with the best BIC score.
    unsigned int indexOfBestBic = std::max_element(bic.begin(), bic.end()) - bic.begin();
    
    // Information to user about the number of kernels
    for(unsigned int i = 0; i < mNumberKernels; i++)
    {
      for(unsigned int run = 0; run < mNumberOfRuns; run++)
      {
	std::stringstream msg;
	
	unsigned int offset  = i * mNumberOfRuns + run;
	
	// Mark the number of kernels with the best score.
	if(offset == indexOfBestBic) msg << "[x] ";
	else msg << "[ ] ";
	
	// Print the bic score and log likelihood.
	msg << "Kernels: " << mNumberKernelsMin + i << " Run: " << run + 1 << " BIC: " << bic[offset] << " LLK: " << llk[offset];
	
	// Print message to console.
	ROS_INFO_STREAM("" << msg.str());
      }
    }
    
    // Store the GMM in the 'mBestGMM' variable.
    mBestGMM = model[indexOfBestBic];
  }
  
  void GMMParameterEstimator::getModel(GaussianMixtureModel& gmm)
  {
    // The mixture weights.
    std::vector<double> weights;
    
    // Get the gaussian mixture distribution.
    plComputableObjectList wList = mBestGMM.get_computable_object_list();
    
    // Extract the mixture weights (in the easy understandable way).
    plComputableObject wObject = wList[0];
    plProbTable wTable = (plProbTable) ((plDistribution) wObject);
    wTable.get_parameters(weights);
    
    // Extract the gaussian distributions (in the easy understandable way).
    plComputableObject dObject = wList[1];
    plCndDistribution dCndDist = (plCndDistribution) dObject;
    
    // Iterate over all gaussian kernels and save them.
    for(unsigned int i = 0; i < weights.size(); i++)
    {
      // Extract the mean and covariane of the kernel.
      plNormal normal = (plNormal) dCndDist.instantiate(i);
      
      plFloatVector mean;
      plFloatMatrix covariance;
      normal.get_parameters(mean, covariance);
      
      // Create a new gaussian kernel.
      GaussianKernel kernel;
      
      // Set the weight.
      kernel.mWeight = weights[i];
      
      // Set the mean vector.
      MathHelper::copy(mean, kernel.mMean);
      
      // Set the covariance matrix.
      MathHelper::copy(covariance, kernel.mCovariance);
      
      // Add the kernel to the GMM.
      gmm.addKernel(kernel);
    }
    
    // We removed kernels, so we have to normalize the weights.
    gmm.normalizeWeights();
    
    ROS_INFO_STREAM("Number of kernels after removing duplicates is " << gmm.getNumberOfKernels() << ".");
  }
  
  void GMMParameterEstimator::plotModel()
  {
    // The mixture weights.
    std::vector<double> weights;
    
    // Get the gaussian mixture distribution.
    plComputableObjectList wList = mBestGMM.get_computable_object_list();
    
    // Extract the mixture weights (in the easy understandable way).
    plComputableObject wObject = wList[0];
    plProbTable wTable = (plProbTable) ((plDistribution) wObject);
    wTable.get_parameters(weights);
    
    // Extract the gaussian distributions (in the easy understandable way).
    plComputableObject dObject = wList[1];
    plCndDistribution dCndDist = (plCndDistribution) dObject;
    
    // Iterate over all gaussian kernels and save them.
    for(unsigned int i = 0; i < weights.size(); i++)
    {
      // Extract the mean and covariane of the kernel.
      plNormal normal = (plNormal) dCndDist.instantiate(i);
      
      plFloatVector mean;
      plFloatMatrix covariance;
      normal.get_parameters(mean, covariance);
      
      // Define learner for histogramm.
      
      plIntervalType histTypeRoll(-M_PI, M_PI, 100);
      plIntervalType histTypePitch(-M_PI, M_PI, 100);
      plIntervalType histTypeYaw(-M_PI, M_PI, 100);
      plSymbol histSymbRoll("roll", histTypeRoll);
      plSymbol histSymbPitch("pitch", histTypePitch);
      plSymbol histSymbYaw("yaw", histTypeYaw);
      plLearnHistogram histLearnerRoll(histSymbRoll);
      plLearnHistogram histLearnerPitch(histSymbPitch);
      plLearnHistogram histLearnerYaw(histSymbYaw);
      
      // Fill learner with samples.
      for(int i = 0; i < 100; i++)
      {
	// Draw samples from
	plValues samples = normal.draw();
	
	// Create Eigen Quaternion
	Eigen::Quaterniond quat = Eigen::Quaterniond(samples[0], samples[1], samples[2], samples[3]);
	
	// Extract euler angles from the given seed sample.
	Eigen::Vector3d ea = quat.toRotationMatrix().eulerAngles(0, 1, 2);
	
	std::cout << ea << std::endl;
	
	// Push negative values into interval.
	if(ea[0] < -M_PI) ea[0] += M_PI;
	if(ea[1] < -M_PI) ea[1] += M_PI;
	if(ea[2] < -M_PI) ea[2] += M_PI;
	
	if(ea[0] > M_PI) ea[0] -= M_PI;
	if(ea[1] > M_PI) ea[1] -= M_PI;
	if(ea[2] > M_PI) ea[2] -= M_PI;
	
	// Add sample to learner.
// 	histLearnerRoll.add_point((ea[0] * 180) / M_PI);
// 	histLearnerPitch.add_point((ea[1] * 180) / M_PI);
// 	histLearnerYaw.add_point((ea[2] * 180) / M_PI);
	histLearnerRoll.add_point(ea[0]);
	histLearnerPitch.add_point(ea[1]);
	histLearnerYaw.add_point(ea[2]);
      }
      
      // Get current tiemstamp.
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      
      // Get and plot distributions.
      plDistribution distRoll = histLearnerRoll.get_distribution();
      plDistribution distPitch = histLearnerPitch.get_distribution();
      plDistribution distYaw = histLearnerYaw.get_distribution();
      
      distRoll.plot(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-roll.gp");
      distPitch.plot(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-pitch.gp");
      distYaw.plot(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-yaw.gp");
    }
  }
  
  void GMMParameterEstimator::runExpectationMaximization(const std::string& file,
					      unsigned int nc,
					      unsigned int& nparams,
					      plFloat& llk,
					      plFloat& bic,
					      plJointDistribution& model)
  {
    // Instantiate random generator.
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_int_distribution<int> distribution(0, mData.size());
    
    // Define the symbols for the two distributions.
    const plSymbol C("C", plIntegerType(0, nc - 1));
    const plArray X("X", plRealType(-1000, 1000), 1, mNumberDimensions);
    
    // Create and randomly initialize the weights table P(C).
    const bool random_prob = true;
    const plProbTable pc_init(C, random_prob);
    
    // Create the initial gaussians: P(X | C)
    plDistributionTable px_init(X, C);
    for(unsigned int i = 0; i < nc; ++i)
    {
      // Initialize covariance matrix with the identity.
      plFloatMatrix cov(mNumberDimensions);
      for(unsigned int j = 0; j < mNumberDimensions; j++)
	cov[j][j] = 0.0001;
      
      // Instantiate random mean vector.
      plFloatVector mean(mNumberDimensions);
      std::vector<double> sample = mData[distribution(generator)];
      for(unsigned int j = 0; j < mNumberDimensions; j++)
	mean[j] = sample[j];
      
      px_init.push(plNormal(X, mean, cov), int(i));
    }
    
    // P(C) is learnt as an histogram.
    plLearnHistogram LC(C);
    
    // P(X | C) is learnt as a set of gaussians (a gaussian for each value of C)
    plCndLearnObject <plLearnNdNormal> LX(X, C);
    
    // Creating the EM learner instance
    plCSVFileDataDescriptor<double> myCSVdata(file, C^X);
    std::vector <plLearnObject*> learn_objs(2); learn_objs[0] = &LC; learn_objs[1] = &LX;
    plEMLearner myEM(pc_init * px_init, learn_objs);
    
    // Run until convergence
    myEM.run(myCSVdata, 0.01, 10);

    // Fill the output parameters
    nparams = myEM.get_n_parameters();
    llk = myEM.get_last_computed_loglikelihood();
    bic = llk - 0.25 * nparams * std::log(myCSVdata.get_n_records());
    model = myEM.get_joint_distribution();
  }
  
}