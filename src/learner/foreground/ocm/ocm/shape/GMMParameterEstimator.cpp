/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/ocm/shape/GMMParameterEstimator.h"

namespace ProbabilisticSceneRecognition {
 
  GMMParameterEstimator::GMMParameterEstimator(unsigned int pNumberDimensions, unsigned int pNumberKernelsMin, unsigned int pNumberKernelsMax, unsigned int pNumberOfRuns, unsigned int pNumberOfSyntheticSamples, double pIntervalPosition, double pIntervalOrientation, std::string pPathOrientationPlots, unsigned int pAttemptsPerRun)
  : mNumberDimensions(pNumberDimensions)
  , mNumberKernelsMin(pNumberKernelsMin)
  , mNumberKernelsMax(pNumberKernelsMax)
  , mNumberKernels(pNumberKernelsMax - pNumberKernelsMin + 1)
  , mNumberOfRuns(pNumberOfRuns)
  , mNumberOfSyntheticSamples(pNumberOfSyntheticSamples)
  , mIntervalPosition(pIntervalPosition)
  , mIntervalOrientation(pIntervalOrientation)
  , mPathOrientationPlots(pPathOrientationPlots)
  , mAttemptsPerRun(pAttemptsPerRun)
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
    std::vector<double> llk;
    std::vector<double> bic;
    std::vector<GaussianMixtureModel> model;

    // Reserve memory for the data structures.
    nparams.resize(mNumberKernels * mNumberOfRuns);
    llk.resize(mNumberKernels * mNumberOfRuns);
    bic.resize(mNumberKernels * mNumberOfRuns);
    model.resize(mNumberKernels * mNumberOfRuns);

    // Learn a GMM for every number of kernels up to the maximal size.
    for(unsigned int i = 0; i < mNumberKernels; i++)
    {
        bool learningCycleCompleted = false;

        // generate mNumberOfRuns models and pick the best.
        for(unsigned int run = 0; run < mNumberOfRuns; run++)
        {

            ROS_INFO_STREAM("Learning run " << run + 1 << ".");

            bool useGenericMatrices = true;

            // Repeat this calculation as long as the expectation maximizations finds a degenerated distribution,
            // but maximal mAttemptsPerRun (formerly 100) times.
            for(int timer = mAttemptsPerRun; timer > 0; timer--)
            {
                unsigned int offset  = i * mNumberOfRuns + run;

                if(runExpectationMaximization(mData,mNumberKernelsMin + i, nparams[offset], llk[offset], bic[offset], model[offset], useGenericMatrices))
                {
                    learningCycleCompleted = true;
                    break;
                }
                else
                {
                    ROS_INFO_STREAM("Training unsuccessful. Repeating cycle.");
                    if (timer == (int) (mAttemptsPerRun / 2) + 1 && mAttemptsPerRun != 1) // at halfway point: switch to less precise but more stable diagonal matrices instead of generic ones
                    {
                        ROS_INFO_STREAM("Attempting to learn diagonal covariance matrix instead of generic one.");
                        useGenericMatrices = false;
                    }
                }
            }
        }

        // Terminate if OpenCV failed to learn a proper model.
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
    gmm = mBestGMM;
    ROS_INFO_STREAM("Number of kernels after removing duplicates is " << mBestGMM.getNumberOfKernels() << ".");
  }
  
  void GMMParameterEstimator::plotModel()
  {
    unsigned int numberOfKernels = mBestGMM.getNumberOfKernels();

    // Iterate over all Gaussian Kernels:
    for (unsigned int i = 0; i < numberOfKernels; i++)
    {
        // Extract the mean and covariance of the kernel.
        GaussianKernel kernel = mBestGMM.getKernels().at(i);
        boost::shared_ptr<Eigen::VectorXd> mean = kernel.mMean;
        boost::shared_ptr<Eigen::MatrixXd> cov = kernel.mCovariance;

        // Prepare learner for histogramm.
        std::vector<double> histInRoll, histInPitch, histInYaw;

        unsigned int sampleAmount = 100;
        // Draw samples
        std::vector<Eigen::VectorXd> samples(sampleAmount);
        // Fill std::vector with Eigen::vectors of the correct size
        for (unsigned int s = 0; s < sampleAmount; s++) samples.at(s) = Eigen::VectorXd(mean->size());
        MathHelper::drawNormal(*mean, *cov, sampleAmount, samples);

        for (unsigned int i = 0; i < sampleAmount; i++)
        {
            Eigen::VectorXd sample = samples[i];

            // Create Eigen Quaternion
            Eigen::Quaterniond quat = Eigen::Quaterniond(sample[0], sample[1], sample[2], sample[3]);

            // Extract euler angles from the given seed sample.
            Eigen::Vector3d ea = quat.toRotationMatrix().eulerAngles(0, 1, 2);

            // Push negative values into interval.
            if(ea[0] < -M_PI) ea[0] += M_PI;
            if(ea[1] < -M_PI) ea[1] += M_PI;
            if(ea[2] < -M_PI) ea[2] += M_PI;

            if(ea[0] > M_PI) ea[0] -= M_PI;
            if(ea[1] > M_PI) ea[1] -= M_PI;
            if(ea[2] > M_PI) ea[2] -= M_PI;

            // Add sample to input of cv::calcHist
            histInRoll.push_back(ea[0]);
            histInPitch.push_back(ea[1]);
            histInYaw.push_back(ea[2]);
        }
        // Get current timestamp.
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // Get and plot distributions.
        // Calculate histograms by hand:
        double lower = -M_PI;   // lower limit
        double upper = M_PI;    // upper limit
        double buckets = sampleAmount;   // amount of buckets

        std::vector<std::pair<double, double>> histOutRoll(buckets), histOutPitch(buckets), histOutYaw(buckets);
        MathHelper::calcHistogram(lower, upper, buckets, histInRoll, histOutRoll);
        MathHelper::calcHistogram(lower, upper, buckets, histInPitch, histOutPitch);
        MathHelper::calcHistogram(lower, upper, buckets, histInYaw, histOutYaw);

        // scale buckets into probabilities
        for (unsigned int i = 0; i < histOutRoll.size(); i++) histOutRoll.at(i).second /= histOutRoll.size();
        for (unsigned int i = 0; i < histOutPitch.size(); i++) histOutPitch.at(i).second /= histOutPitch.size();
        for (unsigned int i = 0; i < histOutYaw.size(); i++) histOutYaw.at(i).second /= histOutYaw.size();

        // plot them into the gnuplot files and on screen
        plotOrientationHistogram(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-roll.gp", histOutRoll, "roll");
        plotOrientationHistogram(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-pitch.gp", histOutPitch, "pitch");
        plotOrientationHistogram(mPathOrientationPlots + "/" + std::to_string(timestamp) + "-yaw.gp", histOutYaw, "yaw");
    }
  }

  bool GMMParameterEstimator::runExpectationMaximization(const std::vector<std::vector<double>> data,
                  unsigned int nc,
                  unsigned int& nparams,
                  double& llk,
                  double& bic,
                  GaussianMixtureModel& model,
                  bool useGenericMatrices)
  {
      int covMatType = cv::EM::COV_MAT_GENERIC; // by default, use a generic covariance matrix
      // If there is little data, a less precise diagonal matrix is more stable as a covariance matrix:
      if (!useGenericMatrices)
          covMatType = cv::EM::COV_MAT_DIAGONAL; // diagonal covariance matrix

      // Creating the EM learner instance
      cv::EM myEM = cv::EM(nc, covMatType,
             cv::TermCriteria(                // termination criteria
                 cv::TermCriteria::COUNT + cv::TermCriteria::EPS, // use maximum iterations and convergence epsilon
                 10,  // number of maximum iterations. 10 was used in ProBT.
                 0.01 // epsilon. 0.01 was used in ProBT.
             )
      );

      cv::Mat loglikelihoods;
      // Put data in cv::Mat
      cv::Mat cvdata(data.size(), data.at(0).size(), CV_64FC1);
      for (int i = 0; i < cvdata.rows; i++)
          for(int j = 0; j < cvdata.cols; j++)
              cvdata.at<double>(i, j) = data.at(i).at(j);

      // Run until convergence
      if (!myEM.train(cvdata, loglikelihoods)) return false;  // Training failed: return false

      // extract covariance matrices and check whether they are symmetric and positive semi-definite, i.e. whether they actually are valid covariance matrices:
      std::vector<cv::Mat> cvcovs = myEM.get<std::vector<cv::Mat>>("covs");
      std::vector<boost::shared_ptr<Eigen::MatrixXd>> covs(cvcovs.size());
      for (unsigned int i = 0; i < cvcovs.size(); i++)
      {
          cv::Mat cvcov = cvcovs.at(i);
          // Check whether matrix is quadratic:
          if (cvcov.rows != cvcov.cols)
          {
              ROS_DEBUG_STREAM("Matrix not quadratic: not a valid covariance matrix");
              return false;
          }
          Eigen::MatrixXd cov(cvcov.rows, cvcov.cols);
          // Check symmetry of matrix:
          for (int j = 0; j < cvcov.rows; j++)
          {
              for (int k = j; k < cvcov.cols; k++)
              {
                  double entry = cvcov.at<double>(j,k);
                  if (j != k)   // if not on diagonal:
                  {
                      if(std::abs(entry - cvcov.at<double>(k,j)) >= std::numeric_limits<double>::epsilon()) // check symmetry
                      {
                          ROS_DEBUG_STREAM("Found unsymmetric covariance matrix entries " << cvcov.at<double>(j,k) << " (" << j << "," << k << "), "
                                           << cvcov.at<double>(k,j) << " (" << k << "," << j << "). Not a valid covariance matrix.");
                          return false; // Not symmetric: return false
                      }
                      cov(k,j) = entry; // if symmetric: set lower entry
                  }
                  cov(j,k) = entry; // set upper entry or entry on diagonal
              }
          }

          // Check positive semi-definiteness by checking whether all eigenvalues are positive:
          Eigen::MatrixXd eigenvalues;
          if (cov.rows() == 3) // dimension 3
          {
              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
              eigenvalues = es.eigenvalues();

          }
          else if (cov.rows() == 4)
          {
              Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(cov);
              eigenvalues = es.eigenvalues();
          }
          else
              throw std::invalid_argument("In GMMParameterEstimator::runExpectationMaximization(): Found matrix with unsupported number of dimensions (supported are 3,4).");

          for (unsigned int i = 0; i < eigenvalues.size(); i++)
          {
              if (eigenvalues(i) < 0)
              {
                  ROS_DEBUG_STREAM("Found invalid eigenvalue " << eigenvalues(i) << ": matrix not positive semi-definite, not a valid covariance matrix.");
                  return false; // Not positive semi-definite: return false
              }
              else if (eigenvalues(i) < std::numeric_limits<double>::epsilon())
              {
                  ROS_DEBUG_STREAM("Found eigenvalue 0: Matrix cannot be inverted, invalid.");
                  return false; // matrix not invertible: return false (is a valid covariance matrix, but cannot be used in inference).
              }
          }

          covs.at(i) = boost::shared_ptr<Eigen::MatrixXd>(new Eigen::MatrixXd(cov)); // only if matrix is valid: put it into list.
      }

      // Fill the output parameters
      llk = 0;
      for (int i = 0; i < loglikelihoods.rows; i++) llk += loglikelihoods.at<double>(i,0); // llk is sum of the loglikelihoods for each sample

      // model:
      // extract weights
      cv::Mat cvweights = myEM.get<cv::Mat>("weights");
      std::vector<double> weights(cvweights.cols);
      for (int i = 0; i < cvweights.cols; i++) weights.at(i) = cvweights.at<double>(i);
      // extract means
      cv::Mat cvmeans = myEM.get<cv::Mat>("means");
      std::vector<boost::shared_ptr<Eigen::VectorXd>> means(cvmeans.rows);
      for (int i = 0; i < cvmeans.rows; i++)
      {
          Eigen::VectorXd mean(cvmeans.cols);
          for (int j = 0; j < cvmeans.cols; j++) mean[j] = cvmeans.at<double>(i, j);
          means.at(i) = boost::shared_ptr<Eigen::VectorXd>(new Eigen::VectorXd(mean));
      }

      GaussianMixtureModel newmodel = GaussianMixtureModel();
      // Iterate over all gaussian kernels and save them.
      for(unsigned int i = 0; i < weights.size(); i++)
      {
        GaussianKernel kernel;          // Create a new gaussian kernel.
        kernel.mWeight = weights[i];    // Set the weight.
        kernel.mMean = means[i];        // Set the mean vector.
        kernel.mCovariance = covs[i];   // Set the covariance matrix.

        newmodel.addKernel(kernel);        // Add the kernel to the GMM.
      }
      // We removed kernels (automatically in addKernel), so we have to normalize the weights
      newmodel.normalizeWeights();
      model = newmodel;

      // number of independent parameters nparams (for generic covariance matrix):
      nparams = (model.getKernels().size() - 1);
      if (!model.getKernels().empty()) {
          unsigned int d = model.getKernels().at(0).mMean->size();       // dimension of mean vector and also symmetric covariance matrix
          if (useGenericMatrices)
              nparams += (d * (d + 1) / 2 + d) * model.getKernels().size();   // (free variables in covariance matrix + in mean) * number of kernels + number of weights - 1
          else
              nparams += 2 * d * model.getKernels().size(); // (free variables on diagonal of covariance matrix + in mean) * number of kernels + number of weights - 1
          // weights are parameters too, but if there is only one kernel, it's not independent (since it has to be 1)
          // one weight is also always determined by the rest, since the sum has to be 1.
      }
      else
          throw std::runtime_error("In GMMParameterEstimator::runExpectationMaximization(): trying to use empty model.");

      // Bayesian information criterion BIC:
      bic = llk - 0.25 * (double) nparams * std::log((double) data.size());

      return true;  // Training succeeded: return true
  }

  void GMMParameterEstimator::plotOrientationHistogram(const std::string& filename, const std::vector<std::pair<double, double>> data, const std::string& rotaxis)
  {
      PlotHelper plotHelper(filename);

      double min_x, max_x, min_y, max_y;
      min_x = max_x = min_y = max_y = 0;
      for (std::pair<double, double> datum: data)
      {
          if (datum.first < min_x) min_x = datum.first;
          if (datum.first > max_x) max_x = datum.first;
          if (datum.second < min_y) min_y = datum.second;
          if (datum.second > max_y) max_y = datum.second;
      }
      double xdistance = 0;
      if (data.size() > 1) xdistance = std::abs(data.at(1).first - data.at(0).first);   // Assuming all data points have equal distance on the x axis
      std::pair<double, double> xRange(min_x, max_x + xdistance);                       // Add xdistance to be able to stretch the last point out, see below
      std::pair<double, double> yRange(min_y, max_y + ((double) 1 / data.size()));          // y axis goes up to the bucket of the histogram with the highest value and a little above

      plotHelper.initPlot("Tabular(" + rotaxis + ")", rotaxis, "P(" + rotaxis + ")", xRange, yRange, data.size()); // using data.size() as amount of samples
      for (unsigned int i = 0; i < data.size() - 1; i++)    // add each data point twice for histogram-like look
      {
          plotHelper.addPointToBuffer(data.at(i).first, data.at(i).second);
          plotHelper.addPointToBuffer(data.at(i+1).first, data.at(i).second);
      }
      if (!data.empty())    // add last point twice too
      {
          plotHelper.addPointToBuffer(data.at(data.size() - 1).first, data.at(data.size() - 1).second);
          plotHelper.addPointToBuffer(data.at(data.size() - 1).first + xdistance, data.at(data.size() - 1).second);
      }
      plotHelper.sendPlot();
  }
  
}
