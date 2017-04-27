/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <random>
#include <string>
#include <vector>
#include <chrono>

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>

#include <Pose.h>

// Local includes.
#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

#include "helper/MathHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Path to the temporary file used b ythe GMM leaner. It stores the learning data in CSV format.
   */
  static const std::string TEMP_FILE ="/tmp/gmm_learner.csv";
  
  /**
   * This is a learner for gaussian mixture models (GMMs). It uses an EM algorithm to learn the mixture distribution. A maximumum likelihood process is used, so different numbers of kernels up to a maximal number are tried. The GMM with the highest likelihood is taken.
   * 
   * The implementation is taken from the probt documentation and can be found in the following subdirectory: /doc/html/d1/d22/a00057.html'
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class GMMParameterEstimator {
  public:

    /**
     * Constructor.
     * 
     * @param pNumberOfDimensions The number of dimensions of the learning samples.
     * @param pNumberOfKernelsMin The minimal number of kernels to try during GMM learning.
     * @param pNumberOfKernelsMax The maximal number of kernels to try during GMM learning.
     * @param pNumberOfRuns The number of runs per kernel.
     * @param pNumberOfSyntheticSamples The number of synthetic voised samples to gereate from a given seed sample.
     * @param pIntervalPosition The position interval for the sample relaxiation.
     * @param pIntervalOrientation The orientation interval for the sample relaxiation.
     * @param pPathOrientationPlots Path to the orientation plots.
     */
    GMMParameterEstimator(unsigned int pNumberOfDimensions, unsigned int pNumberOfKernelsMin, unsigned int pNumberOfKernelsMax, unsigned int pNumberOfRuns, unsigned int pNumberOfSyntheticSamples, double pIntervalPosition, double pIntervalOrientation, std::string pPathOrientationPlots);
    
    /**
     * Destructor.
     */
    ~GMMParameterEstimator();
    
    /**
     * Adds a single datum to the learner.
     * 
     * @param pSample A single sample.
     */
    void addDatum(Eigen::Vector3d pSample);
    
    /**
     * Adds a single datum to the learner.
     * 
     * @param pSample A single sample.
     */
    void addDatum(Eigen::Quaternion<double> pSample);
    
    /**
     * Learns a gaussian mixture model from the given data.
     */
    void learn();
    
    /**
     * Copies the gaussian mixture model into the given container.
     * 
     * @param gmm The container to copy the gaussian mixture model into.
     */
    void getModel(GaussianMixtureModel& gmm);
    
    /**
     * Plots the model using ProBT.
     */
    void plotModel();
    
  private:
    
    /**
     * Runs the EM algorithm for a given number of kernels.
     * 
     * @param file Path to a temporary file used to store working data.
     * @param nc Number of kernels to do the EM algorithm for.
     * @param nparams Number of parameters as determined by the EM algorithm.
     * @param llk Log likelihood of the model evaluated under the learning data.
     * @param bic Bayesian Information Criterion score of the model evaluated under the learning data.
     * @param model Gaussian mixture model in form of a joind distribution.
     */
    void runExpectationMaximization(const std::string& file,
				    unsigned int nc,
				    unsigned int& nparams,
				    plFloat& llk,
				    plFloat& bic,
				    plJointDistribution& model);
    
    /**
     * The number of dimensions of the learning samples.
     */
    unsigned int mNumberDimensions;
    
    /**
     * The minimal, maximal and the full number of kernels.
     * We do EM learning of GMMs for kernels [min,max] and take the GMM with the highest likelihood.
     */
    unsigned int mNumberKernelsMin, mNumberKernelsMax, mNumberKernels;
    
    /**
     * The number of runs per kernel, used for picking the best model.
     */
    unsigned int mNumberOfRuns;
    
    /**
     * For every sample, n noised samples are generated to make orientation and position learning more tolerant.
     */
    unsigned int mNumberOfSyntheticSamples;
    
    /**
     * The intervals (position and orientation) for the sample relaxiation.
     */
    double mIntervalPosition, mIntervalOrientation;
    
    /**
     * Path to the orientation plots.
     */
    std::string mPathOrientationPlots;
    
    /**
     * The data to learn from.
     */
    std::vector<std::vector<double> > mData;
    
    /**
     * The learned GMM with the best BIC score.
     */
    plJointDistribution mBestGMM;
  };
}
