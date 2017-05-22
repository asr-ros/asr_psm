/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <cmath>
#include <vector>

// Package includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <asr_msgs/AsrObject.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Helper class for mathematical calculations.
   *
   */
  class MathHelper {
  public:
    /**
     * Constructor.
     */
    MathHelper();
    
    /**
     * Destructor.
     */
    ~MathHelper();

    /**
     * Draws a sample from a multivariate normal distribution represented by a mean and a covariance matrix
     *
     * @param mean      The mean of the distribution
     * @param cov       The diagonal covariance matrix of the distribution
     * @param amount    The amount of samples to draw
     * @param sample    The Eigen vector to draw the sample into
     */
    static void drawNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov, unsigned int amount, std::vector<Eigen::VectorXd>& samples);

    /**
     * Calculate Histogram
     * @param lower     Lower bound of the values
     * @param upper     Upper bound of the values
     * @param buckets   Number of buckets
     * @param in        Input samples
     * @param out       Output vector
     */
    static void calcHistogram(double lower, double upper, unsigned int buckets, std::vector<double> in, std::vector<std::pair<double, double>>& out);

  };
}
