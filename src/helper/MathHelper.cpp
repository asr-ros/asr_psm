/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/MathHelper.h"
#include <random>

namespace ProbabilisticSceneRecognition {
 
  MathHelper::MathHelper()
  {
  }
  
  MathHelper::~MathHelper()
  {
  }

  void MathHelper::drawNormal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov, unsigned int amount, std::vector<Eigen::VectorXd>& samples)
  {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
      Eigen::MatrixXd t = solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal();
      std::default_random_engine generator(0);  // set fixed seed so the same data gets plotted the same way every time
      std::normal_distribution<double> distr;
      samples.resize(amount);
      for (unsigned int i = 0; i < amount; i++) {
          Eigen::VectorXd randomvect(mean.size());
          for (unsigned int i = 0; i < randomvect.size(); i++) randomvect[i] = distr(generator);
          samples.at(i) = mean + t * randomvect;
      }
  }

  void MathHelper::calcHistogram(double lower, double upper, unsigned int buckets, std::vector<double> in, std::vector<std::pair<double, double>>& out)
  {
      out.resize(buckets);
      double range = std::abs(lower) + std::abs(upper);
      double bucketsize = range / buckets;

      for (unsigned int i = 0; i < buckets; i++)
      {
          out.at(i).first = lower + (i * bucketsize);
          out.at(i).second = 0;
      }
      for (double sample: in)
      {
          if (lower <= sample && sample <= upper) {
              unsigned int bucket = ((double) (sample - lower)) / bucketsize;
              if (0 <= bucket && bucket < buckets) out.at(bucket).second++;
          }
      }
  }
  
}
