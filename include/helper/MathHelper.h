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
#include <cmath>
#include <vector>

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <pbd_msgs/PbdNode.h>
#include <pbd_msgs/PbdObject.h>

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
     * Copies the content of a vector from ProBT to Eigen.
     * 
     * @param pFrom The ProBt vector to copy from.
     * @param pTo The Eigen vector to copy to.
     */
    static void copy(plFloatVector& pFrom, boost::shared_ptr<Eigen::VectorXd>& pTo);

    /**
     * Copies the content of a matrix from ProBT to Eigen.
     * 
     * @param pFrom The ProBt matrix to copy from.
     * @param pTo The Eigen matrix to copy to.
     */
    static void copy(plFloatMatrix& pFrom, boost::shared_ptr<Eigen::MatrixXd>& pTo);
    
  };
}
