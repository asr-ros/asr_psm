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
#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Helper class for converting primitive datatypes into string representations and vice versa.
   *
   */
  class SerializationHelper {
  public:
    
    /**
     * Constructor.
     */
    SerializationHelper();
    
    /**
     * Destructor.
     */
    ~SerializationHelper();
    
    /**
     * Converts a vector of doubles into a string.
     * 
     * @param vec The vector to convert to string.
     * @param str The string to fill with the content of the vector.
     */
    static void convertVectorToString(std::vector<double>& vec, std::string& str);
    
    /**
     * Converts a vector of doubles into a string.
     * 
     * @param str The string to convert to a vector.
     * @param vec The vector to fill with the content of the string.
     */
    static void convertStringToVector(std::string& str, std::vector<double>& vec);
    
    /**
     * Converts an Eigen vector into a string.
     * 
     * @param vec The vector to convert to string.
     * @param str The string to fill with the content of the vector.
     */
    static void convertVectorToString(boost::shared_ptr<Eigen::VectorXd>& vec, std::string& str);
    
    /**
     * Converts a string into an Eigen vector.
     * 
     * @param size The size of the vector.
     * @param str The string to convert to a vector.
     * @param vec The vector to fill with the content of the string.
     */
    static void convertStringToVector(unsigned int size, std::string& str, boost::shared_ptr<Eigen::VectorXd>& vec);
    
    /**
     * Converts am Eigen matrix into a string.
     * 
     * @param mat The Eigen matrix to convert to string.
     * @param str The string to fill with the content of the matrix.
     */
    static void convertMatrixToString(boost::shared_ptr<Eigen::MatrixXd>& mat, std::string& str);
    
    /**
     * Converts a string into an Eigen matrix.
     * 
     * @param size The size of rows and colums of the matrix.
     * @param str The string to convert to an Eigen matrix.
     * @param mat The matrix to fill with the content of the string.
     */
    static void convertStringToMatrix(unsigned int size, std::string& str, boost::shared_ptr<Eigen::MatrixXd>& mat);
  };
}
