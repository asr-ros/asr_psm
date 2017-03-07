/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {
 
  SerializationHelper::SerializationHelper()
  {
  }
  
  SerializationHelper::~SerializationHelper()
  {
  }
  
  void SerializationHelper::convertVectorToString(std::vector<double>& vec, std::string& str)
  {
    // Concatenate the substrings containing the vector elements.
    BOOST_FOREACH(double value, vec)
    {
      str += boost::lexical_cast<std::string>(value) + std::string(" ");
    }
    
    // Trim trailing whitespace.
    boost::trim(str);
  }
  
  void SerializationHelper::convertStringToVector(std::string& str, std::vector<double>& vec)
  {
    // Erase trailing whitespaces (or a phantom value consisting of a whitespace will appear).
    str.erase(str.find_last_not_of(" \n\r\t") + 1);
    
    // Split string into the substrings.
    std::vector<std::string> valuesAsSubstrings;    
    boost::split(valuesAsSubstrings, str, boost::is_any_of(" "));
    
    // Copy the substrings into the vector.
    for(unsigned int i = 0; i < valuesAsSubstrings.size(); i++)
    {
      vec.push_back(boost::lexical_cast<double>(valuesAsSubstrings[i]));
    }
  }
  
  void SerializationHelper::convertVectorToString(boost::shared_ptr<Eigen::VectorXd>& vec, std::string& str)
  {
    // Clear string.
    str = "";
    
    // Serialize the vector to string.
    for(int i = 0; i < vec->size(); i++)
      str += boost::lexical_cast<std::string>((*vec)(i)) + std::string(" ");
  }
  
  void SerializationHelper::convertStringToVector(unsigned int size, std::string& str, boost::shared_ptr<Eigen::VectorXd>& vec)
  {
    // Erase trailing whitespaces (or a phantom value consisting of a whitespace will appear).
    str.erase(str.find_last_not_of(" \n\r\t") + 1);
    
    // Split string into the substrings.
    std::vector<std::string> valuesAsSubstrings;
    boost::split(valuesAsSubstrings, str, boost::is_any_of(" "));

    // Check, if the number of elements is valid.
    if(valuesAsSubstrings.size() != size)
      throw std::invalid_argument("Error in serialization helper. Bad number of substrings.");
    
    // Reset boost pointer to vector.
    vec.reset(new Eigen::VectorXd(size)); 
    
    // Unserialize the matrix from string to matrix.
    for(unsigned int i = 0; i < size; i++)
      (*vec)(i) = boost::lexical_cast<double>(valuesAsSubstrings[i]);
  }
  
  void SerializationHelper::convertMatrixToString(boost::shared_ptr<Eigen::MatrixXd>& mat, std::string& str)
  {
    // Clear string.
    str = "";
    
    // Serialize the matrix to string.
    for(int y = 0; y < mat->rows(); y++)
      for(int x = 0; x < mat->cols(); x++)
	str += boost::lexical_cast<std::string>((*mat)(y, x)) + std::string(" ");
  }
  
  void SerializationHelper::convertStringToMatrix(unsigned int size, std::string& str, boost::shared_ptr<Eigen::MatrixXd>& mat)
  {
    // Erase trailing whitespaces (or a phantom value consisting of a whitespace will appear).
    str.erase(str.find_last_not_of(" \n\r\t") + 1);
    
    // Split string into the substrings.
    std::vector<std::string> valuesAsSubstrings;    
    boost::split(valuesAsSubstrings, str, boost::is_any_of(" "));
    
    // Check, if the number of elements is valid.
    if(valuesAsSubstrings.size() != pow(size, 2))
      throw std::invalid_argument("Error in serialization helper. Bad number of substrings.");
    
    // Reset boost pointer to vector.
    mat.reset(new Eigen::MatrixXd(size, size)); 
    
    // UNserialize the matrix from string to matrix.
    for(unsigned int y = 0; y < size; y++)
      for(unsigned int x = 0; x < size; x++)
	(*mat)(y, x) = boost::lexical_cast<double>(valuesAsSubstrings[y * size + x]);
  }
    
}
