/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/ProbabilityTable.h"

namespace ProbabilisticSceneRecognition {
  
  ProbabilityTable::ProbabilityTable()
  {
  }
  
  ProbabilityTable::ProbabilityTable(unsigned int pRows, unsigned int pColums)
  {
    // Reserve space.
    for(unsigned int y = 0; y < pRows; y++) {
      mEntries.push_back(std::vector<double>());
      for(unsigned int x = 0; x < pColums; x++)
	mEntries[y].push_back(0);
    }
  }
  
  ProbabilityTable::ProbabilityTable(boost::property_tree::ptree& pPt)
  {
    load(pPt);
  }
  
  ProbabilityTable::~ProbabilityTable()
  {
    
  }
  
  void ProbabilityTable::load(boost::property_tree::ptree& pPt)
  {
    // Load the probability table.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pPt.get_child("table"))
    {
      // Only access the 'map' child nodes.
      if(!std::strcmp(v.first.c_str(), "entry"))
      {	
	// Load the string containing the values.
	std::string valuesAsString = v.second.get<std::string>("<xmlattr>.values");

	// Reconstruct a vector of doubles from the string.
	std::vector<double> line;
	SerializationHelper::convertStringToVector(valuesAsString, line);
	
	// Add line to table.
	mEntries.push_back(line);
      }
    }
  }

  void ProbabilityTable::save(boost::property_tree::ptree& pPt)
  {
    // Create a subtree.
    boost::property_tree::ptree subtree;
    
    // Export every line to XML.
    for(unsigned int i = 0; i < mEntries.size(); i++)
    {
      // Convert entries into a string.
      std::string lineAsString;
      SerializationHelper::convertVectorToString(mEntries[i], lineAsString);
      
      // Add value attribute to a new subtree for the entry.
      boost::property_tree::ptree subtreeEntry;
      subtreeEntry.add("<xmlattr>.values", lineAsString);
      subtree.add_child("entry", subtreeEntry);
    }
    
    // Add subtree to main tree.
    pPt.add_child("table", subtree);
  }
  
  double ProbabilityTable::getProbability(unsigned int pRow, unsigned int pColumn)
  {
    // Check, for row boundaries.
    if(!(pRow < mEntries.size()))
      throw std::invalid_argument("Error in probability table. Row out of boundaries.");
    
    // Check, for column boundaries.
    if(!(pColumn < mEntries[pRow].size()))
      throw std::invalid_argument("Error in probability table. Column out of boundaries.");
    
    // Return the probability
    return mEntries[pRow][pColumn];
  }
  
  unsigned int ProbabilityTable::getNumberOfColumns()
  {
    return mEntries[0].size();
  }
  
  unsigned int ProbabilityTable::getNumberOfRows()
  {
    return mEntries.size();
  }
  
  void ProbabilityTable::add(unsigned int pRow, unsigned int pColumn, unsigned int pCounts)
  {
    // Check, for row boundaries.
    if(!(pRow < mEntries.size()))
      throw std::invalid_argument("Error in probability table. Row out of boundaries.");
    
    // Check, for column boundaries.
    if(!(pColumn < mEntries[pRow].size()))
      throw std::invalid_argument("Error in probability table. Column out of boundaries.");
    
    // Inrement counter.
    mEntries[pRow][pColumn] =  mEntries[pRow][pColumn] + pCounts;
  }
  
  void ProbabilityTable::normalize()
  {
    // Iterate over all rows.
    for(unsigned int i = 0; i < mEntries.size(); i++)
    {
      // Sum all colums.
      double sum = 0;
      BOOST_FOREACH(double d, mEntries[i])
	sum += d;
      
      // Use sum from above for normalization.
      for(unsigned int j = 0; j < mEntries[i].size(); j++)
	mEntries[i][j] /= sum;
    }
  }
  
}
