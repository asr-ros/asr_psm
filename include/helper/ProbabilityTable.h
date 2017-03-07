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
#include <map>
#include <string>
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

// Local includes
#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A probability table that handles learning, queries and persistence.
   *
   */
  class ProbabilityTable {
  public:

    /**
     * Constructor.
     */
    ProbabilityTable();
    
    /**
     * Constructor.
     * 
     * @param pRows The number of rows.
     * @param pColums The number of columns.
     */
    ProbabilityTable(unsigned int pRows, unsigned int pColums);
    
    /**
     * Constructor.
     * 
     * @param pPt The data structure used to load the container from XML.
     */
    ProbabilityTable(boost::property_tree::ptree& pPt);
    
    /**
     * Destructor.
     */
    ~ProbabilityTable();
    
    /**
     * Loads the content from XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Saves the content to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void save(boost::property_tree::ptree& pPt); 
    
    /**
     * Returns the probability for the given object type.
     * 
     * @param pRow The number of the row.
     * @param pColumn The number of the column.
     * @return Returns the probability for the given entry.
     */
    double getProbability(unsigned int pRow, unsigned int pColumn);
    
    /**
     * Returns the number of columns in the probability table.
     */
    unsigned int getNumberOfColumns();
    
    /**
     * Returns the number of rows in the probability table.
     */
    unsigned int getNumberOfRows();
    
    /**
     * Adds the given number of counts to the given entry.
     * 
     * @param pRow The number of the row.
     * @param pColumn The number of the column.
     * @param pCounts The number of counts to add.
     */
    void add(unsigned int pRow, unsigned int pColumn, unsigned int pCounts);
    
    /**
     * Normalizes the given probability table so that the values sum up to one.
     */
    void normalize();
    
  private:
    
    /**
     * The entries of the table. To be a valid probability distribution, they must sum up to one.
     */
    std::vector<std::vector<double> > mEntries;
  };
}
