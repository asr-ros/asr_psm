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
#include "helper/ProbabilityTable.h"
#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A probability table that handles learning, queries and persistence. Table entries are queried not ba an index, but by a clear text object name.
   *
   */
  class MappedProbabilityTable {
  public:

    /**
     * Constructor.
     */
    MappedProbabilityTable();
    
    /**
     * Constructor.
     * 
     * @param pPt The data structure used to load the container from XML.
     */
    MappedProbabilityTable(boost::property_tree::ptree& pPt);
    
    /**
     * Destructor.
     */
    ~MappedProbabilityTable();
    
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
     * Initializes the table based on the mapping.
     * 
     * @param pRow The number of the row.
     */
    void initializeTable(unsigned int pRows);
    
    /**
     * Returns the probability for the given object type.
     * 
     * @param pRow The number of the row the probability should be read from.
     * @param pType The type of object to return the probability for.
     * @return Returns the probability for the given object type.
     */
    double getProbability(unsigned int pRow, std::string pType);
    
    /**
     * Adds an entry for the given object type.
     * 
     * @param pType The object type to add an entry for.
     */
    void add(std::string pType);
    
    /**
     * Adds a count for the object in the given row.
     * 
     * @param pRow The number of the row.
     * @param pType The object type to add an entry for.
     */
    void add(unsigned int pRow, std::string pType);
    
    /**
     * Sets the counter for the default class.
     * 
     * @param pRow The number of the row the default class counter should be set.
     * @param pCount The number of counts for the default class.
     */
    void setDefaultClassCounter(unsigned int pRow, double pCount);
    
    /**
     * Normalizes the given probability table so that the values sum up to one.
     */
    void normalize();
    
    /**
     * Returns the number of columns in the probability table.
     */
    unsigned int getNumberOfColumns();
    
    /**
     * Returns the number of rows in the probability table.
     */
    unsigned int getNumberOfRows();
    
  private:
    
    /**
     * Returns the index for the given object type or zero, if not in list.
     * 
     * @return Returns the index for the given object type or zero.
     */
    unsigned int getIndex(std::string pType);
    
    /**
     * The probability table.
     */
    boost::shared_ptr<ProbabilityTable> mTable;
    
    /**
     * Mapping from human readable object type descriptions (from the ros messages) a probability table index.
     */
    std::map<std::string, unsigned int> mMappingTypeToIndice;
  };
}
