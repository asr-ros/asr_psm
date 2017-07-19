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
#include <string>
#include <vector>
#include <math.h>

// Package includes
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/foreground/ForegroundInferenceAlgorithm.h"

#include <ISM/common_type/Object.hpp>


#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/RecordedPattern.hpp>
#include <Eigen/Geometry>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Implementation of the abstract ForegroundInferenceAlgorithm class. It evaluates all foreground scene objects and takes the one with the best score.
   *
   * @author Joshua Link
   * @version See SVN
   */
  class DifferenceForegroundInferenceAlgorithm : public ForegroundInferenceAlgorithm {
  public:
    
    /**
     * Constructor.
     * 
     * @param pSceneObjects The scene objects associated with this foreground scene.
     */
    DifferenceForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects, std::string pDataBaseName);
    
    /**
     * Destructor.
     */
    ~DifferenceForegroundInferenceAlgorithm();
    
    /**
     * Executes the inference based on the given evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void doInference(std::vector<ISM::Object> pEvidenceList, std::ofstream& pRuntimeLogger);

    /**
     * returns the difference between the constellation of pRoot-pTar and pDiffRoot-pDiffTar
     */
    double differenceBetween(ISM::Object pRoot, ISM::Object pTar, ISM::Object pDiffRoot, ISM::Object pDiffTar);


    /**
     * Executes the inference based on the given evidence.
     *
     * @param pList A list containing all objects to search in.
     * @param pTypeAndObservedId A combination of type and id.
     */
    ISM::Object findObjectOfType(std::vector<ISM::Object> pList, std::string pTypeAndObservedId);

    /**
     * Normalizes a vector.
     *
     * @param input vector to get normalized
     */
    void normalizeVector3d(Eigen::Vector3d input);

    
    /**
     * Returns the probability calculated by the inference process.
     * 
     * @return Probability for this scene.
     */
    double getProbability();
    
  private:
    
    /**
     * The probability calculated by this algorithm.
     */
    double mProbability;

    /**
     * The database name of samples of the scene.
     */
    std::string mDataBaseName;

    /**
     * The pattern name of the scene.
     */
    std::string patternName;

    /**
     * TableHelper to extract Objects from ".sqlite"-file.
     */
    boost::shared_ptr<ISM::TableHelper> tableHelper;
  };
}
