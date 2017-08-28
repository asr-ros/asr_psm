/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <trainer/TopologyTreeTrainer.h>
#include <topology_creator/Topology.h>

#include <boost/property_tree/xml_parser.hpp>

#include <ISM/common_type/ObjectSet.hpp>

#include "learner/foreground/ForegroundSceneLearner.h"
#include "learner/foreground/ocm/SceneObjectLearner.h"
#include "learner/foreground/ocm/combinatorial_optimization/TestSet.h"

#include "inference/model/foreground/ForegroundSceneContent.h"

namespace ProbabilisticSceneRecognition {

/**
 * Tests learned models of topologies against valid and invalid test sets.
 */
class AbstractTopologyEvaluator {
public:

    /**
     * Evaluate model learned on topology against test sets, write results to topology.
     * @param pTopology to evaluate.
     * @param pFullyMeshed  whether the topology to evaluate is the fully meshed one.
     * @return  whether a new evaluation was neccessary.
     */
    virtual bool evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, bool pFullyMeshed = false) = 0;

    /**
     * Set valid test sets.
     * @param pValidTestSets    valid test sets to set.
     */
    void setValidTestSets(const std::vector<boost::shared_ptr<TestSet>>& pValidTestSets);

    /**
     * Get valid test sets.
     * @return the valid test sets.
     */
    std::vector<boost::shared_ptr<TestSet>> getValidTestSets();

    /**
     * Set invalid test sets.
     * @param pInvalidTestSets  invalid test sets to set.
     */
    void setInvalidTestSets(const std::vector<boost::shared_ptr<TestSet>>& pInvalidTestSets);

    /**
     * Get invalid test sets.
     * @return the invalid test sets.
     */
    std::vector<boost::shared_ptr<TestSet>> getInvalidTestSets();

    /**
     * Set recognition threshold.
     * @param pRecognitionThreshold recognition threshold to set.
     */
    void setRecognitionThreshold(double pRecognitionThreshold);

    /**
     * Get recognition threshold.
     * @return the recognition threshold.
     */
    double getRecognitionThreshold();

    /**
     * erase the Valid Test Set at the given index.
     * @param pIndex    index of the valid test set to erase.
     */
    void eraseValidTestSet(unsigned int pIndex);

    /**
     * erase the invalid Test Set at the given index.
     * @param pIndex    index of the invalid test set to erase.
     */
    void eraseInvalidTestSet(unsigned int pIndex);

protected:

    /**
     * The test sets which represent the considered scene.
     */
    std::vector<boost::shared_ptr<TestSet>> mValidTestSets;
    /**
     * The test sets that resemble but do not represent the considered scene.
     */
    std::vector<boost::shared_ptr<TestSet>> mInvalidTestSets;


    /**
     * Threshold above which (>) a probability is seen as high enough to represent a scene has been recognized.
     */
    double mRecognitionThreshold;

};

}
