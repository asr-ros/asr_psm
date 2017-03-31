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

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <topology_generator/TopologyGenerator.h>

#include "learner/foreground/ocm/combinatorial_optimization/Evaluator.h"

namespace ProbabilisticSceneRecognition {
/**
 * Generates random evidence test sets.
 */
class TestSetGenerator {

public:
    /**
     * Constructor
     * @param pEvaluator                            evaluator used to validate test sets.
     * @param pObjectTypes                          types of objects appearing in test sets (once each).
     * @param pFullyMeshedTopology                  fully meshed topology used to validate test sets.
     * @param pObjectMissingInTestSetProbability    probability with which an object is missing from a test set.
     */
    TestSetGenerator(boost::shared_ptr<Evaluator> pEvaluator, const std::vector<std::string>& pObjectTypes, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology,
                     double pObjectMissingInTestSetProbability):
        mEvaluator(pEvaluator), mTypes(pObjectTypes), mFullyMeshedTopology(pFullyMeshedTopology), mObjectMissingInTestSetProbability(pObjectMissingInTestSetProbability) { }

    /**
     * Desctructor.
     */
    ~TestSetGenerator() { }

    /**
     * Generate and validate random test sets. Set evaluator's sets accordingly.
     * @param pExamplesList list of object observations serving as basis of the test sets.
     * @param pTestSetCount how many test sets to generate.
     */
    void generateTestSets(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList, unsigned int pTestSetCount);

private:

    /**
     * The test sets which represent the considered scene.
     */
    std::vector<std::vector<asr_msgs::AsrObject>> mValidTestSets;
    /**
     * The test sets that resemble but do not represent the considered scene.
     */
    std::vector<std::vector<asr_msgs::AsrObject>> mInvalidTestSets;

    /**
     * Generate random test sets.
     * @param pExamplesList list of object observations serving as basis of the test sets.
     * @param pTestSetCount how many test sets to generate.
     */
    std::vector<std::vector<asr_msgs::AsrObject>> generateRandomSets(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList, unsigned int pTestSetCount);
    /**
     * Validate whether test sets represent scene.
     * @param pTestSets the test sets to validate.
     */
    void validateSets(std::vector<std::vector<asr_msgs::AsrObject>> pTestSets);

    /**
     * Transform asr_msgs::AsrObservation into asr_msgs::AsrObject.
     * @param pObservation  the observation to transform.
     * @return the AsrObject the observation was transformed into.
     */
    asr_msgs::AsrObject makeAsrObject(asr_msgs::AsrObservations pObservation);
    /**
     * Set the pose of a given object relative to a reference object.
     * @param pObject       the object to have its pose transformed.
     * @param pReference    the reference object.
     */
    void setPoseOfObjectRelativeToReference(asr_msgs::AsrObject& pObject, const asr_msgs::AsrObject& pReference);

    /**
     * Evaluator used to validate test sets.
     */
    boost::shared_ptr<Evaluator> mEvaluator;
    /**
     * Types of objects appearing in test sets (once each).
     */
    std::vector<std::string> mTypes;
    /**
     * Fully meshed topology used to validate test sets.
     */
    boost::shared_ptr<SceneModel::Topology> mFullyMeshedTopology;
    /**
     * Probability with which an object is missing from a test set.
     */
    double mObjectMissingInTestSetProbability;
};

}
