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

#include "learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.h"

namespace ProbabilisticSceneRecognition {

/**
 * Generates test sets by randomly picking poses for each object from different points in time on the given trajectories.
 * Uses the absolute pose of a random reference object and the poses relative to that object for all others.
 */
class ReferenceTestSetGenerator: public TestSetGenerator {

public:
    /**
     * Constructor
     * @param pEvaluator                            evaluator used to validate test sets.
     * @param pObjectTypes                          types of objects appearing in test sets (once each).
     * @param pFullyMeshedTopology                  fully meshed topology used to validate test sets.
     */
    ReferenceTestSetGenerator(boost::shared_ptr<AbstractTopologyEvaluator> pEvaluator, const std::vector<std::string>& pObjectTypes, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology);

    /**
     * Destructor.
     */
    ~ReferenceTestSetGenerator();

private:
    /**
     * Generate random test sets.
     * @param pExamplesList list of object observations serving as basis of the test sets.
     * @param pTestSetCount number of test sets to generate.
     */
    virtual std::vector<boost::shared_ptr<TestSet>> generateRandomSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount);

};

}
