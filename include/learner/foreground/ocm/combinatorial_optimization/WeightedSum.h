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

#include <ros/ros.h>

#include <ISM/combinatorial_optimization/CostFunction.hpp>
#include <topology_creator/Topology.h>

namespace ProbabilisticSceneRecognition {

/**
 * Calculates the cost of a topology as a weighted sum of its number of false positives and average recognition runtime on test sets.
 */
class WeightedSum: public ISM::CostFunction<boost::shared_ptr<SceneModel::Topology>> {

    // similar to lib_ism/WeightedSum:
public:
    /**
     * Constructor.
     * @param minFalsePositives             Minimum number of false positives for normalization.
     * @param maxFalsePositives             Maximum number of false positives for normalization.
     * @param minAverageRecognitionRuntime  Minimum average recognition runtime for normalization.
     * @param maxAverageRecognitionRuntime  Maximum average recognition runtime for normalization.
     * @param alpha                         Weight of the normalized number of false positives.
     * @param beta                          Weight of the normalized average recognition runtime.
     * @param gamma                         Weight of the normalized number of false negatives.
     */
    WeightedSum(unsigned int minFalsePositives, unsigned int maxFalsePositives,
                unsigned int minFalseNegatives, unsigned int maxFalseNegatives,
            double minAverageRecognitionRuntime, double maxAverageRecognitionRuntime,
            double alpha, double beta, double gamma)
        : mMinFalsePositives(minFalsePositives)
        , mMaxFalsePositives(maxFalsePositives)
        , mMinFalseNegatives(minFalseNegatives)
        , mMaxFalseNegatives(maxFalseNegatives)
        , mMinAverageRecognitionRuntime(minAverageRecognitionRuntime)
        , mMaxAverageRecognitionRuntime(maxAverageRecognitionRuntime)
        , mAlpha(alpha)
        , mBeta(beta)
        , mGamma(gamma)
    {}

    /**
     * Destructor.
     */
    ~WeightedSum() { }

    /**
     * Calculate the cost of the topology as weighted sum of its number of false positives and average recognition runtime on test sets.
     * Stores the cost in the topology.
     * @param instance  the topology to calculate the cost for.
     * @return the cost of the topology.
     */
    double calculateCost(boost::shared_ptr<SceneModel::Topology> instance);

private:
    /**
     * Minimum number of false positives for normalization.
     */
    unsigned int mMinFalsePositives;
    /**
     * Maximum number of false positives for normalization.
     */
    unsigned int mMaxFalsePositives;

    /**
     * Minimum number of false negatives for normalization.
     */
    unsigned int mMinFalseNegatives;

    /**
     * Maximum number of false negatives for normalization.
     */
    unsigned int mMaxFalseNegatives;

    /**
     * Minimum average recognition runtime for normalization.
     */
    double mMinAverageRecognitionRuntime;
    /**
     * Maximum average recognition runtime for normalization.
     */
    double mMaxAverageRecognitionRuntime;

    /**
     * Weight of the normalized number of false positives.
     */
    double mAlpha;
    /**
     * Weight of the normalized average recognition runtime.
     */
    double mBeta;
    /**
     * Weight of the normalized number of false negatives.
     */
    double mGamma;

    /**
     * Normalize number of false positives.
     * @param falsePositives    the number of false positives.
     * @return the normalized number of false positives.
     */
    double getNormalisedFalsePositives(unsigned int falsePositives);

    /**
     * Normalize number of false negatives.
     * @param falsePositives    the number of false negatives.
     * @return the normalized number of false negatives.
     */
    double getNormalisedFalseNegatives(unsigned int falseNegatives);

    /**
     * Normalize average recognition runtime.
     * @param averageRecognitionRuntime    the average recognition runtime.
     * @return the normalized average recognition runtime.
     */
    double getNormalisedAverageRecognitionRuntime(double averageRecognitionRuntime);

};

}
