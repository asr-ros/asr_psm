/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/AbstractEvaluator.h"

namespace ProbabilisticSceneRecognition {

    void AbstractEvaluator::setValidTestSets(const std::vector<boost::shared_ptr<TestSet>>& pValidTestSets)
    {
        mValidTestSets = pValidTestSets;
    }

    std::vector<boost::shared_ptr<TestSet>> AbstractEvaluator::getValidTestSets()
    {
        return mValidTestSets;
    }

    void AbstractEvaluator::setInvalidTestSets(const std::vector<boost::shared_ptr<TestSet>>& pInvalidTestSets)
    {
        mInvalidTestSets = pInvalidTestSets;
    }

    std::vector<boost::shared_ptr<TestSet>> AbstractEvaluator::getInvalidTestSets()
    {
        return mInvalidTestSets;
    }

    void AbstractEvaluator::setRecognitionThreshold(double pRecognitionThreshold)
    {
        mRecognitionThreshold = pRecognitionThreshold;
    }

    double AbstractEvaluator::getRecognitionThreshold()
    {
        return mRecognitionThreshold;
    }

    void AbstractEvaluator::eraseValidTestSet(unsigned int pIndex)
    {
        if (pIndex >= mValidTestSets.size())
            throw std::runtime_error("In AbstractEvaluator::eraseValidTestSet(): index " + boost::lexical_cast<std::string>(pIndex) + " out of bounds.");
        mValidTestSets.erase(mValidTestSets.begin() + pIndex);
    }

    void AbstractEvaluator::eraseInvalidTestSet(unsigned int pIndex)
    {
        if (pIndex >= mInvalidTestSets.size())
            throw std::runtime_error("In AbstractEvaluator::eraseInvalidTestSet(): index " + boost::lexical_cast<std::string>(pIndex) + " out of bounds.");
        mInvalidTestSets.erase(mInvalidTestSets.begin() + pIndex);
    }

}

