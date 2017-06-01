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

#include "learner/foreground/ocm/combinatorial_optimization/AbstractEvaluator.h"
#include "helper/PrintHelper.h"

namespace ProbabilisticSceneRecognition {

/**
 * This class can be used to filter out test sets loaded from files that are classified as valid or invalid incorrectly considering the Probabilistic Scene Model.
 */
class TestSetSelection {

public:

    /**
     * Constructor.
     * @param pEvaluator    the Evaluator that contains the test sets that should be filtered.
     */
    TestSetSelection(boost::shared_ptr<AbstractEvaluator> pEvaluator):
        mEvaluator(pEvaluator), mPrintHelper('=')
    { }

    /**
     * Destructor.
     */
    ~TestSetSelection() { }

    /**
     * Removes unusable test sets from the evaluator.
     * Test sets are considered unusable if they were loaded as valid or invalid but their probability when tested against the fully meshed topology
     * was either lower than the highest invalid probability (in case of valid test sets) or higher than the smalles valid one.
     * @param pMinValidProbability      the minimum probability of the valid test sets after the unsusable ones were removed.
     * @param pMaxInvalidProbability    the maximum probability of the invalid test sets after the unsusable ones were removed.
     */
    void removeUnusableTestSets(double& pMinValidProbability, double& pMaxInvalidProbability);

private:

    /**
     * The evaluator contains the test sets that should be filtered.
     */
    boost::shared_ptr<AbstractEvaluator> mEvaluator;

    /**
     * Helper used to print information as headers, marked with special dividers.
     */
    PrintHelper mPrintHelper;


};

}
