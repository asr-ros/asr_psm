/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/TestSetSelection.h"

namespace ProbabilisticSceneRecognition {

    void TestSetSelection::removeUnusableTestSets(double& pMinValidProbability, double& pMaxInvalidProbability)
    {
        double minValidProbability;
        double maxInvalidProbability;
        unsigned int size = mEvaluator->getValidTestSets().size() + mEvaluator->getInvalidTestSets().size();
        for (unsigned int counter = 0; counter < size; counter++)
        {
            minValidProbability = 1.0;
            unsigned int minValidIndex = mEvaluator->getValidTestSets().size();
            for (unsigned int i = 0; i < mEvaluator->getValidTestSets().size(); i++)
            {
                double validProbability = mEvaluator->getValidTestSets()[i]->mFullyMeshedProbability;
                if (validProbability <= minValidProbability)
                {
                    minValidProbability = validProbability;
                    minValidIndex = i;
                }
            }
            if (minValidIndex == mEvaluator->getValidTestSets().size())
                throw std::runtime_error("In CombinatorialTrainer::initFullyMeshedTopology(): No minimum valid probability could be found.");

            maxInvalidProbability = 0.0;
            unsigned int maxInvalidIndex = mEvaluator->getInvalidTestSets().size();
            for (unsigned int i = 0; i < mEvaluator->getInvalidTestSets().size(); i++) {
                double invalidProbability = mEvaluator->getInvalidTestSets()[i]->mFullyMeshedProbability;
                if (invalidProbability >= maxInvalidProbability)
                {
                    maxInvalidProbability = invalidProbability;
                    maxInvalidIndex = i;
                }
            }
            if (maxInvalidIndex == mEvaluator->getInvalidTestSets().size())
                throw std::runtime_error("In CombinatorialTrainer::initFullyMeshedTopology(): No maximum invalid probability could be found.");

            mPrintHelper.printAsHeader("Minimum valid probability: " + boost::lexical_cast<std::string>(minValidProbability) + ", Maximum invalid probability: " + boost::lexical_cast<std::string>(maxInvalidProbability));

            double below = 0;
            for (boost::shared_ptr<TestSet> valid: mEvaluator->getValidTestSets())
            {
                double validProbability = valid->mFullyMeshedProbability;
                if (validProbability < maxInvalidProbability)
                    below++;
            }

            double above = 0.0;
            for (boost::shared_ptr<TestSet> invalid: mEvaluator->getInvalidTestSets())
            {
                double invalidProbability = invalid->mFullyMeshedProbability;
                if (invalidProbability > minValidProbability)
                    above++;
            }

            ROS_INFO_STREAM("Valid probabilities below maximum invalid probability: " << below << "/" << mEvaluator->getValidTestSets().size());
            ROS_INFO_STREAM("Invalid probabilities above minimum valid probability: " << above << "/" << mEvaluator->getInvalidTestSets().size());

            if (minValidProbability <= maxInvalidProbability)
            {
                ROS_INFO_STREAM("Minimum valid probability smaller than maximum invalid probability.");

                if (below/((double) mEvaluator->getValidTestSets().size()) > above/((double) mEvaluator->getInvalidTestSets().size()))
                {
                    ROS_INFO_STREAM("Ratio of valid probabilities worse. Removing maximum invalid probability.");
                    mEvaluator->eraseInvalidTestSet(maxInvalidIndex);
                }
                else
                {
                    ROS_INFO_STREAM("Ratio of invalid probabilities worse. Removing minimum valid probability.");
                    mEvaluator->eraseValidTestSet(minValidIndex);
                }
            }
            else
            {
                ROS_INFO_STREAM("All unusable test sets removed after " << counter << "/" << size << " steps.");
                ROS_INFO_STREAM("Test sets remanining: " << mEvaluator->getValidTestSets().size() << " valid, " << mEvaluator->getInvalidTestSets().size() << " invalid.");
                break;
            }
        }
        // in the worst case scenario, all test sets from one of the two lists have been removed (or one was empty to begin with):
        if (mEvaluator->getValidTestSets().empty() || mEvaluator->getInvalidTestSets().empty())
            ROS_INFO_STREAM("NOTE: One of the two test set lists is empty. This will impede performance.");

        if (mEvaluator->getValidTestSets().empty() && mEvaluator->getInvalidTestSets().empty())
            throw std::runtime_error("After TestSetSelection::removeUnusableTestSets(): Both test sets are now empty.");

        // Set output parameters:
        pMinValidProbability = minValidProbability;
        pMaxInvalidProbability = maxInvalidProbability;
    }

}

