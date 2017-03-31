/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/WeightedSum.h"

namespace ProbabilisticSceneRecognition {

// very similar to lib_ism:
double WeightedSum::calculateCost(boost::shared_ptr<SceneModel::Topology> instance)
{
    if (!instance->mEvaluated) throw std::runtime_error("In WeightedSum::calculateCost(): given topology (" + instance->mIdentifier + ") has not been evaluated yet.");
    std::string fromEarlier = "";
    if (instance->mCostValid) fromEarlier = " (from earlier)";
    else
    {
        //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        //std::cout << "false positives: [" << mMinFalsePositives << "," << mMaxFalsePositives << "]. avg runtime: [" << mMinAverageRecognitionRuntime << "," << mMaxAverageRecognitionRuntime << "]." << std::endl;

        double normalisedFalsePositives = getNormalisedFalsePositives(instance->mFalsePositives);
        double normalisedAverageRecognitionRuntime = getNormalisedAverageRecognitionRuntime(instance->mAverageRecognitionRuntime);

        //std::cout << "Normalised false positives: " << normalisedFalsePositives << " (" << instance->mFalsePositives << ")" << std::endl;
        //std::cout << "Normalised recognition runtime: " << normalisedAverageRecognitionRuntime << " (" << instance->mAverageRecognitionRuntime << ")" << std::endl;

        double cost;
        //std::cout << "cost";
        if (normalisedFalsePositives < 0 || normalisedAverageRecognitionRuntime < 0)
        {
            cost = std::numeric_limits<double>::infinity();
        }
        else
        {
            //std::cout << " = " << mAlpha << " * " << normalisedFalsePositives << " + " << mBeta << " * " << normalisedAverageRecognitionRuntime;
            cost = mAlpha * normalisedFalsePositives + mBeta * normalisedAverageRecognitionRuntime;
        }

        instance->mCost = cost;
        instance->mCostValid = true;
    }
    //std::cout << " = " << instance->mCost << std::endl;
    ROS_INFO_STREAM("Cost of topology " << instance->mIdentifier << " is " << instance->mCost << fromEarlier);

    return instance->mCost;
}

double WeightedSum::getNormalisedFalsePositives(unsigned int falsePositives)
{
    if (falsePositives > mMaxFalsePositives)
    {
        //False Postivies are higher than the false positves of the worst star topology.
        return -1;
    }

    if (mMaxFalsePositives == mMinFalsePositives)
    {
        return 0;
    }

    return ((double) (falsePositives - mMinFalsePositives)) / ((double) (mMaxFalsePositives - mMinFalsePositives));
}


double WeightedSum::getNormalisedAverageRecognitionRuntime(double averageRecognitionRuntime)
{
    if (averageRecognitionRuntime > mMaxAverageRecognitionRuntime)
    {
        //Average recognition runtime is longer than the average recognition runtime of the fully meshed topology.
        return -1;
    }

    if (mMaxAverageRecognitionRuntime == averageRecognitionRuntime)
    {
        //return 0;
        return 1;
    }

    return (averageRecognitionRuntime - mMinAverageRecognitionRuntime) /
        (mMaxAverageRecognitionRuntime - mMinAverageRecognitionRuntime);
}

}
