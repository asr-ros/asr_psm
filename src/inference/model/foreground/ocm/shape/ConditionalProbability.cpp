/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "inference/model/foreground/ocm/shape/ConditionalProbability.h"

namespace ProbabilisticSceneRecognition {

ConditionalProbability::ConditionalProbability(): mWasRead(false)
{ }

ConditionalProbability::~ConditionalProbability()
{ }

void ConditionalProbability::setProbability(std::string pParentId, double pProbability)
{
    if (mWasRead) throw std::runtime_error("In ConditionalProbability::setProbability(" + pParentId + ", "
                                           + std::to_string(pProbability) + "): trying to add to a probability that has already been read.");
    mParentProbabilities[pParentId] = pProbability; // overwrites older probabilities.
}

double ConditionalProbability::getProbability()
{
    mWasRead = true;

    // to increase speed: if there is only one parent anyways, return its probability immediately.
    if (mParentProbabilities.size() == 1)
        return mParentProbabilities.begin()->second;

    if (mParentProbabilities.empty())
        throw std::runtime_error("In MinimumConditionalProbability::getProbability(): trying to access probability that has not been set.");

    return calculateProbability();
}

std::string ConditionalProbability::printParentProbabilities()
{
    std::string result;
    for (std::pair<std::string, double> parentProbability: mParentProbabilities)
        result += "(" + parentProbability.first + ", " + std::to_string(parentProbability.second) + ")";
    return result;
}

}

