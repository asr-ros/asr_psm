/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once
#include <stdexcept>
#include <map>

namespace ProbabilisticSceneRecognition {

/**
 * This class represents a conditional probability of an evidences pose given its parent evidences' poses.
 */
class ConditionalProbability {
public:

    /**
     * Constructor.
     */
    ConditionalProbability();

    /**
     * Destructor.
     */
    ~ConditionalProbability();

    /**
     * Add a value to the conditional probability.
     * @param pParentId the parent to set the probability for.
     * @param pProb the value of the conditional probability.
     */
    void setProbability(std::string pParentId, double pProbability);

    /**
     * Get the value of the conditional probability.
     * @return the value of the conditional probability.
     */
    double getProbability();

    /**
     * Returns the list of probabilities by parent ids as a string.
     * @return list of probabilities by parent ids as a string.
     */
    std::string printParentProbabilities();

protected:

    /**
     * Calculate the probability from the parent probabilities.
     * @return the probability calculated from the parent probabilities.
     */
    virtual double calculateProbability() = 0;

    /**
     * Probabilities by parent ids.
     */
    std::map<std::string, double> mParentProbabilities;

private:

    /**
     * Whether this probability has already been read once. Adding to a probability that has already been read is forbidden.
     */
    bool mWasRead;

};

}
