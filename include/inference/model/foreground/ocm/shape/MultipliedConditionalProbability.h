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
#include "inference/model/foreground/ocm/shape/ConditionalProbability.h"

namespace ProbabilisticSceneRecognition {

/**
 * This class represents a conditional probability of an evidences pose x given its parent evidence's pose p: P(x|p).
 * If there are several parents p1, p2, ..., it uses the multiplied probability of the separate conditional probabilities:
 * P(x|p1) * P(x|p2) * ...).
 */
class MultipliedConditionalProbability: public ConditionalProbability {
public:
    /**
     * Constructor.
     */
    MultipliedConditionalProbability(): mProbability(1.0), mIsSet(false), mWasRead(false)
    { }
    /**
     * Destructor.
     */
    ~MultipliedConditionalProbability()
    { }
    /**
     * Add a value of a conditional probability.
     * @param pProb the value of the conditional probability.
     */
    virtual void addProbability(double pProbability)
    {
        if (mWasRead) throw std::runtime_error("In MultipliedConditionalProbability::addProbability(): trying to add to probability that has already been read once.");
        mProbability *= pProbability;
        mIsSet = true;
    }
    /**
     * Get the multiplied value of the conditional probabilities.
     * @return the multiplied value of the conditional probabilities.
     */
    virtual double getProbability()
    {
        if (!mIsSet) throw std::runtime_error("In MultipliedConditionalProbability::getProbability(): trying to access probability that has not been set.");
        //std::cout << "In this slot, " << mCount << " probabilities were multiplied to generate probability " << mProbability << std::endl;
        mWasRead = true;
        return mProbability;
    }
private:
    /**
     * The current multiplied value of the conditional probabilities.
     */
    double mProbability;
    /**
     * Whether a value for the conditional probability has been set.
     */
    bool mIsSet;
    /**
     * Whether this probability has already been read once. Adding to a probability that has already been read is forbidden.
     */
    bool mWasRead;
};

}

