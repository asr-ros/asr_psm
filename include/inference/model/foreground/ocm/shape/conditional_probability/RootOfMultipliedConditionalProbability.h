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
#include <cmath>

namespace ProbabilisticSceneRecognition {

/**
 * This class represents a conditional probability of an evidences pose x given its parent evidence's pose p: P(x|p).
 * If there are several parents p1, p2, ... pn, it uses the nth root of the multiplied probability of the separate conditional probabilities:
 * root_n(P(x|p1) * P(x|p2) * ...).
 */
class RootOfMultipliedConditionalProbability: public ConditionalProbability {
public:
    /**
     * Constructor.
     */
    RootOfMultipliedConditionalProbability();

    /**
     * Destructor.
     */
    ~RootOfMultipliedConditionalProbability();

private:
    /**
     * Calculate the probability from the parent probabilities.
     * @return the probability calculated from the parent probabilities.
     */
    virtual double calculateProbability();
};

}

