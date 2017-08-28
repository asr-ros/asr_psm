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

#include <ISM/common_type/ObjectSet.hpp>

namespace ProbabilisticSceneRecognition {

/**
 * This class describes a test set, that is a set of objects used to test topologies against during combinatorial optimization.
 */
class TestSet {

public:

    /**
     * Constructor.
     */
    TestSet();

    /**
     * Destructor.
     */
    ~TestSet();

    /**
     * The set of objects this test set consists of.
     */
    ISM::ObjectSetPtr mObjectSet;

    /**
     * Set the results of the test of this set with the fully meshed topology.
     * @param pFullyMeshedProbability           The probability of this set tested with the fully meshed topology.
     * @param pFullyMeshedRecognitionRuntime    The recognition runtime of this test set when tested with the fully meshed topology.
     */
    void setFullyMeshedTestResult(double pFullyMeshedProbability, double pFullyMeshedRecognitionRuntime);

    /**
     * Get the probability of this set tested with the fully meshed topology.
     * @return the probability of this set tested with the fully meshed topology.
     */
    double getFullyMeshedProbability() const;

    /**
     * Get the recognition runtime of this test set when tested with the fully meshed topology.
     * @return the recognition runtime of this test set when tested with the fully meshed topology.
     */
    double getFullyMeshedRecognitionRuntime() const;

private:
    /**
     * The probability of this set tested with the fully meshed topology.
     */
    double mFullyMeshedProbability;

    /**
     * The recognition runtime of this test set when tested with the fully meshed topology.
     */
    double mFullyMeshedRecognitionRuntime;

    /**
     * Whether this test set has been tested with the fully meshed topology,
     * i.e. whether probability and recognition runtime above are valid.
     */
    bool mTestedWithFullyMeshed;

};

}
