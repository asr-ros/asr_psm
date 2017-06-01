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

#include <string>

namespace ProbabilisticSceneRecognition {

/**
 * This class offers functionality to help printing to the screen.
 */
class PrintHelper {

public:
    /**
     * Constructor.
     * @param pMarker    Symbol used to mark output with dividers consisting of it.
     */
    PrintHelper(char pMarker)
    {
        mDivider = std::string(60, pMarker);
    }

    /**
     * Destructor.
     */
    ~PrintHelper() { }

    /**
     * Add a line to print.
     * @param pLine to print.
     */
    void addLine(const std::string& pLine)
    {
        mLines.push_back(pLine);
    }

    /**
     * Print added lines as header, between two dividers, and reset lines.
     */
    void printAsHeader()
    {
        ROS_INFO_STREAM(mDivider);
        for (std::string line: mLines)
            ROS_INFO_STREAM(line);
        ROS_INFO_STREAM(mDivider);
        mLines = std::vector<std::string>(); // reset mLines.
    }

    /**
     * Print a single line as a header.
     * @param pLine
     */
    void printAsHeader(const std::string& pLine)
    {
        addLine(pLine);
        printAsHeader();
    }

private:

    /**
     * Divider used to mark output.
     */
    std::string mDivider;

    /**
     * Lines to print.
     */
    std::vector<std::string> mLines;

};

}
