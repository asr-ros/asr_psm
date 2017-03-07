/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/PlotHelper.h"

namespace ProbabilisticSceneRecognition {

    PlotHelper::PlotHelper(): PlotHelper("") { }
    PlotHelper::PlotHelper(std::string filename): mPlotFileHandler(filename) { }

    void PlotHelper::initPlot(const std::string& pPlotTitle,
                           const std::string& pXLabel,
                           const std::string& pYLabel,
                           const std::pair<double, double>& pXRange,
                           const std::pair<double, double>& pYRange,
                           unsigned int samples)
    {
        mPlotFileHandler.reset();   //Create a clean interface to gnuplot.

        mPlotFileHandler.add("set xlabel \"" + pXLabel + "\"\n");     //Set label for x axis
        mPlotFileHandler.add("set ylabel \"" + pYLabel + "\"\n");     //Set label for y axis
        mPlotFileHandler.add("\n");
        mPlotFileHandler.add("set xrange [" + boost::lexical_cast<std::string>(pXRange.first) +  ":" + boost::lexical_cast<std::string>(pXRange.second) + "]\n");   //Set range in x direction
        mPlotFileHandler.add("set yrange [" + boost::lexical_cast<std::string>(pYRange.first) +  ":" + boost::lexical_cast<std::string>(pYRange.second) + "]\n");   //Set range in y direction
        mPlotFileHandler.add("set nokey\n");                // don't show legend
        mPlotFileHandler.add("set samples " + boost::lexical_cast<std::string>(samples) + "\n");         // set number of samples
        mPlotFileHandler.add("set title \"" + pPlotTitle + "\"\n"); //Set title
        mPlotFileHandler.add("\n");

    }

    void PlotHelper::addPointToBuffer(double x, double y)
    {
        mPointBuffer.push_back(std::make_pair(x, y));
    }

    void PlotHelper::sendPlot()
    {
        ROS_ASSERT(mPlotFileHandler.gnuplotExists());
        //Prevent system from trying to send data to non-initialized gnuplot handler
        if(!mPlotFileHandler.gnuplotExists())
          throw std::runtime_error("Cannot show non-existing gnuplot visualization.");

        mPlotFileHandler.add("plot \"-\" with lines\n");    // Prepare for plotting.

        //Push data to gnuplot and file. "end" is added to end of file automatically, see below.
        mPlotFileHandler.send(mPointBuffer);
        mPlotFileHandler.flush();
    }

    PlotHelper::PlotFileHandler::PlotFileHandler(std::string filename): mFileBuffer(""), mFilename(filename) { }

    void PlotHelper::PlotFileHandler::reset() { mGnuplotHandler.reset(new Gnuplot); }

    void PlotHelper::PlotFileHandler::add(std::string in)
    {
        *(mGnuplotHandler) << in;
        mFileBuffer += in;
    }

    void PlotHelper::PlotFileHandler::send(std::vector<std::pair<double, double> > pointBuffer)
    {
        // Write to file if requested and possible
        if (mFilename != "")
        {
            std::ofstream file;
            file.open(mFilename);
            if (file.is_open())
            {
                file << mFileBuffer;    // Write plot configuration to file
                for (unsigned int i = 0; i < pointBuffer.size(); i++) {     // Write data to file
                    file << pointBuffer.at(i).first << " " << pointBuffer.at(i).second << "\n";
                }
                file << "end\n";    // Add "end" so gnuplot later recognizes the end of the data block
                file << "pause -1 \"Hit return to continue\"";  // Add pause and waiting for user input to the end of the file
                file.close();
            }
            else { ROS_INFO_STREAM("Could not open file " + mFilename + ". Proceeding without writing to it."); }
        }
        // send to gnuplot
        mGnuplotHandler->send(pointBuffer);
    }

    bool PlotHelper::PlotFileHandler::gnuplotExists()
    {
        if(!mGnuplotHandler) return false;
        else return true;
    }

    void PlotHelper::PlotFileHandler::flush() { mGnuplotHandler->flush(); }
}
