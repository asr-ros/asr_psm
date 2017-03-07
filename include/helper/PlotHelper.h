/**

Copyright (c) 2016, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>

#include "helper/gnuplot-iostream.h"

namespace ProbabilisticSceneRecognition {

    // similar to active_scene_recognition/recognizer_prediction_psm asrVisualizer
    /**
     * Helper class for plotting data
     */
    class PlotHelper
    {
    public:
        /**
         * Constructor
         * If constructed with this, PlotHelper doesn't write the gnuplot script to file
         * and just executes it.
         */
        PlotHelper();
        /**
         * Constructor
         * @param filename  Name of the file to write the gnuplot script into
         * If filename == "": don't write plot to file
         * Data is written into filename + ".data" file
         */
        PlotHelper(std::string filename);

        /**
         * Initializes the Gnuplot
         * @param pPlotTitle - the name of the plot
         * @param pxLabel - label of the x-axis
         * @param pyLabel - label of the y-axis
         * @param pXRange - x-axis range (min, max)
         * @param pYRange - y-axis range (min, max)
         * @param samples - number of samples
         */
        void initPlot(const std::string& pPlotTitle,
                               const std::string& pXLabel,
                               const std::string& pYLabel,
                               const std::pair<double, double>& pXRange,
                               const std::pair<double, double>& pYRange,
                               unsigned int samples);

        /**
         * Adds a new point to the list of points
         */
        void addPointToBuffer(double x, double y);

        /**
         * Sends the data to the gnuplot and draws the window
         * which contains the plot.
         * Also writes the gnuplot data created and used to file.
         */
        void sendPlot();

    private:
        /**
         * Private class handling the writing to file and gnuplot output of the data
         */
        class PlotFileHandler
        {
        public:
            /**
             * Constructor
             * @param filename  Name of the file to write the gnuplot script into
             * If "": Do not write to file
             */
            PlotFileHandler(std::string filename);

            /**
             * Create a clean interface to gnuplot.
             */
            void reset();

            /**
             * Add string to gnuplot and file buffer
             */
            void add(std::string in);

            /**
             * Send data to gnuplot and write them to file
             * @param pointBuffer   The buffer containing the point data
             */
            void send(std::vector<std::pair<double, double>> pointBuffer);

            /**
             * Checks whether gnuplot exists or not
             * @return existence of gnuplot
             */
            bool gnuplotExists();

            /**
             * Flushes the gnuplot.
             */
            void flush();

        private:
            //Interface with which configurations or data is sent to gnuplot.
            boost::shared_ptr<Gnuplot> mGnuplotHandler;
            // Buffer for text to be written to file
            std::string mFileBuffer;
            // Name of the file to write the gnuplot script into
            std::string mFilename;
        };

        //Interface through which configurations or data is sent to gnuplot and file
        PlotFileHandler mPlotFileHandler;

        //buffer for visualization points
        std::vector<std::pair<double, double> > mPointBuffer;
    };
}
