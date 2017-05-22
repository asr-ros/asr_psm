/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <chrono>

#include <Eigen/Dense>

#include <asr_msgs/AsrObject.h>

#include <ISM/common_type/Object.hpp>

namespace ProbabilisticSceneRecognition {
	/**
	 * The KalmanFilter class implements a KalmanFilter on a multidimensional space.
	 * A Kalman Filter estimates the real value vector in terms of the expected input error and expected output error.
	 * Which means the smaller the expected input error is, the more likely is the system to change the state.
	 * The bigger the expected output error is, the less likely the system is to change. And vice versa.
	 *
	 * @author Ralf Schleicher <mail@ralfschleicher.de>
     */;
	class KalmanFilter {
	private:
		/**
		 * If reset flag is true, reset the system to the current measurement.
		 */
		bool mReset;
		
		/**
		 * The time since the last update.
		 */
		std::chrono::high_resolution_clock::time_point lastUpdate;
		
		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mF;
;
		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mH;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mQ;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mR;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mP;

		/**
		 * A multidimensional vector.
		 */
		Eigen::VectorXd mX;

		/**
		 * A multidimensional vector.
		 */
		Eigen::VectorXd mZ;
		
        /**
         * Instance of the current AsrObject.
		 */
        ISM::Object mInstance;


	public:
		/**
		 * Creates a kalman filter for given input matrices.
		 * @param pObject The inintial measurement.
         */
        KalmanFilter(ISM::Object pObject);

		/**
		 * Destructor.
		 */
		~KalmanFilter();

		/**
		 * Resets the kalman filter.
		 */
		void reset();

		/**
		 * Updates the current state of the system
		 * @param pObject The new measurement to update the filter.
         */
        void update(ISM::Object pObject);
		
		/**
		 * Checks if the last update has been longer ago than the given threshold.
		 * 
		 * @param threshold Valid time in milliseconds since the last update.
		 */
		bool isTimedOut(unsigned int threshold);
		
		/**
         * Returns the AsrObject wrapped by the filter.
         * @return The AsrObject wrapped by the filter.
         */
        ISM::Object getObject();
	};
}
