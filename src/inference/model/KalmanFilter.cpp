/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <iostream>

#include "inference/model/KalmanFilter.h"

namespace ProbabilisticSceneRecognition {
    KalmanFilter::KalmanFilter(ISM::Object pObject) :

			mReset(true)
	{
	  	mF = Eigen::MatrixXd::Identity(7, 7);
		mH = Eigen::MatrixXd::Identity(7, 7);

		mQ = 0.1 * Eigen::MatrixXd::Identity(7, 7);
		mR = 1 * Eigen::MatrixXd::Identity(7, 7);
		
		mP = Eigen::MatrixXd::Identity(mF.rows(), mF.cols());
		
		update(pObject);
	}

	// dtor
	KalmanFilter::~KalmanFilter() {
	}

	void KalmanFilter::reset() {
		mReset = true;
	}

    void KalmanFilter::update(ISM::Object pObject) {
	  

		// Bring last update time uptodate.
		lastUpdate = std::chrono::high_resolution_clock::now();
	  
		// Rename the mesaurement to x.
		Eigen::VectorXd x = Eigen::VectorXd::Zero(7);


        if(!pObject.pose){
          std::cerr << "Got a Object without poses." << std::endl;
		  std::exit(1);    
		}

        boost::shared_ptr<ISM::Pose> current_pose = pObject.pose;
        x(0) = current_pose->point->getEigen().x();
        x(1) = current_pose->point->getEigen().y();
        x(2) = current_pose->point->getEigen().z();
        x(3) = current_pose->quat->getEigen().w();
        x(4) = current_pose->quat->getEigen().x();
        x(5) = current_pose->quat->getEigen().y();
        x(6) = current_pose->quat->getEigen().z();
		
		// If reset flag is true, reset the system to the current measurement.
		if (mReset) {
			mReset = false;
			mX = x;
			mZ = mH * x;
		}

		// Prediction step.
		Eigen::VectorXd xPred = mF * x;
		Eigen::MatrixXd PPred = mF * mP * mF.transpose() + mQ;

		// Update step
		Eigen::MatrixXd Ku = PPred * mH.transpose();
		Eigen::MatrixXd Kl = (mH * PPred * mH.transpose() + mR);
		Eigen::MatrixXd K = Ku * Kl.inverse();
		Eigen::VectorXd xNew = xPred + K * (mZ - mH * xPred);
		mP = PPred - K * mH * PPred;

		// Set the x and the z
		mX = xNew;
		mZ = mH * mX;

        // Save instance of Asr::AsrObject.
		mInstance = pObject;

	}
	
	bool KalmanFilter::isTimedOut(unsigned int threshold)
	{
	  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastUpdate).count() > threshold;
	}

    ISM::Object KalmanFilter::getObject()
	{

	  // Write the pose maintained by the kalman-filter back into the object.
      geometry_msgs::Pose current_pose;
      current_pose.position.x = mZ(0);
      current_pose.position.y = mZ(1);
      current_pose.position.z = mZ(2);
      current_pose.orientation.w = mZ(3);
      current_pose.orientation.x = mZ(4);
      current_pose.orientation.y = mZ(5);
      current_pose.orientation.z = mZ(6);

      //mInstance.sampledPoses.pop_back();
      geometry_msgs::PoseWithCovariance current_pose_with_c;
      current_pose_with_c.pose = current_pose;
     // mInstance.sampledPoses.push_back(current_pose_with_c);
	  // Return the instance updated by the kalman filter.
	  return mInstance;
	}
}
