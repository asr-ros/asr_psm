/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.h"

namespace ProbabilisticSceneRecognition {

void TestSetGenerator::generateTestSets(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
                                        unsigned int pTestSetCount)
{
    validateSets(generateRandomSets(pExamplesList, pTestSetCount));
}

std::vector<std::vector<asr_msgs::AsrObject>> TestSetGenerator::generateRandomSets(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList, unsigned int pTestSetCount)
{
    ROS_INFO_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    ROS_INFO_STREAM("Generating " << pTestSetCount << " random test sets.");
    ROS_INFO_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    //std::cout << "Generating random test sets for object types ";
    //for (std::string type: mTypes) std::cout << " " << type;
    //std::cout << std::endl;
    std::random_device rd;
    boost::mt19937  eng;    // Mersenne Twister
    eng.seed(rd());
    boost::uniform_int<> dist(0,RAND_MAX);  // Normal Distribution
    boost::variate_generator<boost::mt19937,boost::uniform_int<>>  gen(eng,dist);  // Variate generator

    // Generate all possible realtions:
    std::vector<SceneModel::Relation> allRelations;
    for (unsigned int i = 0; i < mTypes.size() - 1; i++)
        for (unsigned int j = i + 1; j < mTypes.size(); j++)
            allRelations.push_back(SceneModel::Relation(mTypes[i], mTypes[j]));

    std::vector<std::vector<asr_msgs::AsrObject>> testSets;

    unsigned int testSetsPerSceneGraph = pTestSetCount / pExamplesList.size();
    for (unsigned int i = 0; i < pExamplesList.size(); i++)
    {
        boost::shared_ptr<const asr_msgs::AsrSceneGraph> currentSceneGraph = pExamplesList[i];
        for (unsigned int j = 0; j < testSetsPerSceneGraph; j++)     // generate a certain number of test sets for each available scene graph for the scene
        {
            // compare bachelor thesis of Fabian Hanselmann
            std::vector<asr_msgs::AsrObject> testSet;
            unsigned int randomTrackIndex = gen() % currentSceneGraph->scene_elements.size();
            asr_msgs::AsrNode randomTrack = currentSceneGraph->scene_elements[randomTrackIndex];
            unsigned int randomPoseIndex = gen() % randomTrack.track.size();    // Select random scene observation, assuming all tracks have the same length.
            asr_msgs::AsrObject referenceObject = makeAsrObject(randomTrack.track[randomPoseIndex]);  // Pick random pose on trajectory
            //testSet.push_back(referenceObject);   // currently gets pushed like any other object, in transformed form.

            for (asr_msgs::AsrNode trajectory: currentSceneGraph->scene_elements)   // since each object can only appear once, iterate over all trajectories
            {
                //std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                unsigned int occurs;
                if (mObjectMissingInTestSetProbability > 1) throw std::runtime_error("parameter object_missing_in_test_set_probability should not be larger than 1.");
                if (mObjectMissingInTestSetProbability > 0)
                    occurs = gen() % 1 / mObjectMissingInTestSetProbability;   // only with a certain probability:
                else occurs = 1;
                if (occurs != 0)                    // object does occur
                {
                    asr_msgs::AsrObservations observation = trajectory.track[randomPoseIndex];  // Pick random pose on trajectory
                    asr_msgs::AsrObject object = makeAsrObject(observation);

                    /*std::cout << "Took observation with type " << object.type << " from random index " << randomPoseIndex << " in trajectory." << std::endl;
                    for (geometry_msgs::PoseWithCovariance pwc: object.sampledPoses)
                    {
                        std::cout << "position = (" << pwc.pose.position.x << ", " << pwc.pose.position.y << ", " << pwc.pose.position.z << ")" << std::endl;
                        std::cout << "orientation = (" << pwc.pose.orientation.w << pwc.pose.orientation.x << ", " << pwc.pose.orientation.y << ", " << pwc.pose.orientation.z << ")" << std::endl;
                    }*/

                    setPoseOfObjectRelativeToReference(object, referenceObject);
                    /*std::cout << "Relative to reference object:" << std::endl;
                    for (geometry_msgs::PoseWithCovariance pwc: object.sampledPoses)
                    {
                        std::cout << "position = (" << pwc.pose.position.x << ", " << pwc.pose.position.y << ", " << pwc.pose.position.z << ")" << std::endl;
                        std::cout << "orientation = (" << pwc.pose.orientation.w << ", " << pwc.pose.orientation.x << ", " << pwc.pose.orientation.y << ", " << pwc.pose.orientation.z << ")" << std::endl;
                    }*/

                    //if (referenceObject.type == trajectory.track[randomPoseIndex].type) std::cout << "(Was reference object)" << std::endl;
                    testSet.push_back(object);  // probabliy add "else" in front of this statement, so that transformed reference doesn't get pushed.

                }
                //else std::cout << "Did not take observation." << std::endl;
            }
            testSets.push_back(testSet);

        }
    }
    /*std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;*/

    return testSets;
}

void TestSetGenerator::validateSets(std::vector<std::vector<asr_msgs::AsrObject>> pTestSets)
{
    // Use fully meshed topology to calculate probabilities:
    if (!mFullyMeshedTopology->mTree) throw std::runtime_error("in TestSetGenerator::validateSets(): fully meshed topology has no valid tree.");
    // set all tests as valid (so that fully meshed topology, which cannot have false positives, gets its evaluation result set already)
    mEvaluator->setValidTestSets(pTestSets);
    mEvaluator->setInvalidTestSets(std::vector<std::vector<asr_msgs::AsrObject>>());
    mEvaluator->setRecognitionThreshold(-1);    // so every test set gets recognized

    ROS_INFO_STREAM("Recognizing test sets with fully meshed topology.");
    std::vector<double> testSetProbabilities;   // Needs to be in the same order as the testSets.
    mEvaluator->evaluate(mFullyMeshedTopology, testSetProbabilities);
    if (mFullyMeshedTopology->mFalsePositives != 0)
        throw std::runtime_error("In TestSetGenerator::validateSets(): Error when evaluating fully meshed topology: has false positives, should have none");

    double maxProbability = 0;
    double minProbability = 1;  // Minumum probability greater than zero. set to possible maximum so it will be lowered in each case
    unsigned int zeroSets = 0;
    for (double probability: testSetProbabilities)
    {
        if (probability > maxProbability) maxProbability = probability;
        if (probability == 0) zeroSets++;
        else if (probability < minProbability) minProbability = probability;    // set lowest non-zero probability as new minimum
    }

    double recognitionThreshold = ((maxProbability - minProbability) / 2) + minProbability;  // for testing: set recognition threshold to the middle of the probability range.
    //mRecognitionThreshold = minProbability;    // for testing: all probabilities greater than the minimum probability are valid.
    //mRecognitionThreshold = 0;      // for testing: considers all scenes with any probability greater than zero as recognized.

    std::vector<std::vector<asr_msgs::AsrObject>> validTestSets;
    std::vector<std::vector<asr_msgs::AsrObject>> invalidTestSets;

    for (unsigned int i = 0; i < testSetProbabilities.size(); i++)
    {
        if (testSetProbabilities[i] > recognitionThreshold)
            validTestSets.push_back(pTestSets[i]);
        else invalidTestSets.push_back(pTestSets[i]);
    }

    ROS_INFO_STREAM("===========================================================");
    ROS_INFO_STREAM("Test set creation complete.");
    ROS_INFO_STREAM("Maximum probability: " << maxProbability);
    ROS_INFO_STREAM("Minimum probability greater than zero: " << minProbability);
    ROS_INFO_STREAM("Recognition threshold: " << recognitionThreshold);
    ROS_INFO_STREAM("Found " << validTestSets.size() << " valid and " << invalidTestSets.size() << " invalid test sets.");
    ROS_INFO_STREAM(zeroSets << " test sets have probability 0.");
    ROS_INFO_STREAM("===========================================================");

    mEvaluator->setValidTestSets(validTestSets);    // update with newfound valid test sets
    mEvaluator->setInvalidTestSets(invalidTestSets);    // initialize properly
    mEvaluator->setRecognitionThreshold(recognitionThreshold);

    mValidTestSets = validTestSets;
    mInvalidTestSets = invalidTestSets;
}

asr_msgs::AsrObject TestSetGenerator::makeAsrObject(asr_msgs::AsrObservations pObservation)
{
    asr_msgs::AsrObject object;
    object.header.stamp = pObservation.stamp;
    geometry_msgs::PoseWithCovariance pwc;
    pwc.pose = pObservation.transform;
    object.sampledPoses.push_back(pwc);
    object.type = pObservation.type;
    return object;
}

void TestSetGenerator::setPoseOfObjectRelativeToReference(asr_msgs::AsrObject& pObject, const asr_msgs::AsrObject& pReference)
{
    std::vector<geometry_msgs::PoseWithCovariance> newPoses;
    for (geometry_msgs::PoseWithCovariance refPose: pReference.sampledPoses)
    {
        Eigen::Vector3d refPosePosition(refPose.pose.position.x, refPose.pose.position.y, refPose.pose.position.z);
        Eigen::Quaterniond refPoseOrientation(refPose.pose.orientation.w, refPose.pose.orientation.x, refPose.pose.orientation.y, refPose.pose.orientation.z);
        for (geometry_msgs::PoseWithCovariance objectPose: pObject.sampledPoses)
        {
            Eigen::Vector3d objectPosePosition(objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z);
            Eigen::Quaterniond objectPoseOrientation(objectPose.pose.orientation.w, objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z);
            // compare ISM::GeometryHelper
            if ((refPosePosition - objectPosePosition).norm() == 0) {
                //avoid special case
                objectPosePosition.x() = (objectPosePosition.x() + 0.0000001);
            }
            Eigen::Vector3d objToRefVector = refPosePosition - objectPosePosition;
            Eigen::Vector3d refToObjVector = objToRefVector * -1.0;
            Eigen::Quaternion<double> p = objectPoseOrientation;
            Eigen::Quaternion<double> r = refPoseOrientation;

            //rotate everything relative to ref pose
            Eigen::Quaternion<double> refToObjectPoseQuat = (r.inverse()) * p;

            geometry_msgs::PoseWithCovariance newPose;
            newPose.pose.position.x = refToObjVector.x();
            newPose.pose.position.y = refToObjVector.y();
            newPose.pose.position.z = refToObjVector.z();
            newPose.pose.orientation.w = refToObjectPoseQuat.w();
            newPose.pose.orientation.x = refToObjectPoseQuat.x();
            newPose.pose.orientation.y = refToObjectPoseQuat.y();
            newPose.pose.orientation.z = refToObjectPoseQuat.z();

            newPoses.push_back(newPose);
        }
    }

    pObject.sampledPoses = newPoses;
    //std::cout << "+++++++++++++++++++++++++++++" << std::endl;
}

}
