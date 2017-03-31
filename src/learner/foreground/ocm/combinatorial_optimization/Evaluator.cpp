/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/Evaluator.h"

namespace ProbabilisticSceneRecognition {

Evaluator::Evaluator(const std::string pInferenceAlgorithm, std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
                     std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners, double pRecognitionThreshold, const std::string& pXmlOutput, const std::string& pXmlFilePath):
    mInferenceAlgorithm(pInferenceAlgorithm), mExamplesList(pExamplesList), mXmlOutput(pXmlOutput), mLearners(pLearners), mRecognitionThreshold(pRecognitionThreshold), mXmlFilePath(pXmlFilePath), mRunNumber(0)
{ }

void Evaluator::evaluate(boost::shared_ptr<SceneModel::Topology> pTopology)
{
    std::vector<double> testSetProbabilities; // gets discarded afterwards
    evaluate(pTopology, testSetProbabilities);
}

void Evaluator::evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, std::vector<double>& pTestSetProbabilities)
{
    if (!pTopology) throw std::runtime_error("In Evaluator::evaluate(): topology from argument is null pointer.");

    if (pTopology->mEvaluated)
    {
        /*std::cout << "===========================================================" << std::endl;
        std::cout << "Topology " << pTopology->mIdentifier << " has already been evaluated." << std::endl;
        std::cout << "Results were: false positives = " << pTopology->mFalsePositives << ", average recognition runtime = " << pTopology->mAverageRecognitionRuntime << std::endl;
        std::cout << "===========================================================" << std::endl;*/
        return;
    }

    ROS_INFO_STREAM("===========================================================");
    ROS_INFO_STREAM("Evaluating topology " << pTopology->mIdentifier << ":");
    ROS_INFO_STREAM("===========================================================");
    if (!pTopology->mTree) throw std::runtime_error("In Evaluator::evaluate(): topology from argument has no tree associated with it.");

    update(pTopology->mTree);

    if (mValidTestSets.empty() && mInvalidTestSets.empty()) throw std::runtime_error("In Evaluator::evaluate(): no valid or invalid test sets found.");

    unsigned int falseNegatives = 0;
    unsigned int falsePositives = 0;
    std::vector<double> testSetProbabilities;

    struct timeval start;
    struct timeval end;

    gettimeofday(&start, NULL); // get start time
    for (std::vector<asr_msgs::AsrObject> valid: mValidTestSets)
    {
        double probability = getProbability(valid);
        testSetProbabilities.push_back(probability);
    }

    for (std::vector<asr_msgs::AsrObject> invalid: mInvalidTestSets)
    {
        double probability = getProbability(invalid); // Foreground scene probability of dummy test set for partial model
        testSetProbabilities.push_back(probability);
    }
    gettimeofday(&end, NULL);   // get the stop time

    double recognitionRuntime, seconds, useconds;
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    recognitionRuntime = seconds + useconds / 1000000;
    double avgRuntime = recognitionRuntime / (mValidTestSets.size() + mInvalidTestSets.size());

    // Output and checks are handled outside of the calculation to not interfere with the runtime measuring
    if (testSetProbabilities.size() != mValidTestSets.size() + mInvalidTestSets.size())
        throw std::runtime_error("In Evaluator::evaluate(): size of list of all test set probabilities is not equal to combined sizes of test set lists.");
    for (unsigned int i = 0; i < mValidTestSets.size(); i++)
        if (testSetProbabilities[i] <= mRecognitionThreshold) falseNegatives++;   // did not recognize a valid scene.
    //std::cout << "Valid test sets result: false negatives: " << falseNegatives << std::endl;
    if (falseNegatives > 0)
        throw std::runtime_error("This model should never produce false negatives.");
    for (unsigned int i = mValidTestSets.size(); i < mValidTestSets.size() + mInvalidTestSets.size(); i++)
        if (testSetProbabilities[i] > mRecognitionThreshold) falsePositives++;   // did recognize an invalid scene.

    /*std::cout << "Invalid test sets result: false positives: " << falsePositives << std::endl;
    std::cout << "Overall testing runtime: " << recognitionRuntime << ". Average: " << avgRuntime << std::endl;*/

    ROS_INFO_STREAM("Evaluation result: " << falseNegatives << " false Negatives, " << falsePositives << " false positives, " << avgRuntime << "s average recognition runtime.");

    xmlOutput(falsePositives, avgRuntime);

    pTopology->mFalsePositives = falsePositives;
    pTopology->mAverageRecognitionRuntime = avgRuntime;
    pTopology->mEvaluated = true;
    pTestSetProbabilities = testSetProbabilities;
}

double Evaluator::getProbability(const std::vector<asr_msgs::AsrObject>& pEvidence)
{
    mForegroundSceneContent.update(pEvidence, mRuntimeLogger);
    return mForegroundSceneContent.getSceneProbability(); // Foreground scene probability for partial model
}

void Evaluator::update(boost::shared_ptr<SceneModel::TreeNode> pTree)
{
    // reset mModel:
    mModel = boost::property_tree::ptree();

    ROS_INFO_STREAM("===========================================================");
    ROS_INFO_STREAM("Starting to learn OCM foreground model:");
    ROS_INFO_STREAM("===========================================================");

    // Now just forward all examples for the scene to the scene object learners and save partial model to property tree.
    for (boost::shared_ptr<SceneObjectLearner> learner: mLearners)
    {
        learner->learn(mExamplesList, pTree);
        learner->save(mModel);
    }
    ROS_INFO_STREAM("===========================================================");
    ROS_INFO_STREAM("Learning complete. Preparing inference.");
    ROS_INFO_STREAM("===========================================================");

    // Reset mForegroundSceneContent:
    mForegroundSceneContent = ForegroundSceneContent();
    mForegroundSceneContent.initializeInferenceAlgorithms(mInferenceAlgorithm);
    for (boost::shared_ptr<const asr_msgs::AsrSceneGraph> sceneGraph: mExamplesList)
        mForegroundSceneContent.update(sceneGraph);     // is this correct and even necessary?

    mForegroundSceneContent.load(mModel);   // loads partial model.

    boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> dummyViz(new Visualization::ProbabilisticSceneVisualization("dummy"));
    mForegroundSceneContent.initializeVisualizer(dummyViz);
    // psm offers the possibility to add foreground learners other than OCM, which might use no or different relations.
    // because of this and since the complete model is only assembled at the very end, only a partial model is used here.
}

void Evaluator::xmlOutput(double pFalsePositives, double pAvgRuntime)
{
    if (mXmlOutput != "none")
    {
        boost::property_tree::ptree extendedModel;

        mModel.add("<xmlattr>.run", mRunNumber);
        mModel.add("<xmlattr>.false_positives", pFalsePositives);
        mModel.add("<xmlattr>.average_recognition_runtime", pAvgRuntime);

        extendedModel.add_child("evaluator_result", mModel);

        if (mXmlOutput == "screen")
        {
            std::cout << "Partial model in Evaluator run " << mRunNumber << ":" << std::cout;
            write_xml(std::cout, extendedModel);
            std::cout << std::endl;
        }
        else if (mXmlOutput == "file")
        {
            std::string xmlFileName = mXmlFilePath + "evaluator_run_" + boost::lexical_cast<std::string>(mRunNumber);
            //std::cout << "Trying to write xml output to file " << xmlFileName << std::endl;

            write_xml(xmlFileName, extendedModel);
        }
        else throw std::runtime_error("Parameter xml_output has invalid value " + mXmlOutput);
    }
    // if value is "none": do nothing.
    mRunNumber++;
}

}
