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

Evaluator::Evaluator(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
                     std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners, double pRecognitionThreshold):
    mExamplesList(pExamplesList), mLearners(pLearners), mRecognitionThreshold(pRecognitionThreshold), mRunNumber(0)
{
    ros::NodeHandle nodeHandle("~");

    // Try to get the name of the inference algorithm.
    if(!nodeHandle.getParam("inference_algorithm", mInferenceAlgorithm))
       throw std::runtime_error("Please specify parameter inference_algorithm when starting this node.");

    // Try to get the target of the output of the learned model in xml form.
    if(!nodeHandle.getParam("xml_output", mXmlOutput))
       throw std::runtime_error("Please specify parameter xml_output when starting this node.");

    mXmlFilePath = "";
    // If output to file, try to get the file path.
    if (mXmlOutput == "file")
        if(!nodeHandle.getParam("xml_file_path", mXmlFilePath))
           throw std::runtime_error("Please specify parameter xml_file_path when starting this node.");

    // Try to get whether to create a log of the runtime in a file.
    if(!nodeHandle.getParam("create_runtime_log", mCreateRuntimeLog))
        throw std::runtime_error("Please specify parameter create_runtime_log when starting this node.");

    std::string runtimeLogFileName = "";
    if (mCreateRuntimeLog)
    {
        // Try to get the complete log file name.
        if(!nodeHandle.getParam("log_file_name", runtimeLogFileName))
            throw std::runtime_error("Please specify parameter log_file_name when starting this node.");

        mRuntimeLogger.open(runtimeLogFileName);
        mRuntimeLogger << "sceneobject,runtime\n";
    }

    // Try to get whether to visualize intermediate inference runs:
    if(!nodeHandle.getParam("visualize_inference", mVisualize))
        throw std::runtime_error("Please specify parameter visualize_inference when starting this node.");

    // Try to get whether to use targeting help in visualization:
    if(!nodeHandle.getParam("targeting_help", mTargetingHelp))
        throw std::runtime_error("Please specify parameter targeting_help when starting this node.");

    // Try to get the drawing parameters:
    if(!nodeHandle.getParam("base_frame_id", mFrameId))
        throw std::runtime_error("Please specify parameter base_frame_id when starting this node.");

    if(!nodeHandle.getParam("scale_factor", mScaleFactor))
        throw std::runtime_error("Please specify parameter scale_factor when starting this node.");
    if(!nodeHandle.getParam("sigma_multiplicator", mSigmaMultiplier))
        throw std::runtime_error("Please specify parameter sigma_multiplicator when starting this node.");

    // Set up foreground scene content:
    mForegroundSceneContent.initializeInferenceAlgorithms(mInferenceAlgorithm);
    for (boost::shared_ptr<const asr_msgs::AsrSceneGraph> sceneGraph: mExamplesList)
        mForegroundSceneContent.update(sceneGraph);
}

Evaluator::~Evaluator() {
    if (mCreateRuntimeLog && mRuntimeLogger.is_open())
    {
        mRuntimeLogger.close();
    }
}

void Evaluator::evaluate(boost::shared_ptr<SceneModel::Topology> pTopology)
{
    std::vector<double> testSetProbabilities; // gets discarded afterwards
    evaluate(pTopology, testSetProbabilities);
}

void Evaluator::evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, std::vector<double>& pTestSetProbabilities)
{
    if (!pTopology) throw std::runtime_error("In Evaluator::evaluate(): topology from argument is null pointer.");

    if (pTopology->mEvaluated) return;

    printDivider();
    ROS_INFO_STREAM("Evaluating topology " << pTopology->mIdentifier << ":");
    printDivider();
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
        double probability = getProbability(invalid); // Foreground scene probability of test set for partial model
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
    for (unsigned int i = mValidTestSets.size(); i < mValidTestSets.size() + mInvalidTestSets.size(); i++)
        if (testSetProbabilities[i] > mRecognitionThreshold) falsePositives++;   // did recognize an invalid scene.

    ROS_INFO_STREAM("Evaluation result: " << falsePositives << " false positives, " << falseNegatives << " false negatives, " << avgRuntime << "s average recognition runtime.");

    pTopology->mFalsePositives = falsePositives;
    pTopology->mAverageRecognitionRuntime = avgRuntime;
    pTopology->mFalseNegatives = falseNegatives;
    pTopology->mEvaluated = true;

    xmlOutput(pTopology);

    pTestSetProbabilities = testSetProbabilities;
}

double Evaluator::getProbability(const std::vector<asr_msgs::AsrObject>& pEvidence)
{
    mForegroundSceneContent.update(pEvidence, mRuntimeLogger);
    if (mVisualize)
    {
        if (mTargetingHelp)
            mVisualizer->drawInTargetingMode();
        else
            mVisualizer->drawInInferenceMode();
    }
    return mForegroundSceneContent.getSceneProbability(); // Foreground scene probability for partial model;
}

void Evaluator::update(boost::shared_ptr<SceneModel::TreeNode> pTree)
{
    // reset mModel:
    mModel = boost::property_tree::ptree();

    printDivider();
    ROS_INFO_STREAM("Starting to learn OCM foreground model:");
    printDivider();

    // Now just forward all examples for the scene to the scene object learners and save partial model to property tree.
    for (boost::shared_ptr<SceneObjectLearner> learner: mLearners)
    {
        learner->learn(mExamplesList, pTree);
        learner->save(mModel);
    }
    printDivider();
    ROS_INFO_STREAM("Learning complete. Preparing inference.");
    printDivider();

    mForegroundSceneContent.load(mModel);   // loads partial model.

    // Set up visualizer:
    std::string sceneId = mExamplesList[0]->identifier;
    mVisualizer.reset(new Visualization::ProbabilisticSceneVisualization(sceneId));

    mForegroundSceneContent.initializeVisualizer(mVisualizer);
    mVisualizer->setDrawingParameters(mScaleFactor, mSigmaMultiplier, mFrameId);

    // psm offers the possibility to add foreground learners other than OCM, which might use no or different relations.
    // because of this and since the complete model is only assembled at the very end, only a partial model is used here.
}

void Evaluator::xmlOutput(boost::shared_ptr<SceneModel::Topology> pTopology)
{
    if (mXmlOutput != "none")
    {
        boost::property_tree::ptree extendedModel;

        mModel.add("<xmlattr>.run", mRunNumber);
        mModel.add("<xmlattr>.topology_id", pTopology->mIdentifier);
        mModel.add("<xmlattr>.false_positives", pTopology->mFalsePositives);
        mModel.add("<xmlattr>.false_negatives", pTopology->mFalseNegatives);
        mModel.add("<xmlattr>.average_recognition_runtime", pTopology->mAverageRecognitionRuntime);

        extendedModel.add_child("evaluator_result", mModel);

        if (mXmlOutput == "screen")
        {
            std::cout << "Partial model in Evaluator run " << mRunNumber << ":" << std::cout;
            write_xml(std::cout, extendedModel);
            std::cout << std::endl;
        }
        else if (mXmlOutput == "file")
        {
            std::string xmlFileName = mXmlFilePath + "evaluated_tree_" + boost::lexical_cast<std::string>(mRunNumber);
            write_xml(xmlFileName, extendedModel);
        }
        else throw std::runtime_error("Parameter xml_output has invalid value " + mXmlOutput);
    }
    // if value is "none": do nothing.
    mRunNumber++;
}

}
