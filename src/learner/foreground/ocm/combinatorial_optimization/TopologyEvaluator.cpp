/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization//TopologyEvaluator.h"

namespace ProbabilisticSceneRecognition {

TopologyEvaluator::TopologyEvaluator(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList,
                     std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners, double pRecognitionThreshold):
    AbstractTopologyEvaluator(), mExamplesList(pExamplesList), mLearners(pLearners), mRunNumber(0), mPrintHelper('-')
{
    mRecognitionThreshold = pRecognitionThreshold;

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
}

TopologyEvaluator::~TopologyEvaluator() {
    if (mCreateRuntimeLog && mRuntimeLogger.is_open())
    {
        mRuntimeLogger.close();
    }
}

bool TopologyEvaluator::evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, bool pFullyMeshed)
{
    if (!pTopology) throw std::runtime_error("In TopologyEvaluator::evaluate(): topology from argument is null pointer.");

    if (pTopology->isEvaluated()) return false;    // no new evaluation needed

    mPrintHelper.printAsHeader("Evaluating topology " + pTopology->mIdentifier + ":");

    // learn partial model:
    update(pTopology->getTree());

    // output partial model:
    xmlOutput(pTopology);

    if (mValidTestSets.empty() && mInvalidTestSets.empty()) throw std::runtime_error("In TopologyEvaluator::evaluate(): no test sets found.");

    unsigned int falseNegatives = 0;
    unsigned int falsePositives = 0;
    double recognitionRuntimeSum = 0;

    for (unsigned int i = 0; i < mValidTestSets.size(); i++)
    {
        boost::shared_ptr<TestSet> valid = mValidTestSets[i];
        ROS_INFO_STREAM("Evaluating topology " << pTopology->mIdentifier << " against valid test set " << i << "/" << mValidTestSets.size());
        std::pair<double, double> recognitionResult = recognize(valid, pFullyMeshed);
        if (recognitionResult.first <= mRecognitionThreshold) falseNegatives++; // did not recognize a valid scene.
        recognitionRuntimeSum += recognitionResult.second;
    }

    for (unsigned int j = 0; j < mInvalidTestSets.size(); j++)
    {
        boost::shared_ptr<TestSet> invalid = mInvalidTestSets[j];
        ROS_INFO_STREAM("Evaluating topology " << pTopology->mIdentifier << " against invalid test set " << j << "/" << mInvalidTestSets.size());
        std::pair<double, double> recognitionResult = recognize(invalid, pFullyMeshed);
        ROS_INFO_STREAM("Evaluated topology " << pTopology->mIdentifier << " against invalid test set " << j << "/" << mInvalidTestSets.size());
        if (recognitionResult.first > mRecognitionThreshold) falsePositives++; // did recognize an invalid scene.
        recognitionRuntimeSum += recognitionResult.second;
    }

    double avgRuntime = recognitionRuntimeSum / (mValidTestSets.size() + mInvalidTestSets.size());

    ROS_INFO_STREAM("Evaluated topology " << pTopology->mIdentifier << " against " << mValidTestSets.size() << " valid and " << mInvalidTestSets.size() << " invalid test sets.");
    ROS_INFO_STREAM("Evaluation result: " << falsePositives << " false positives, " << falseNegatives << " false negatives, " << avgRuntime << "s average recognition runtime.");

    pTopology->setEvaluationResult(avgRuntime, falsePositives, falseNegatives);

    return true;    // was evaluated.
}

std::pair<double, double> TopologyEvaluator::recognize(boost::shared_ptr<TestSet> pEvidence, bool pFullyMeshed)
{
    struct timeval start;
    struct timeval end;

    gettimeofday(&start, NULL); // get start time
    double probability = getProbability(pEvidence);
    gettimeofday(&end, NULL);   // get the stop time;

    double recognitionRuntime, seconds, useconds;
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    recognitionRuntime = seconds + useconds / 1000000;

    if (pFullyMeshed)
        pEvidence->setFullyMeshedTestResult(probability, recognitionRuntime);

    return std::pair<double, double>(probability, recognitionRuntime);
}

double TopologyEvaluator::getProbability(boost::shared_ptr<TestSet> pEvidence)
{
    std::vector<ISM::Object> evidence;
    for (ISM::ObjectPtr objPtr: pEvidence->mObjectSet->objects)
        evidence.push_back(*objPtr);
    mForegroundSceneContent.update(evidence, mRuntimeLogger);
    if (mVisualize)
    {
        if (mTargetingHelp)
            mVisualizer->drawInTargetingMode();
        else
            mVisualizer->drawInInferenceMode();
    }
    return mForegroundSceneContent.getSceneProbability(); // Foreground scene probability for partial model;
}

void TopologyEvaluator::update(boost::shared_ptr<SceneModel::TreeNode> pTree)
{
    // reset mModel:
    mModel = boost::property_tree::ptree();

    mPrintHelper.printAsHeader("Starting to learn OCM foreground model:");

    // Now just forward all examples for the scene to the scene object learners and save partial model to property tree.
    for (boost::shared_ptr<SceneObjectLearner> learner: mLearners)
    {
        learner->learn(mExamplesList, pTree);
        learner->save(mModel);
    }

    mPrintHelper.printAsHeader("Learning complete. Preparing inference.");

    // Reset and set up foreground scene content:
    mForegroundSceneContent = ForegroundSceneContent();
    mForegroundSceneContent.initializeInferenceAlgorithms(mInferenceAlgorithm);
    mForegroundSceneContent.load(mModel);   // loads partial model.

    // Set up visualizer:
    std::string sceneId = mExamplesList[0]->mIdentifier;
    mVisualizer.reset(new Visualization::ProbabilisticSceneVisualization(sceneId));

    mForegroundSceneContent.initializeVisualizer(mVisualizer);
    mVisualizer->setDrawingParameters(mScaleFactor, mSigmaMultiplier, mFrameId);

    // psm offers the possibility to add foreground learners other than OCM, which might use no or different relations.
    // because of this and since the complete model is only assembled at the very end, only a partial model is used here.
}

void TopologyEvaluator::xmlOutput(boost::shared_ptr<SceneModel::Topology> pTopology)
{
    if (mXmlOutput != "none")
    {
        boost::property_tree::ptree extendedModel;

        mModel.add("<xmlattr>.run", mRunNumber);
        mModel.add("<xmlattr>.topology_id", pTopology->mIdentifier);

        extendedModel.add_child("partial_model", mModel);

        if (mXmlOutput == "screen")
        {
            std::cout << "Partial model in TopologyEvaluator run " << mRunNumber << ":" << std::cout;
            write_xml(std::cout, extendedModel);
            std::cout << std::endl;
        }
        else if (mXmlOutput == "file")
        {
            std::string xmlFileName = mXmlFilePath + "partial_model" + boost::lexical_cast<std::string>(mRunNumber);
            write_xml(xmlFileName, extendedModel);
        }
        else throw std::runtime_error("Parameter xml_output has invalid value " + mXmlOutput);
    }
    // if value is "none": do nothing.
    mRunNumber++;
}

}
