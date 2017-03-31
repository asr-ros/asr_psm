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

#include <trainer/TopologyTreeTrainer.h>
#include <topology_generator/Topology.h>

#include <boost/property_tree/xml_parser.hpp>

#include "learner/foreground/ForegroundSceneLearner.h"
#include "learner/foreground/ocm/SceneObjectLearner.h"

#include "inference/model/foreground/ForegroundSceneContent.h"

namespace ProbabilisticSceneRecognition {

/**
 * Tests learned models of topologies against valid and invalid test sets.
 */
class Evaluator {
public:

    /**
     * Constructor.
     * @param pInferenceAlgorithm   algorithm to be used to combine inference results.
     * @param pExamplesList         list of object observations (evidences) to train on.
     * @param pLearners             learners to learn models to test.
     * @param pRecognitionThreshold threshold above which (>) a probability is seen as high enough to represent a scene has been recognized.
     * @param pXmlOutput            type of output of the learned model in xml format: "none", "screen", "file"
     * @param pXmlFilePath          if pXmlOutput is "file", the path where to store the files.
     */
    Evaluator(const std::string pInferenceAlgorithm, std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
              std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners, double pRecognitionThreshold, const std::string& pXmlOutput, const std::string& pXmlFilePath);

    /**
     * Desctuctor.
     */
    ~Evaluator() { }

    /**
     * Evaluate model learned on topology against test sets, write results to topology.
     * @param pTopology to evaluate.
     */
    void evaluate(boost::shared_ptr<SceneModel::Topology> pTopology);
    /**
     * Evaluate model learned on topology against test sets, write results to topology.
     * @param pTopology             pTopology to evaluate.
     * @param pTestSetProbabilities list of probabilities for each test set (first valid, then invalid) found during evaluation.
     */
    void evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, std::vector<double>& pTestSetProbabilities);

    /**
     * Set valid test sets.
     * @param pValidTestSets    valid test sets to set.
     */
    void setValidTestSets(const std::vector<std::vector<asr_msgs::AsrObject>>& pValidTestSets)
    {
        mValidTestSets = pValidTestSets;
    }

    /**
     * Set invalid test sets.
     * @param pInvalidTestSets  invalid test sets to set.
     */
    void setInvalidTestSets(const std::vector<std::vector<asr_msgs::AsrObject>>& pInvalidTestSets)
    {
        mInvalidTestSets = pInvalidTestSets;
    }

    /**
     * Set recognition threshold.
     * @param pRecognitionThreshold recognition threshold to set.
     */
    void setRecognitionThreshold(double pRecognitionThreshold)
    {
        mRecognitionThreshold = pRecognitionThreshold;
    }

private:
    /**
     * Returns the probability whether the given evidence represents the scene represented by the last learned model.
     * @param pEvidence the object observations to calculate the probability for.
     * @return the probability whether the given evidence represents the scene represented by the last learned model.
     */
    double getProbability(const std::vector<asr_msgs::AsrObject>& pEvidence);

    /**
     * Update the model to represent the given tree.
     * @param pTree tree to form the basis of the new model.
     */
    void update(boost::shared_ptr<SceneModel::TreeNode> pTree);

    /**
     * Output model in xml format.
     * @param pFalsePositives   number of false positives to be output.
     * @param pAvgRuntime       average recognition runtime to beo output.
     */
    void xmlOutput(double pFalsePositives, double pAvgRuntime);

    /**
     * The test sets which represent the considered scene.
     */
    std::vector<std::vector<asr_msgs::AsrObject>> mValidTestSets;
    /**
     * The test sets that resemble but do not represent the considered scene.
     */
    std::vector<std::vector<asr_msgs::AsrObject>> mInvalidTestSets;

    /**
     * Learners to learn models to test.
     */
    std::vector<boost::shared_ptr<SceneObjectLearner>> mLearners;

    /**
     * Threshold above which (>) a probability is seen as high enough to represent a scene has been recognized.
     */
    double mRecognitionThreshold;

    /**
     * A logger for runs.
     */
    std::ofstream mRuntimeLogger;   // NOT PROPERLY USED

    /**
     * Used to do inference on the learned model.
     */
    ForegroundSceneContent mForegroundSceneContent;

    /**
     * Algorithm to be used to combine inference results.
     * from { "powerset", "summarized", "multiplied", "maximum" }
     */
    std::string mInferenceAlgorithm;
    /**
     * List of object observations (evidences) to train on.
     */
    std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> mExamplesList;

    /**
     * Type of output of the learned model in xml format.
     * from { "none", "screen", "file" }
     */
    std::string mXmlOutput;

    /**
     * If pXmlOutput is "file", the path where to store the files.
     */
    std::string mXmlFilePath;

    /**
     * Number of the current evaluator run, increased whenever evaluate() is called.
     * Used in file names when mXmlOutput is "file".
     */
    unsigned int mRunNumber;

    /**
     * xml property tree describing the partial model learned on the current topoloy.
     */
    boost::property_tree::ptree mModel;
};

}
