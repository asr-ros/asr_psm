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

#include "learner/foreground/ocm/combinatorial_optimization/AbstractTopologyEvaluator.h"

#include "helper/PrintHelper.h"


namespace ProbabilisticSceneRecognition {

/**
 * Tests learned models of topologies against valid and invalid test sets.
 */
class TopologyEvaluator: public AbstractTopologyEvaluator {
public:

    /**
     * Constructor.
     * @param pExamplesList         list of object observations (evidences) to train on.
     * @param pLearners             learners to learn models to test.
     * @param pRecognitionThreshold the threshold above (>) which a scene is considered recognized.
     */
    TopologyEvaluator(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList,
              std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners, double pRecognitionThreshold);

    /**
     * Desctuctor.
     */
    ~TopologyEvaluator();

    /**
     * Evaluate model learned on topology against test sets, write results to topology.
     * @param pTopology to evaluate.
     * @param pFullyMeshed  whether the topology to evaluate is the fully meshed one.
     * @return  whether a new evaluation was neccessary.
     */
    bool evaluate(boost::shared_ptr<SceneModel::Topology> pTopology, bool pFullyMeshed = false);

private:

    /**
     * Runs recognition on evidence and returns probability recognition runtime.
     * Also writes recognition runtime and probability to TestSet if the topology used to evaluate against is the fully meshed one.
     * @param pEvidence the TestSet to recognize
     * @param pFullyMeshed whether the topology used to recognize is the fully meshed one, and whether to store recognition runtime and probability in TestSet
     * @return the probability and recognition runtime for this test set.
     */
    std::pair<double, double> recognize(boost::shared_ptr<TestSet> pEvidence, bool pFullyMeshed = false);

    /**
     * Returns the probability whether the given evidence represents the scene represented by the last learned model.
     * @param pEvidence the object observations to calculate the probability for.
     * @return the probability whether the given evidence represents the scene represented by the last learned model.
     */
    double getProbability(boost::shared_ptr<TestSet> pEvidence);

    /**
     * Update the model to represent the given tree.
     * @param pTree tree to form the basis of the new model.
     */
    void update(boost::shared_ptr<SceneModel::TreeNode> pTree);

    /**
     * Output model in xml format.
     * @param pTopology     the topology underlying the model to write and containing its evalution results.
     */
    void xmlOutput(boost::shared_ptr<SceneModel::Topology> pTopology);

    /**
     * List of object observations (evidences) to train on.
     */
    std::vector<boost::shared_ptr<ISM::ObjectSet>> mExamplesList;

    /**
     * Learners to learn models to test.
     */
    std::vector<boost::shared_ptr<SceneObjectLearner>> mLearners;

    /**
     * Number of the current evaluator run, increased whenever evaluate() is called.
     * Used in file names when mXmlOutput is "file".
     */
    unsigned int mRunNumber;

    /**
     * Class used to print lines as headers, marked with dividers.
     */
    PrintHelper mPrintHelper;

    /**
     * A logger for the runtimes for the scene objects.
     */
    std::ofstream mRuntimeLogger;

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
     * Type of output of the learned model in xml format.
     * from { "none", "screen", "file" }
     */
    std::string mXmlOutput;

    /**
     * If pXmlOutput is "file", the path where to store the files.
     */
    std::string mXmlFilePath;

    /**
     * xml property tree describing the partial model learned on the current topoloy.
     */
    boost::property_tree::ptree mModel;

    /**
     * Whether to create a runtime log.
     */
    bool mCreateRuntimeLog;

    // Visualization parameters:
    /**
     * Whether to visualize intermediate inference runs in rviz.
     */
    bool mVisualize;
    /**
     * Set true, to overwrite the visualization of results of intermediate inference runs and plot the target distributions instead.
     */
    bool mTargetingHelp;
    /**
     * The name of the frame the objects should be transformed to.
     */
    std::string mFrameId;
    /**
     * The visualization is pretty small, this scale factor enlarges it.
     */
    double mScaleFactor;
    /**
     * This factor determines the multiple of the standard deviation.
     */
    double mSigmaMultiplier;

    /**
     * Visualizer for the ForegroundSceneContent.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mVisualizer;   
};

}
