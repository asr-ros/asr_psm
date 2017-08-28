/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/TopologyManager.h"

namespace ProbabilisticSceneRecognition {

TopologyManager::TopologyManager(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList,
                const std::vector<std::string>& pObjectTypes,
                boost::shared_ptr<SceneModel::AbstractTopologyCreator> pTopologyCreator,
                boost::shared_ptr<AbstractTopologyEvaluator> pEvaluator):
    mEvaluator(pEvaluator), mTopologyCreator(pTopologyCreator), mExamplesList(pExamplesList), mObjectTypes(pObjectTypes), mHistoryIndex(0), mPrintHelper('#')
{
    ros::NodeHandle nodeHandle("~");

    // Try to get output target for the optimization history; from "none", "screen", "file".
    if(!nodeHandle.getParam("optimization_history_output", mHistoryOutput))
        throw std::runtime_error("Please specify parameter optimization_history_output when starting this node.");

    // Try to get the history file path.
    if(!nodeHandle.getParam("optimization_history_file_path", mHistoryFilePath))
        throw std::runtime_error("Please specify parameter optimization_history_file_path when starting this node.");

    // Try to get whether to revisit .
    if(!nodeHandle.getParam("revisit_topologies", mRevisitTopologies))
        throw std::runtime_error("Please specify parameter revisit_topologies when starting this node.");

    if (mHistoryOutput == "svg")
        mSVGHelper.reset(new ISM::SVGHelper(mHistoryFilePath));
}

TopologyManager::~TopologyManager()
{
    if (mHistoryOutput == "svg")
        mSVGHelper->writeResult();
}

boost::shared_ptr<SceneModel::Topology> TopologyManager::getNextNeighbour()
{
    // pop neighbour:
    boost::shared_ptr<SceneModel::Topology> nextNeighbour = mNeighbours[0];
    mNeighbours.erase(mNeighbours.begin());

    // initialize neighbour:
    if (!nextNeighbour->isTreeValid()) makeTree(nextNeighbour); // make sure the neighbour has a tree associated with it, which will be used by evaluator
    mEvaluator->evaluate(nextNeighbour);                // if nextNeighbour has already been evaluated, the Evaluator returns without rerunning the evaluation

    nextNeighbour->mUsedInOptimization = true;  // topology has been used in optimization

    mHistory[mHistoryIndex].push_back(std::pair<boost::shared_ptr<SceneModel::Topology>, bool>(nextNeighbour, false));   // add neighbour to history

    return nextNeighbour;
}

bool TopologyManager::hasNextNeighbour()
{
    return !(mNeighbours.empty());
}

void TopologyManager::setReferenceInstance(boost::shared_ptr<SceneModel::Topology> instance)
{
    if (!mTopologyCreator) throw std::runtime_error("In TopologyManager::setReferenceInstance(): TopologyCreator not initialized.");
    std::vector<boost::shared_ptr<SceneModel::Topology>> allNeighbours = mTopologyCreator->generateNeighbours(instance);
    std::vector<boost::shared_ptr<SceneModel::Topology>> selectedNeighbours;
    for (boost::shared_ptr<SceneModel::Topology> neighbour: allNeighbours)
    {
        if (!mSeenTopologies[neighbour->mIdentifier])   // no pointer assigned
        {
            selectedNeighbours.push_back(neighbour);
            mSeenTopologies[neighbour->mIdentifier] = neighbour;
        }
        else if (mRevisitTopologies || !mSeenTopologies[neighbour->mIdentifier]->mUsedInOptimization) // if topologies get revisited or if topology in list of seen topologies, but not yet used in optimization
        {
            selectedNeighbours.push_back(mSeenTopologies[neighbour->mIdentifier]);
        }
    }

    // add first step to history:
    if (mHistory.empty())
    {
        mHistory.push_back(std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>>());
        mHistoryIndex = 0;
        // add starting topology to history and mark it as selected:
        mHistory[mHistoryIndex].push_back(std::pair<boost::shared_ptr<SceneModel::Topology>, bool>(instance, true));
    }

    // mark selected topology in last step:
    for (unsigned int i = 0; i < mHistory[mHistoryIndex].size(); i++)
        if (mHistory[mHistoryIndex][i].first->mIdentifier == instance->mIdentifier)
            mHistory[mHistoryIndex][i].second = true;

    // add a new step to history:
    mHistoryIndex++;
    mHistory.push_back(std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>>());

    mNeighbours = selectedNeighbours;
}

boost::shared_ptr<SceneModel::Topology> TopologyManager::getFullyMeshedTopology()
{
    if (!mTopologyCreator) throw std::runtime_error("In TopologyManager::getFullyMeshedTopology(): TopologyCreator not initialized.");
    boost::shared_ptr<SceneModel::Topology> fullyMeshed = mTopologyCreator->generateFullyMeshedTopology();
    if (mSeenTopologies[fullyMeshed->mIdentifier])
        return mSeenTopologies[fullyMeshed->mIdentifier];   // only set up tree once
    makeTree(fullyMeshed);
    mSeenTopologies[fullyMeshed->mIdentifier] = fullyMeshed;

    return fullyMeshed;
}

std::vector<boost::shared_ptr<SceneModel::Topology>> TopologyManager::getStarTopologies()
{
    if (!mTopologyCreator) throw std::runtime_error("In TopologyManager::getStarTopologies(): TopologyCreator not initialized.");
    std::vector<boost::shared_ptr<SceneModel::Topology>> starTopologies = mTopologyCreator->generateStarTopologies();
    for (unsigned int i = 0; i < starTopologies.size(); i++)
    {
        boost::shared_ptr<SceneModel::Topology> star = starTopologies[i];
        if (!mSeenTopologies[star->mIdentifier])
        {
            makeTree(star);
            mSeenTopologies[star->mIdentifier] = star;
        }
        else starTopologies[i] = mSeenTopologies[star->mIdentifier];    // only set up tree once
    }
    return starTopologies;
}

boost::shared_ptr<SceneModel::Topology> TopologyManager::getRandomTopology()
{
    if (!mTopologyCreator) throw std::runtime_error("In TopologyManager::getRandomTopology(): TopologyCreator not initialized.");
    boost::shared_ptr<SceneModel::Topology> randomTopology = mTopologyCreator->generateRandomTopology();
    if (mSeenTopologies[randomTopology->mIdentifier])
        return mSeenTopologies[randomTopology->mIdentifier];   // only set up tree once
    makeTree(randomTopology);
    mSeenTopologies[randomTopology->mIdentifier] = randomTopology;

    return randomTopology;
}

void TopologyManager::makeTree(boost::shared_ptr<SceneModel::Topology> pTopology)
{

    mPrintHelper.printAsHeader("Generating tree from topology " + pTopology->mIdentifier);

    SceneModel::TopologyTreeTrainer tttrainer(pTopology->mRelations);
    tttrainer.addSceneGraphMessages(mExamplesList);

    tttrainer.loadTrajectoriesAndBuildTree();

    boost::shared_ptr<SceneModel::TreeNode> tree = tttrainer.getTree();

    // Print tree.
    ROS_INFO("Topology tree trainer successfully generated tree.");
    std::cout << "------------- TREE:" << std::endl;
    tree->printTreeToConsole(0);
    std::cout << "---------------------" << std::endl;

    pTopology->setTree(tree);
}

void TopologyManager::resetTopologies()
{
    for (std::pair<std::string, boost::shared_ptr<SceneModel::Topology>> topology: mSeenTopologies)
        topology.second->mUsedInOptimization = false;
    mHistory.clear();   // reset history
    mHistoryIndex = 0;
}

void TopologyManager::printHistory(unsigned int pRunNumber)
{
    if (mHistoryOutput != "none")
    {
        std::pair<unsigned int, unsigned int> bestCostIndices;
        double overallBestCost = std::numeric_limits<double>::max();

        // find and mark best topology:
        for (unsigned int i = 0; i < mHistory.size(); i++)
        {
            std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>> step = mHistory[i];

            for (unsigned int j = 0; j < step.size(); j++)
            {
                if (step[j].first->isCostValid() && step[j].first->getCost() < overallBestCost) // because of first check, this doesn't throw if there has not been set a cost.
                {
                    overallBestCost = step[j].first->getCost();
                    bestCostIndices.first = i;
                    bestCostIndices.second = j;
                }
            }
        }

        if (mHistoryOutput == "svg")
        {
            // Adapt history to ISM requirements:
            std::vector<std::vector<std::pair<ISM::TopologyPtr, unsigned int>>> history;
            std::string sceneId = mExamplesList[0]->mIdentifier; // Assuming all scene graphs are examples for the same scene.
            boost::shared_ptr<TopologyAdapter> topologyAdapter(new TopologyAdapter(mObjectTypes, sceneId));

            for (std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>> psmstep: mHistory)
            {
                std::vector<std::pair<ISM::TopologyPtr, unsigned int>> ismstep;
                for (std::pair<boost::shared_ptr<SceneModel::Topology>, bool> psmtop: psmstep)
                {
                    ISM::TopologyPtr ismtop = topologyAdapter->psmToIsm(psmtop.first);
                    // ISM has no false negatives, so SVGHelper doesn't normally output them.
                    // Here, they are added to the false positives, so the output of them instead becomes the output of the number of recognition failures.
                    ismtop->evaluationResult.falsePositives += ismtop->evaluationResult.falseNegatives;
                    unsigned int selected;
                    if (psmtop.second) selected = 1;
                    else selected = 0;
                    ismstep.push_back(std::pair<ISM::TopologyPtr, unsigned int>(ismtop, selected));
                }
                history.push_back(ismstep);
            }

            // mark best topology:
            history[bestCostIndices.first][bestCostIndices.second].second = 4;

            ISM::CostFunctionPtr<ISM::TopologyPtr> globalCostFunction(new TopologyAdapter::CostFunction());
            mSVGHelper->processHistory(history, globalCostFunction, sceneId);
            // result gets written after all runs are completed.
        }
        else
        {
            std::stringstream documentation;
            std::string title = " step 000: ";
            std::string thirddivider = "";  // a third of a divider
            for (unsigned int i = 0; i < title.length(); i++) thirddivider += "=";
            std::stringstream divider;
            divider << thirddivider << thirddivider << thirddivider << std::endl;

            // Print History:
            documentation << divider.str() << "History of optimization run " << pRunNumber << ":" << std::endl << divider.str();

            unsigned int numberOfVisitedTopologies = 0;
            for (unsigned int i = 0; i < mHistory.size(); i++)
            {
                std::string padding = "";
                if (i < 10) padding += "0";
                if (i < 100) padding += "0";
                std::string title = " step " + padding + boost::lexical_cast<std::string>(i) + ": ";
                documentation << thirddivider << title << thirddivider << std::endl;

                std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>> step = mHistory[i];

                for (unsigned int j = 0; j < step.size(); j++)
                {
                    std::pair<boost::shared_ptr<SceneModel::Topology>, bool> topologyPair = step[j];
                    boost::shared_ptr<SceneModel::Topology> topology = topologyPair.first;
                    documentation << topology->mIdentifier << ": ";
                    numberOfVisitedTopologies++;
                    if (topology->isEvaluated())
                    {
                        documentation << topology->getFalsePositives() << " false positives, " << topology->getFalseNegatives() << " false negatives, " <<  topology->getAverageRecognitionRuntime() << "s average recognition runtime.";
                        if (topology->isCostValid())
                        {
                            documentation << " cost: " << topology->getCost();
                            if (topologyPair.second) documentation << "[selected]";
                            if (i == bestCostIndices.first && j == bestCostIndices.second) documentation << "(best)";
                        }
                        else documentation << " <cost not calculated> ";
                    }
                    else documentation << " <not evaluated> ";
                    documentation << std::endl;
                }
            }
            documentation << divider.str();
            documentation << "Visited " << numberOfVisitedTopologies << " topologies  in " << mHistory.size() << " steps." << std::endl;
            documentation << "Best topology overall: " << std::endl;
            documentation << mHistory[bestCostIndices.first][bestCostIndices.second].first->mIdentifier << ": ";
            documentation << " cost: "  << overallBestCost << std::endl << divider.str();

            if (mHistoryOutput == "screen") // print to console:
            {
                mPrintHelper.printAsHeader("History:");
                std::cout << documentation.str();
            }
            else if (mHistoryOutput == "txt")
            {
                std::string filename = mHistoryFilePath + "history_of_run_" + boost::lexical_cast<std::string>(pRunNumber) + ".txt";
                std::ofstream file;
                file.open(filename);
                if (file.is_open())
                {
                    file << documentation.str();
                    file.close();
                }
                else throw std::runtime_error("Could not open file " + filename);
            }
            else throw std::runtime_error("Invalid history output type " + mHistoryOutput);
        }
    }
    // If mHistoryOutput == none: do nothing.
}

}

