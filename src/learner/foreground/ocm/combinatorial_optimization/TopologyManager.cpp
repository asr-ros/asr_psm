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

TopologyManager::TopologyManager(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
                boost::shared_ptr<SceneModel::TopologyGenerator> pTopologyGenerator,
                boost::shared_ptr<Evaluator> pEvaluator,
                std::string pHistoryOutput, std::string pHistoryFilePath):
    mEvaluator(pEvaluator), mTopologyGenerator(pTopologyGenerator), mExamplesList(pExamplesList), mHistoryIndex(0), mHistoryOutput(pHistoryOutput), mHistoryFilePath(pHistoryFilePath)
{ }

boost::shared_ptr<SceneModel::Topology> TopologyManager::getNextNeighbour()
{
    // pop neighbour:
    boost::shared_ptr<SceneModel::Topology> nextNeighbour = mNeighbours[0];
    mNeighbours.erase(mNeighbours.begin());

    // initialize neighbour:
    if (!nextNeighbour->mTree) makeTree(nextNeighbour); // make sure the neighbour has a tree associated with it, which will be used by evaluator
    mEvaluator->evaluate(nextNeighbour);                // if nextNeighbour has already been evaluated, the Evaluator returns without rerunning the evaluation

    //mSeenTopologies[nextNeighbour->mIdentifier]->mUsedInOptimization = true;  // topology has been used in optimization
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
    if (!mTopologyGenerator) throw std::runtime_error("In TopologyManager::setReferenceInstance(): TopologyGenerator not initialized.");
    std::vector<boost::shared_ptr<SceneModel::Topology>> allNeighbours = mTopologyGenerator->generateNeighbours(instance);
    std::vector<boost::shared_ptr<SceneModel::Topology>> selectedNeighbours;
    for (boost::shared_ptr<SceneModel::Topology> neighbour: allNeighbours)
    {
        if (!mSeenTopologies[neighbour->mIdentifier])   // no pointer assigned
        {
            selectedNeighbours.push_back(neighbour);
            mSeenTopologies[neighbour->mIdentifier] = neighbour;
        }
        else if (!mSeenTopologies[neighbour->mIdentifier]->mUsedInOptimization) // if in list of seen topologies, but not yet used in optimization
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
    if (!mTopologyGenerator) throw std::runtime_error("In TopologyManager::getFullyMeshedTopology(): TopologyGenerator not initialized.");
    boost::shared_ptr<SceneModel::Topology> fullyMeshed = mTopologyGenerator->generateFullyMeshedTopology();
    if (mSeenTopologies[fullyMeshed->mIdentifier])
        return mSeenTopologies[fullyMeshed->mIdentifier];   // only set up tree once
    makeTree(fullyMeshed);
    mSeenTopologies[fullyMeshed->mIdentifier] = fullyMeshed;

    return fullyMeshed;
}

std::vector<boost::shared_ptr<SceneModel::Topology>> TopologyManager::getStarTopologies()
{
    if (!mTopologyGenerator) throw std::runtime_error("In TopologyManager::getStarTopologies(): TopologyGenerator not initialized.");
    std::vector<boost::shared_ptr<SceneModel::Topology>> starTopologies = mTopologyGenerator->generateStarTopologies();
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
    if (!mTopologyGenerator) throw std::runtime_error("In TopologyManager::getRandomTopology(): TopologyGenerator not initialized.");
    boost::shared_ptr<SceneModel::Topology> randomTopology = mTopologyGenerator->generateRandomTopology();
    if (mSeenTopologies[randomTopology->mIdentifier])
        return mSeenTopologies[randomTopology->mIdentifier];   // only set up tree once
    makeTree(randomTopology);
    mSeenTopologies[randomTopology->mIdentifier] = randomTopology;

    return randomTopology;
}

void TopologyManager::makeTree(boost::shared_ptr<SceneModel::Topology> pTopology)
{
    ROS_INFO_STREAM("===========================================================");
    ROS_INFO_STREAM("Generating tree from topology " << pTopology->mIdentifier);
    ROS_INFO_STREAM("===========================================================");
    SceneModel::TopologyTreeTrainer tttrainer(pTopology->mRelations);
    tttrainer.addSceneGraphMessages(mExamplesList);

    tttrainer.loadTrajectoriesAndBuildTree();

    boost::shared_ptr<SceneModel::TreeNode> tree = tttrainer.getTree();

    // Print tree.
    ROS_INFO("Topology tree trainer successfully generated tree.");
    std::cout << "------------- TREE:" << std::endl;
    tree->printTreeToConsole(0);
    std::cout << "---------------------" << std::endl;

    pTopology->mTree = tree;
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
        std::stringstream documentation;

        // Print History:
        documentation << "===========================================================" << std::endl;
        documentation << "History of optimization run " << pRunNumber << ":" << std::endl;
        documentation << "===========================================================" << std::endl;

        double overallBestCost = std::numeric_limits<double>::max();

        for (unsigned int i = 0; i < mHistory.size(); i++)
        {
            documentation << "====================== step " << i << ": ============================" << std::endl;
            std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>> step = mHistory[i];

            double bestCost = std::numeric_limits<double>::max();

            // find best cost of this step:
            for (unsigned int i = 0; i < step.size(); i++)
                if (step[i].first->mCostValid && step[i].first->mCost < bestCost)
                    bestCost = step[i].first->mCost;
            if (bestCost < overallBestCost) overallBestCost = bestCost;

            for (std::pair<boost::shared_ptr<SceneModel::Topology>, bool> topologyPair: step)
            {
                boost::shared_ptr<SceneModel::Topology> topology = topologyPair.first;
                documentation << topology->mIdentifier << ": ";
                if (topology->mEvaluated)
                {
                    documentation << topology->mFalsePositives << " false positives, " << topology->mAverageRecognitionRuntime << "s average recognition runtime.";
                    if (topology->mCostValid)
                    {
                        documentation << " cost: " << topology->mCost;
                        if (topology->mCost == overallBestCost) documentation << " (best so far)";
                        else if (topology->mCost == bestCost) documentation << " (best of step)";
                        if (topologyPair.second) documentation << "[selected]";
                    }
                }
                else documentation << " - ";
                documentation << std::endl;
            }
        }
        documentation << "===========================================================" << std::endl;
        documentation << "best cost overall: " << overallBestCost << std::endl;

        documentation << "===========================================================" << std::endl;

        if (mHistoryOutput == "screen") // print to console:
            std::cout << documentation.str();
        else if (mHistoryOutput == "file")
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
    // If mHistoryOutput == none: do nothing.
}

}

