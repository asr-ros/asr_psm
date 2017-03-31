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

#include <topology_generator/Topology.h>
#include <topology_generator/TopologyGenerator.h>

#include <ISM/combinatorial_optimization/NeighbourhoodFunction.hpp>

#include "learner/foreground/ocm/combinatorial_optimization/Evaluator.h"

namespace ProbabilisticSceneRecognition {

/**
 * Class used to access topologies of certain types and neighbour topologies
 * for use in combinatorial optimization.
 * All topologies get evaluated before they are returned.
 * Keeps track of the history of topolgy access and thus optimization.
 */
class TopologyManager: public ISM::NeighbourhoodFunction<boost::shared_ptr<SceneModel::Topology>> {
public:
    /**
     * Constructor.
     * @param pExamplesList         List of object evidence used as basis of the topologies.
     * @param pTopologyGenerator    Generator creating the topologies.
     * @param pEvaluator            Evaluator to evaluate the topologies.
     */
    TopologyManager(std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList,
                    boost::shared_ptr<SceneModel::TopologyGenerator> pTopologyGenerator,
                    boost::shared_ptr<Evaluator> pEvaluator,
                    std::string pHistoryOutput, std::string pHistoryFilePath);

    /**
     * Desctructor.
     */
    ~TopologyManager() { }

    /**
     * Get the next neighbour of the current reference topology.
     * @return the next neighbour of the current reference topology.
     */
    virtual boost::shared_ptr<SceneModel::Topology> getNextNeighbour();
    /**
     * Check whether the current reference topology has (unseen) neighbours.
     * @return whether the current reference topology has neighbours.
     */
    virtual bool hasNextNeighbour();
    /**
     * Set a new reference topology to get the neighbours for.
     * @param instance  the new reference topology.
     */
    virtual void setReferenceInstance(boost::shared_ptr<SceneModel::Topology> instance);

    /**
     * Get fully meshed topology for the object types in the evidence list.
     * @return fully meshed topology.
     */
    boost::shared_ptr<SceneModel::Topology> getFullyMeshedTopology();
    /**
     * Get all the star topologies for the object types in the evidence list.
     * @return star topologies.
     */
    std::vector<boost::shared_ptr<SceneModel::Topology>> getStarTopologies();
    /**
     * Get a random topology for the object types in the evidence list.
     * @return a random topology.
     */
    boost::shared_ptr<SceneModel::Topology> getRandomTopology();

    /**
     * Make a tree with references out of a given topology.
     * @param pTopology to transform into a tree with references.
     */
    void makeTree(boost::shared_ptr<SceneModel::Topology> pTopology);
    /**
     * Forget which topologies have already been used in optimization before running another round.
     * Keeps cached topologies so thex don't have to be rebuilt and reevaluated.
     */
    void resetTopologies();

    /**
     * Print history of topology access since construction or last call of resetTopologies().
     * @param pRunNumber    Unique ID of current run.
     */
    void printHistory(unsigned int pRunNumber);

private:

    /**
     * List of all unvisited neighbours of current reference topology.
     */
    std::vector<boost::shared_ptr<SceneModel::Topology>> mNeighbours;
    /**
     * List of all topologies visited since construction or last call of resetTopologies().
     */
    std::map<std::string, boost::shared_ptr<SceneModel::Topology>> mSeenTopologies;

    /**
     * Evaluator to evaluate the topologies.
     */
    boost::shared_ptr<Evaluator> mEvaluator;
    /**
     * @brief Generator creating the topologies.
     */
    boost::shared_ptr<SceneModel::TopologyGenerator> mTopologyGenerator;
    /**
     * List of object evidence used as basis of the topologies.
     */
    std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> mExamplesList;

    /**
     * The history of topolgy access and thus optimization.
     * The boolean in the pair indicates whether the topology was selected for use as reference in the next step.
     */
    std::vector<std::vector<std::pair<boost::shared_ptr<SceneModel::Topology>, bool>>> mHistory;
    /**
     * Index of the current optimization step in mHistory.
     * increased by setReferenceInstance().
     */
    unsigned int mHistoryIndex;

    /**
     * Output target of the optimization run history.
     * From "none", "screen", "file".
     */
    std::string mHistoryOutput;

    /**
     * Path to the files to write the histories into if mHistoryOutput is set to "file".
     */
    std::string mHistoryFilePath;
};

}
