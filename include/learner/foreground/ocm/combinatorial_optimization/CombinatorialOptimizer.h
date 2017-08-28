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

#include <boost/filesystem.hpp> // needed by RecordHunt, SimulatedAnnealing

#include <topology_creator/TopologyCreator.h>

#include <ISM/combinatorial_optimization/HillClimbingAlogrithm.hpp>
#include <ISM/combinatorial_optimization/RecordHuntAlgorithm.hpp>
#include <ISM/combinatorial_optimization/CostDeltaAcceptanceFunction.hpp>
#include <ISM/combinatorial_optimization/SimulatedAnnealingAlgorithm.hpp>
#include <ISM/combinatorial_optimization/ExponentialCoolingSchedule.hpp>

#include <ISM/common_type/ObjectSet.hpp>

#include "learner/foreground/ocm/combinatorial_optimization/WeightedSum.h"
#include "learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.h"
#include "learner/foreground/ocm/combinatorial_optimization/AbsoluteTestSetGenerator.h"
#include "learner/foreground/ocm/combinatorial_optimization/RelativeTestSetGenerator.h"
#include "learner/foreground/ocm/combinatorial_optimization/ReferenceTestSetGenerator.h"
#include "learner/foreground/ocm/combinatorial_optimization/TopologyManager.h"
#include "learner/foreground/ocm/combinatorial_optimization/TestSetSelection.h"

#include "helper/PrintHelper.h"

namespace ProbabilisticSceneRecognition {

/**
 * Uses combinatorial optimization to train a relation graph that is optimal
 * considering false negatives and average recognition runtime on random test sets.
 */
class CombinatorialOptimizer {

public:
    /**
     * Constructor.
     * @param pLearners     The learners to be used to learn OCM models for the relation graphs considered.
     * @param pObjectTypes  All possible object types appearing in the relation graphs (once each)
     * @param pExamplesList The list of evidences to train on.
     */
    CombinatorialOptimizer(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                         std::vector<std::string> pObjectTypes, std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList);

    /**
     * Destructor.
     */
    ~CombinatorialOptimizer();

    /**
     * Perform optimization.
     * @return the optimized relation graph topology.
     */
    boost::shared_ptr<SceneModel::Topology> runOptimization();

private:

    /**
     * Initialize a test set generator and set evaluator test sets.
     * @param pObjectTypes  All possible object types appearing in the relation graphs (once each)
     * @param pExamplesList The list of evidences to train on.
     */
    void initTestSets(const std::vector<std::string>& pObjectTypes, const std::vector<boost::shared_ptr<ISM::ObjectSet>>& pExamplesList);

    /**
     * Initialize the fully meshed topology used to divide the test sets into valid and invalid ones
     * and find the worst recognition runtime.
     * Also deals with loaded test sets, if those were not created by the program.
     */
    void initFullyMeshedTopologyAndFilterLoadedTestSets();

    /**
     * Initialize the star topologies used to find the worst false positive numbers
     * and fastest average runtimes.
     */
    void initStarTopologies();

    /**
     * Initialize starting topologies to perform optimization from.
     * @param pNumberOfStartingTopologies   number of starting topologies to initialize.
     * @param pStartingTopologiesType       type of algorithm to use to generate starting topologies.
     */
    void initStartingTopologies(unsigned int pNumberOfStartingTopologies, const std::string& pStartingTopologiesType);

    /**
     * Initialize the cost function used to judge topologies.
     * @param pType type of the cost function to be used: only "WeightedSum".
     */
    void initCostFunction(const std::string& pType);

    /**
     * Initialize neighbourhood function used to generate neighbours of currently selected topology during optimizations.
     * @param pType type of the neighbourhood function to be used: only "TopologyManager".
     */
    void initNeighbourhoodFunction(const std::string& pType);

    /**
     * Initialize optimization algorithm to be used.
     * @param pType type of the optimization algorithm to be used: from "HillClimbing", "RecordHunt", "SimulatedAnnealing".
     */
    void initOptimizationAlgorithm(const std::string& pType);

    /**
     * optimize from the given topology. Sets new mBestOptimizedTopology if a better one has been found.
     * @param pStartingTopology         to optimize from.
     * @param pStartingTopologyNumber   the number of the starting topology, for output.
     */
    void optimize(boost::shared_ptr<SceneModel::Topology> pStartingTopology, unsigned int pStartingTopologyNumber);

    /**
     * Handle to ros node for parameter access.
     */
    ros::NodeHandle mNodeHandle;

    /**
     * list of all star topologies for the given object types.
     */
    std::vector<boost::shared_ptr<SceneModel::Topology>> mStarTopologies;

    /**
     * fully meshed topology for all given object types.
     */
    boost::shared_ptr<SceneModel::Topology> mFullyMeshedTopology;

    /**
     * Starting topologies to perform optimization from.
     */
    std::vector<boost::shared_ptr<SceneModel::Topology>> mStartingTopologies;

    /**
     * Evaluator used to check the learned models of the topologies against the test sets.
     */
    boost::shared_ptr<AbstractTopologyEvaluator> mEvaluator;

    /**
     * The cost function used to judge topologies.
     */
    boost::shared_ptr<ISM::CostFunction<boost::shared_ptr<SceneModel::Topology>>> mCostFunction;

    /**
     * Topology Manager used to get fully meshed, star and starting topologies.
     * If parameter "neighbourhood_function" from yaml is set to "TopologyManager", also used as neighbourhood function for optimization.
     */
    boost::shared_ptr<TopologyManager> mTopologyManager;
    /**
     * Neighbourhood function used to generate neighbours of currently selected topology during optimizations.
     */
    boost::shared_ptr<ISM::NeighbourhoodFunction<boost::shared_ptr<SceneModel::Topology>>> mNeighbourhoodFunction;

    /**
     * Optimization algorithm to be used.
     */
    boost::shared_ptr<ISM::OptimizationAlgorithm<boost::shared_ptr<SceneModel::Topology>>> mOptimizationAlgorithm;

    /**
     * Minimum number of false positives. Set to 0.
     */
    unsigned int mMinFalsePositives;        // set in ctor
    /**
     * Minimum number of false negatives. Set to 0.
     */
    unsigned int mMinFalseNegatives;        // set in ctor
    /**
     * Minimum average recognition runtime.
     */
    double mMinAverageRecognitionRuntime;   // set in ctor
    /**
     * Maximum number of false positives.
     * All topologies with more can be replaced with a star topology, which will be faster.
     */
    unsigned int mMaxFalsePositives;        // set in initStar
    /**
     * Maximum number of false negatives.
     * All topologies with more can be replaced with a star topology, which will be faster.
     */
    unsigned int mMaxFalseNegatives;        // set in initStar
    /**
     * Maximum average recognition runtime.
     * All topologies with average runtimes above can be replaced by the fully meshed one, which has no false positives.
     */
    double mMaxAverageRecognitionRuntime;   // set in initFullyMeshed
    /**
     * Whether the maxima and minima have been properly initialized yet.
     */
    bool mMinFPInitialized, mMinFNInitialized, mMinTimeInitialized, mMaxFPInitialized, mMaxFNInitialized, mMaxTimeInitialized;

    /**
     * The best topology considered so far.
     */
    boost::shared_ptr<SceneModel::Topology> mBestOptimizedTopology;

    /**
     * The worst star topology. This pointer only gets set when it was requested via parameter get_worst_star_topology
     * that only star topologies are compared (for testing purposes).
     */
    boost::shared_ptr<SceneModel::Topology> mWorstStarIfRequested;

    /**
     * Probability that, in HillClimbing, a random restart is performed. Set to 0 for all other algorithms.
     */
    double mRandomRestartProbability;

    /**
     * When test sets are loaded from a file, set this to true to select the recognition threshold based on the test sets.
     */
    bool mUseFlexibleRecognitionThreshold;

    /**
     * Class used to print lines as headers, marked with dividers.
     */
    PrintHelper mPrintHelper;

    /**
     * How many test sets to use in optimization.
     */
    int mTestSetCount;

    /**
     * How many test sets loaded from the database to use at first.
     * Reduced to test_set_count later, but since some loaded sets may be misclassified and removed, it can be useful to load more than test_set_count.
     */
    int mLoadedTestSetCount;

    /**
     * The probability above which a test set is considered as valid.
     */
    double mRecognitionThreshold;
};

}
