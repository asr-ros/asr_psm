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

#include <topology_generator/TopologyGenerator.h>

#include <ISM/combinatorial_optimization/HillClimbingAlogrithm.hpp>
#include <ISM/combinatorial_optimization/RecordHuntAlgorithm.hpp>
#include <ISM/combinatorial_optimization/CostDeltaAcceptanceFunction.hpp>
#include <ISM/combinatorial_optimization/SimulatedAnnealingAlgorithm.hpp>
#include <ISM/combinatorial_optimization/ExponentialCoolingSchedule.hpp>


#include "learner/foreground/ocm/combinatorial_optimization/WeightedSum.h"
#include "learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.h"
#include "learner/foreground/ocm/combinatorial_optimization/TopologyManager.h"

namespace ProbabilisticSceneRecognition {

/**
 * Uses combinatorial optimization to train a relation graph that is optimal
 * considering false negatives and average recognition runtime on random test sets.
 */
class CombinatorialTrainer {

public:
    /**
     * Constructor.
     * @param pLearners     The learners to be used to learn OCM models for the relation graphs considered.
     * @param pObjectTypes  All possible object types appearing in the relation graphs (once each)
     * @param pExamplesList The list of evidences to train on.
     */
    CombinatorialTrainer(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                         std::vector<std::string> pObjectTypes, std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList);

    /**
     * Destructor.
     */
    ~CombinatorialTrainer() { }

    /**
     * Perform optimization.
     * @return the optimized relation graph topology.
     */
    boost::shared_ptr<SceneModel::Topology> runOptimization();

private:
    /**
     * Initialize the fully meshed topology used to divide the test sets into valid and invalid ones
     * and find the worst recognition runtime.
     */
    void initFullyMeshedTopology();

    /**
     * Initialize the star topologies used to find the worst false positive numbers
     * and fastest average runtimes.
     */
    void initStarTopologies();

    /**
     * Initialize starting topologies to perform optimization from.
     */
    void initStartingTopologies();

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
    boost::shared_ptr<Evaluator> mEvaluator;

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
     * Maximum average recognition runtime.
     * All topologies with average runtimes above can be replaced by the fully meshed one, which has no false positives.
     */
    double mMaxAverageRecognitionRuntime;   // set in initFullyMeshed
    /**
     * Maximum number of false positives.
     * All topologies with more can be replaced with a star topology, which will be faster.
     */
    unsigned int mMaxFalsePositives;        // set in initStar
    /**
     * Minimum average recognition runtime.
     */
    double mMinAverageRecognitionRuntime;   // set in initStar
    /**
     * Minimum number of false positives. Set to 0.
     */
    unsigned int mMinFalsePositives;        // set in ctor
    /**
     * The factor to weigh the number of false positives with in cost function WeightedSum.
     */
    double mFalsePositivesFactor;           // from ROS parameter server
    /**
     * The factor to weigh the average recognition runtime with in cost function WeightedSum.
     */
    double mAvgRecognitionTimeFactor;       // from ROS parameter server
    /**
     * Whether the maxima and minima have been properly initialized yet.
     */
    bool mMaxTimeInitialized, mMinTimeInitialized, mMaxFPInitialized, mMinFPInitialized;

    /**
     * The type of the algorithm used to select starting topologies: "Random".
     */
    std::string mStartingTopologiesType;    // from ROS parameter server
    /**
     * Number of starting topologies to use.
     * For each, optimization will be run separately,
     * results will be compared and best (lowest) will be seleced.
     */
    unsigned int mNumberOfStartingTopologies;   // from ROS parameter server

    // for HillClimbing:
    /**
     * The probability that HillClimbing, if it is selected as the optimization algorithm
     * in initOptimizationAlgorithm(), will perform a random walk during optimization,
     * i.e. randomly accept a worse topology.
     */
    double mHillClimbingRandomWalkProbability;     // from ROS parameter server

    /**
     * All possible object types appearing in the relation graphs (once each).
     */
    std::vector<std::string> mObjectTypes;  // could be retrieved from examples list, but this would lead to an unneccessary additional loop over that list

    // for RecordHunt:
    /**
     * The initial cost acceptance delta for RecordHunt.
     */
    double mInitialAcceptabeCostDelta;
    /**
     * The cost delta decrease factor for RecordHunt.
     */
    double mCostDeltaDecreaseFactor;

    // for SimulatedAnnealing:
    /**
     * The start temperature for SimulatedAnnealing.
     */
    double mStartTemperature;
    /**
     * The end temperature for SimulatedAnnealing.
     */
    double mEndTemperature;
    /**
     * The number of repetitions before temperature update in SimulatedAnnealing.
     */
    unsigned int mRepetitionsBeforeUpdate;
    /**
     * The factor by which the temperature changes in SimulatedAnnealing.
     */
    double mTemperatureFactor;

    //for testing:
    /**
     * A pointer to the star topology with the shortest average recognition runtime
     * to possibly be used as starting topology.
     */
    boost::shared_ptr<SceneModel::Topology> mFastestStar;

};

}
