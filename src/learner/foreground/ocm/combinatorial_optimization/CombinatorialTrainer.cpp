/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/CombinatorialTrainer.h"

namespace ProbabilisticSceneRecognition {


    CombinatorialTrainer::CombinatorialTrainer(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                                               std::vector<std::string> pObjectTypes, std::vector<boost::shared_ptr<const asr_msgs::AsrSceneGraph>> pExamplesList):
        mObjectTypes(pObjectTypes), mMinFalsePositives(0), mMinFPInitialized(true), // No false positives found, like in fully meshed.
        mMaxTimeInitialized(false), mMinTimeInitialized(false), mMaxFPInitialized(false)
    {
        ros::NodeHandle nodeHandle("~");

        std::string inferenceAlgorithm;
        // Try to get the name of the inference algorithm.
        if(!nodeHandle.getParam("inference_algorithm", inferenceAlgorithm))
           throw std::runtime_error("Please specify parameter inference_algorithm when starting this node.");

        // Try to get the false positives weight for the cost function.
        if(!nodeHandle.getParam("false_positives_weight", mFalsePositivesFactor))
           throw std::runtime_error("Please specify parameter false_positives_weight when starting this node.");

        // Try to get the average recognition runtime weight for the cost function.
        if(!nodeHandle.getParam("avg_recognition_time_weight", mAvgRecognitionTimeFactor))
           throw std::runtime_error("Please specify parameter avg_recognition_time_weight when starting this node.");

        int testSetCount;
        // Try to get the number of test sets to create.
        if(!nodeHandle.getParam("test_set_count", testSetCount))
           throw std::runtime_error("Please specify parameter test_set_count when starting this node.");
        if (testSetCount < 1)
            throw std::runtime_error("Parameter test_set_count should be larger than 0 (cannont create a negative amount of tets sets).");

        std::string costFunctionType;
        // Try to get the type of the cost function.
        if(!nodeHandle.getParam("cost_function", costFunctionType))
           throw std::runtime_error("Please specify parameter cost_function when starting this node.");

        std::string neighbourhoodFunctionType;
        // Try to get the type of the neighbourhood function.
        if(!nodeHandle.getParam("neighbourhood_function", neighbourhoodFunctionType))
           throw std::runtime_error("Please specify parameter neighbourhood_function when starting this node.");

        std::string optimizationAlgorithmType;
        // Try to get the type of the optimization algorithm.
        if(!nodeHandle.getParam("optimization_algorithm", optimizationAlgorithmType))
           throw std::runtime_error("Please specify parameter optimization_algorithm when starting this node.");

        // Try to get the type of starting topology selection.
        if(!nodeHandle.getParam("starting_topologies_type", mStartingTopologiesType))
           throw std::runtime_error("Please specify parameter starting_topologies_type when starting this node.");

        int numberOfStartingTopologies;
        // Try to get the number of starting topologies.
        if(!nodeHandle.getParam("number_of_starting_topologies", numberOfStartingTopologies))
           throw std::runtime_error("Please specify parameter number_of_starting_topologies when starting this node.");
        if (numberOfStartingTopologies < 1)
            throw std::runtime_error("Parameter number_of_starting_topologies should be larger than 0 (cannot use a negative amount of starting topologies).");
        mNumberOfStartingTopologies = numberOfStartingTopologies;   // cast to unsigned int. Should not lead to problems considering the check above.

        std::string xmlOutput;
        // Try to get the target of the output of the learned model in xml form.
        if(!nodeHandle.getParam("xml_output", xmlOutput))
           throw std::runtime_error("Please specify parameter xml_output when starting this node.");

        std::string xmlFilePath = "";
        // If output to file, try to get the file path.
        if (xmlOutput == "file")
            if(!nodeHandle.getParam("xml_file_path", xmlFilePath))
               throw std::runtime_error("Please specify parameter xml_file_path when starting this node.");

        bool removeRelations;
        // Try to get whether to allow the neighbourhood function to remove relations.
        if(!nodeHandle.getParam("remove_relations", removeRelations))
           throw std::runtime_error("Please specify parameter remove_relations when starting this node.");

        bool swapRelations;
        // Try to get whether to allow the neighbourhood function to swap relations.
        if(!nodeHandle.getParam("swap_relations", swapRelations))
           throw std::runtime_error("Please specify parameter swap_relations when starting this node.");

        double recognitionThreshold;
        // Try to get the probability threshold over which (>) a scene is considered as recognized.
        if(!nodeHandle.getParam("recognition_threshold", recognitionThreshold))
           throw std::runtime_error("Please specify parameter recognition_threshold when starting this node.");

        int maximumNeighbourCount;
        // Try to get the maximum neighbour count.
        if(!nodeHandle.getParam("maximum_neighbour_count", maximumNeighbourCount))
           throw std::runtime_error("Please specify parameter maximum_neighbour_count when starting this node.");
        if (maximumNeighbourCount < 1)
            throw std::runtime_error("Parameter maximum_neighbour_count should be larger than 0 (cannot use a negative amount of neighbours).");

        double objectMissingInTestSetProbability;
        // Try to get the probability for each object that, in a newly generated test set, this object is missing.
        if(!nodeHandle.getParam("object_missing_in_test_set_probability", objectMissingInTestSetProbability))
           throw std::runtime_error("Please specify parameter object_missing_in_test_set_probability when starting this node.");

        if (optimizationAlgorithmType == "HillClimbing")
        {
            // Try to get the probability that hill climbing performs a random walk.
            if(!nodeHandle.getParam("hill_climbing_random_walk_probability", mHillClimbingRandomWalkProbability))
                throw std::runtime_error("Please specify parameter hill_climbing_random_walk_probability when starting this node.");
        }
        else if (optimizationAlgorithmType == "RecordHunt")
        {
            // Try to get the initial acceptable cost delta.
            if(!nodeHandle.getParam("record_hunt_initial_acceptable_cost_delta", mInitialAcceptabeCostDelta))
                throw std::runtime_error("Please specify parameter record_hunt_initial_acceptable_cost_delta when starting this node.");

            // Try to get the probability that hill climbing performs a random walk.
            if(!nodeHandle.getParam("record_hunt_cost_delta_decrease_factor", mCostDeltaDecreaseFactor))
                throw std::runtime_error("Please specify parameter record_hunt_cost_delta_decrease_factor when starting this node.");
        }
        else if (optimizationAlgorithmType == "SimulatedAnnealing")
        {
            // Try to get the start temperature.
            if(!nodeHandle.getParam("simulated_annealing_start_temperature", mStartTemperature))
                throw std::runtime_error("Please specify parameter simulated_annealing_start_temperature when starting this node.");

            // Try to get the end temperature.
            if(!nodeHandle.getParam("simulated_annealing_end_temperature", mEndTemperature))
                throw std::runtime_error("Please specify parameter simulated_annealing_end_temperature when starting this node.");

            int repetitionsBeforeUpdate;
            // Try to get the number of repetitions before temperature update.
            if(!nodeHandle.getParam("simulated_annealing_repetitions_before_update", repetitionsBeforeUpdate))
                throw std::runtime_error("Please specify parameter simulated_annealing_repetitions_before_update when starting this node.");
            if (repetitionsBeforeUpdate < 0)
                throw std::runtime_error("Parameter simulated_annealing_repetitions_before_update should be larger than 0 (cannot repeat steps a negative amount of times).");
            mRepetitionsBeforeUpdate = repetitionsBeforeUpdate;

            // Try to get the factor by which the temperature changes.
            if(!nodeHandle.getParam("simulated_annealing_temperature_factor", mTemperatureFactor))
                throw std::runtime_error("Please specify parameter simulated_annealing_temperature_factor when starting this node.");
        }

        std::string historyOutput;
        // Try to get output target for the optimization history; from "none", "screen", "file".
        if(!nodeHandle.getParam("optimization_history_output", historyOutput))
            throw std::runtime_error("Please specify parameter optimization_history_output when starting this node.");

        std::string historyFilePath;
        // Try to get the start temperature.
        if(!nodeHandle.getParam("optimization_history_file_path", historyFilePath))
            throw std::runtime_error("Please specify parameter optimization_history_file_path when starting this node.");

        // Create evaluator
        mEvaluator.reset(new Evaluator(inferenceAlgorithm, pExamplesList, pLearners, recognitionThreshold, xmlOutput, xmlFilePath));

        unsigned int maxNbs = maximumNeighbourCount;  // possible because of check above
        //unsigned int maxNbs = mObjectTypes.size() * mObjectTypes.size() + 1;  // maximum neighbour count higher than possible number of relations
        boost::shared_ptr<SceneModel::TopologyGenerator> topgen(new SceneModel::TopologyGenerator(mObjectTypes, maxNbs, removeRelations, swapRelations));
        mTopologyManager.reset(new TopologyManager(pExamplesList, topgen , mEvaluator, historyOutput, historyFilePath));

        boost::shared_ptr<SceneModel::Topology> fullyMeshedTopology = mTopologyManager->getFullyMeshedTopology();
        if (!fullyMeshedTopology) throw std::runtime_error("In CombinatorialTrainer(): failed to get fully meshed topology from TopologyManager");

        // Create test set generator
        TestSetGenerator testSetGenerator(mEvaluator, mObjectTypes, fullyMeshedTopology, objectMissingInTestSetProbability);
        testSetGenerator.generateTestSets(pExamplesList, testSetCount); // completes the evaluator
        // evaluator complete

        // Do not change order of functions below!
        initFullyMeshedTopology();
        initStarTopologies();
        initStartingTopologies();

        initCostFunction(costFunctionType);
        initNeighbourhoodFunction(neighbourhoodFunctionType);
        initOptimizationAlgorithm(optimizationAlgorithmType);
    }

    boost::shared_ptr<SceneModel::Topology> CombinatorialTrainer::runOptimization()
    {
        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Starting optimization.");
        ROS_INFO_STREAM("===========================================================");
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();   // Get the start time.
        if (mStartingTopologies.empty()) throw std::runtime_error("No starting topologies found.");
        boost::shared_ptr<SceneModel::Topology> bestOptimizedTopology;
        double bestCost = std::numeric_limits<double>::max();
        for (unsigned int i = 0; i < mStartingTopologies.size(); i++)
        {
            boost::shared_ptr<SceneModel::Topology> startTopology = mStartingTopologies[i];
            ROS_INFO_STREAM("===========================================================");
            ROS_INFO_STREAM("Optimizing from starting topology " << i <<" (" << startTopology->mIdentifier << ")");
            ROS_INFO_STREAM("===========================================================");
            boost::shared_ptr<SceneModel::Topology> optimizedTopology = mOptimizationAlgorithm->optimize(startTopology);
            if (!optimizedTopology->mCostValid) throw std::runtime_error("In CombinatorialTrainer::runOptimization(): optimization returned topology without valid cost.");

            if (optimizedTopology->mCost < bestCost)
            {
                bestOptimizedTopology = optimizedTopology;
                bestCost = bestOptimizedTopology->mCost;
            }
            mTopologyManager->printHistory(i);
            mTopologyManager->resetTopologies();    // set all cached topologies to "unvisited"

            ROS_INFO_STREAM("===========================================================");
            ROS_INFO_STREAM("Optimization from starting topology " << i << " (" << startTopology->mIdentifier << ") complete.");
            ROS_INFO_STREAM("Best topology is " << optimizedTopology->mIdentifier << " with cost " << optimizedTopology->mCost);
            ROS_INFO_STREAM("===========================================================");
        }

        std::chrono::duration<float> diff = std::chrono::high_resolution_clock::now() - start;  // Get the stop time.
        double optimizationTime = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();

        if (!bestOptimizedTopology) // if no best optimized topology was found:
            throw std::runtime_error("In CombinatorialTrainer::runOptimization(): No best optimized topology found.");

        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Combinatorial optimization starting at " << mStartingTopologiesType << " topology complete.");
        ROS_INFO_STREAM("Optimal topology is " << bestOptimizedTopology->mIdentifier << " with a cost of " << bestOptimizedTopology->mCost);
        ROS_INFO_STREAM("Optimization took " << optimizationTime << " milliseconds.");
        ROS_INFO_STREAM("===========================================================");

        return bestOptimizedTopology;
    }

    void CombinatorialTrainer::initFullyMeshedTopology()
    {
        boost::shared_ptr<SceneModel::Topology> fm = mTopologyManager->getFullyMeshedTopology();
        mEvaluator->evaluate(fm);   // should already have been evaluated, just for safety: if topology was already evaluated, Evaluator returns immediately
        mMaxAverageRecognitionRuntime = fm->mAverageRecognitionRuntime;
        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Evaluation of fully meshed topology:");
        ROS_INFO_STREAM("Average recognition runtime (maximum of all topologies) is " << mMaxAverageRecognitionRuntime);
        ROS_INFO_STREAM("===========================================================");

        mMaxTimeInitialized = true;
    }

    void CombinatorialTrainer::initStarTopologies()
    {
        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Evaluation of star topologies:");
        ROS_INFO_STREAM("===========================================================");

        std::vector<boost::shared_ptr<SceneModel::Topology>> stars = mTopologyManager->getStarTopologies();
        if (stars.empty()) throw std::runtime_error("In CombinatorialTrainer::initStarTopologies(): no star topologies found.");

        unsigned int maxFalsePositives = 0;
        double minAverageRecognitionRuntime = mMaxAverageRecognitionRuntime;    // set to maximum for the beginning so it can only get lower
        for (boost::shared_ptr<SceneModel::Topology> star: stars)
        {
            if (!star) throw std::runtime_error("In CombinatorialTrainer::initStarTopologies(): invalid star topology.");

            mEvaluator->evaluate(star);
            double starFalsePositives = star->mFalsePositives;
            double starAverageRecognitionRuntime = star->mAverageRecognitionRuntime;
            if (starFalsePositives >= maxFalsePositives)
                maxFalsePositives = starFalsePositives;     // find maximum
            if (starAverageRecognitionRuntime <= minAverageRecognitionRuntime)
            {
                minAverageRecognitionRuntime = starAverageRecognitionRuntime;   // find mimumum
                mFastestStar = star;
            }
        }
        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Star topology evaluation complete.");
        ROS_INFO_STREAM("Maximum number of false positives is " << maxFalsePositives);
        ROS_INFO_STREAM("Minimum average runtime is " << minAverageRecognitionRuntime);
        ROS_INFO_STREAM("===========================================================");

        mMaxFalsePositives = maxFalsePositives;
        mMaxFPInitialized = true;
        mMinAverageRecognitionRuntime = minAverageRecognitionRuntime;
        mMinTimeInitialized = true;
    }

    void CombinatorialTrainer::initStartingTopologies()
    {
        ROS_INFO_STREAM("===========================================================");
        ROS_INFO_STREAM("Initializing " << mNumberOfStartingTopologies << " starting topologies of type " << mStartingTopologiesType);
        ROS_INFO_STREAM("===========================================================");
        if (mStartingTopologiesType == "FastestStar")
        {
            if (mNumberOfStartingTopologies > 1)
            {
                ROS_INFO_STREAM("There is only one fastest star topology, which will be used as the single starting topology ");
                ROS_INFO_STREAM("(change number_of_starting_topologies to 1 or start_topologies to something other than FastestStar)");
            }
            mStartingTopologies.push_back(mFastestStar);
        }
        else if (mStartingTopologiesType == "Random")
        {            
            for (unsigned int i = 0; i < mNumberOfStartingTopologies; i++)
            {
                ROS_INFO_STREAM("===========================================================");
                ROS_INFO_STREAM("Generating random topology");
                boost::shared_ptr<SceneModel::Topology> randomTopology = mTopologyManager->getRandomTopology();
                ROS_INFO_STREAM("Evaluating random topology " << randomTopology->mIdentifier);
                ROS_INFO_STREAM("===========================================================");
                mEvaluator->evaluate(randomTopology);
                ROS_INFO_STREAM("===========================================================");
                ROS_INFO_STREAM("Random topology " << randomTopology->mIdentifier << " evaluation complete.");
                ROS_INFO_STREAM("===========================================================");

                mStartingTopologies.push_back(randomTopology);
            }
        }
        else throw std::runtime_error("Parameter starting_topologies_type has invalid value " + mStartingTopologiesType);
    }

    void CombinatorialTrainer::initCostFunction(const std::string& pType)
    {
        if (pType == "WeightedSum")
        {
            ROS_INFO_STREAM("Creating WeightedSum as cost function.");
            if (!(mMaxTimeInitialized && mMinTimeInitialized && mMaxFPInitialized && mMinFPInitialized))
                throw std::runtime_error("In CombinatorialTrainer::initCostFunction: some of the minima and maxima for false positives and runtime are not initialized.");
            mCostFunction.reset(new WeightedSum(mMinFalsePositives, mMaxFalsePositives,
                                                mMinAverageRecognitionRuntime, mMaxAverageRecognitionRuntime,
                                                    mFalsePositivesFactor, mAvgRecognitionTimeFactor));
        }
        else throw std::runtime_error("Invalid cost function type " + pType);
    }

    void CombinatorialTrainer::initNeighbourhoodFunction(const std::string& pType)
    {
        if (pType == "TopologyManager")
        {
            ROS_INFO_STREAM("Using TopologyManager as neighbourhood function.");
            //mTopologyManager->setReferenceInstance(mFastestStar); // for testing: start from star with shortest average recognition runtime . unneccessary here?
            mNeighbourhoodFunction = mTopologyManager;
        }
        else throw std::runtime_error("Invalid neighbourhood function type " + pType);
    }

    void CombinatorialTrainer::initOptimizationAlgorithm(const std::string& pType)
    {
        if (pType == "HillClimbing")
        {
            ROS_INFO_STREAM("Creating ISM::HillClimbingAlgorithm as optimization algorithm.");
            mOptimizationAlgorithm.reset(new ISM::HillClimbingAlogrithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, mHillClimbingRandomWalkProbability));
            // class name misspelled as alOGrithm
        }
        else if (pType == "RecordHunt")
        {
            ROS_INFO_STREAM("Creating ISM::RecordHuntAlgorithm as optimization algorithm.");
            boost::shared_ptr<ISM::AcceptanceFunction> acceptanceFunction(new ISM::CostDeltaAcceptanceFunction(mInitialAcceptabeCostDelta, mCostDeltaDecreaseFactor));
            mOptimizationAlgorithm.reset(new ISM::RecordHuntAlgorithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, acceptanceFunction));
        }
        else if (pType == "SimulatedAnnealing")
        {
            ROS_INFO_STREAM("Creating ISM::SimulatedAnnealingAlgorithm as optimization algorithm.");
            boost::shared_ptr<ISM::CoolingSchedule> coolingSchedule(new ISM::ExponentialCoolingSchedule(mStartTemperature, mEndTemperature, mRepetitionsBeforeUpdate, mTemperatureFactor));
            mOptimizationAlgorithm.reset(new ISM::SimulatedAnnealingAlgorithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, coolingSchedule));
        }
        else throw std::runtime_error("Invalid optimization algorithm type " + pType);
    }


}
