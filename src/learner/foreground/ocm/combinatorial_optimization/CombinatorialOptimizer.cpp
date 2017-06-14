/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/CombinatorialOptimizer.h"

namespace ProbabilisticSceneRecognition {


    CombinatorialOptimizer::CombinatorialOptimizer(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                                               std::vector<std::string> pObjectTypes, std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList):
        mMinFalsePositives(0), // No false positives found, like in fully meshed.
        mMinFalseNegatives(0), // No false negatives found, like in fully meshed.
        mMinAverageRecognitionRuntime(0.0), // minimum recognition runtime is not really limited
        mMinFPInitialized(true), mMinFNInitialized(true), mMinTimeInitialized(true),
        mMaxFPInitialized(false), mMaxFNInitialized(false), mMaxTimeInitialized(false),
        mRandomRestartProbability(0.0),  // no random restart by default.
        mPrintHelper('=')
    {
        mNodeHandle = ros::NodeHandle("~");

        std::string startingTopologiesType;
        // Try to get the type of starting topology selection.
        if(!mNodeHandle.getParam("starting_topologies_type", startingTopologiesType))
           throw std::runtime_error("Please specify parameter starting_topologies_type when starting this node.");

        int numberOfStartingTopologies;
        // Try to get the number of starting topologies.
        if(!mNodeHandle.getParam("number_of_starting_topologies", numberOfStartingTopologies))
           throw std::runtime_error("Please specify parameter number_of_starting_topologies when starting this node.");
        if (numberOfStartingTopologies < 1)
            throw std::runtime_error("Parameter number_of_starting_topologies should be larger than 0 (cannot use a negative amount of starting topologies).");

        bool removeRelations;
        // Try to get whether to allow the neighbourhood function to remove relations.
        if(!mNodeHandle.getParam("remove_relations", removeRelations))
           throw std::runtime_error("Please specify parameter remove_relations when starting this node.");

        bool swapRelations;
        // Try to get whether to allow the neighbourhood function to swap relations.
        if(!mNodeHandle.getParam("swap_relations", swapRelations))
           throw std::runtime_error("Please specify parameter swap_relations when starting this node.");

        int maximumNeighbourCount;
        // Try to get the maximum neighbour count.
        if(!mNodeHandle.getParam("maximum_neighbour_count", maximumNeighbourCount))
           throw std::runtime_error("Please specify parameter maximum_neighbour_count when starting this node.");
        if (maximumNeighbourCount < 1)
            throw std::runtime_error("Parameter maximum_neighbour_count should be larger than 0 (cannot use a negative amount of neighbours).");

        // Try to get whether to use a flexible threshold:
        if(!mNodeHandle.getParam("flexible_recognition_threshold", mUseFlexibleRecognitionThreshold))
           throw std::runtime_error("Please specify parameter flexible_recognition_threshold when starting this node.");

        double baseRecognitionThreshold;
        // Try to get the probability threshold over which (>) a scene is considered as recognized.
        if(!mNodeHandle.getParam("recognition_threshold", baseRecognitionThreshold))
            throw std::runtime_error("Please specify parameter recognition_threshold when starting this node.");
        mRecognitionThreshold = std::pow(baseRecognitionThreshold, pObjectTypes.size() - 1);


        bool quitAfterTestSetEvaluation;
        // Try to get whether to quit after test set evaluation (helps with testing):
        if(!mNodeHandle.getParam("quit_after_test_set_evaluation", quitAfterTestSetEvaluation))
           throw std::runtime_error("Please specify parameter quit_after_test_set_evaluation when starting this node.");

        // Do not change order of functions below!

        // Create evaluator
        mEvaluator.reset(new TopologyEvaluator(pExamplesList, pLearners, mRecognitionThreshold));

        boost::shared_ptr<SceneModel::AbstractTopologyCreator> topgen(new SceneModel::TopologyCreator(pObjectTypes, maximumNeighbourCount, removeRelations, swapRelations));
        mTopologyManager.reset(new TopologyManager(pExamplesList, pObjectTypes, topgen , mEvaluator));

        initTestSets(pObjectTypes, pExamplesList);
        // evaluator complete

        initFullyMeshedTopologyAndFilterLoadedTestSets();

        if (quitAfterTestSetEvaluation)
        {
            ROS_INFO_STREAM("TEST SET EVALUATION COMPLETE: QUITTING EARLY.");
            exit (EXIT_SUCCESS);
        }

        initStarTopologies();
        initStartingTopologies(numberOfStartingTopologies, startingTopologiesType);

        // Initialize components od combinatorial optimization:

        std::string costFunctionType;
        // Try to get the type of the cost function.
        if(!mNodeHandle.getParam("cost_function", costFunctionType))
           throw std::runtime_error("Please specify parameter cost_function when starting this node.");
        initCostFunction(costFunctionType);

        // calculate cost for fully meshed and star topologies and set mBestOptimizedTopology to best.
        // Has to be done after initialization since the cost function could not be initialised before:
        if (!mCostFunction) throw std::runtime_error("In CombinatorialTrainer: cost function not initialised.");
        mBestOptimizedTopology = mTopologyManager->getFullyMeshedTopology();
        mCostFunction->calculateCost(mBestOptimizedTopology);

        std::vector<boost::shared_ptr<SceneModel::Topology>> starTopologies = mTopologyManager->getStarTopologies();
        boost::shared_ptr<SceneModel::Topology> bestStar = starTopologies[0];
        mCostFunction->calculateCost(bestStar);
        for (unsigned int i = 1; i < starTopologies.size(); i++)
        {
            boost::shared_ptr<SceneModel::Topology> star = starTopologies[i];
            mCostFunction->calculateCost(star);
            if (star->getCost() < bestStar->getCost())
                bestStar = star;
        }
        if (bestStar->getCost() < mBestOptimizedTopology->getCost())
            mBestOptimizedTopology = bestStar;

        ROS_INFO_STREAM("Found best topology from fully meshed and stars. Was " << mBestOptimizedTopology->mIdentifier << " with cost " << mBestOptimizedTopology->getCost());

        bool optimizeStarTopologies;
        // Try to get whether to optimize only the star topologies.
        if(!mNodeHandle.getParam("optimize_star_topologies", optimizeStarTopologies))
           throw std::runtime_error("Please specify parameter optimize_star_topologies when starting this node.");

        if (optimizeStarTopologies)
        {
            ROS_INFO_STREAM("Best star is " << bestStar->mIdentifier << " with cost " << bestStar->getCost());
            mBestStarIfRequested = bestStar;
        }

        std::string neighbourhoodFunctionType;
        // Try to get the type of the neighbourhood function.
        if(!mNodeHandle.getParam("neighbourhood_function", neighbourhoodFunctionType))
           throw std::runtime_error("Please specify parameter neighbourhood_function when starting this node.");
        initNeighbourhoodFunction(neighbourhoodFunctionType);

        std::string optimizationAlgorithmType;
        // Try to get the type of the optimization algorithm.
        if(!mNodeHandle.getParam("optimization_algorithm", optimizationAlgorithmType))
           throw std::runtime_error("Please specify parameter optimization_algorithm when starting this node.");
        initOptimizationAlgorithm(optimizationAlgorithmType);
    }

    CombinatorialOptimizer::~CombinatorialOptimizer()
    { }

    boost::shared_ptr<SceneModel::Topology> CombinatorialOptimizer::runOptimization()
    {
        if (mBestStarIfRequested)
            return mBestStarIfRequested;

        mPrintHelper.printAsHeader("Starting optimization.");
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();   // Get the start time.
        if (mStartingTopologies.empty()) throw std::runtime_error("No starting topologies found.");

        unsigned int i = 0;
        for (boost::shared_ptr<SceneModel::Topology> startingTopology: mStartingTopologies)
        {
            optimize(startingTopology, i);
            i++;

            // check whether to perform a random restart:
            if (mRandomRestartProbability > 0.0)
            {
                std::default_random_engine generator;
                std::uniform_real_distribution<double> distribution = std::uniform_real_distribution<double>(0.0, 1.0);
                while (distribution(generator) < mRandomRestartProbability)
                {
                    mPrintHelper.printAsHeader("PERFORMING RANDOM RESTART.");
                    boost::shared_ptr<SceneModel::Topology> randomStartingTopology = mTopologyManager->getRandomTopology();
                    mEvaluator->evaluate(randomStartingTopology);
                    optimize(randomStartingTopology, i);
                    i++;
                }
            }
        }

        std::chrono::duration<float> diff = std::chrono::high_resolution_clock::now() - start;  // Get the stop time.
        double optimizationTime = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();

        if (!mBestOptimizedTopology) // if no best optimized topology was found:
            throw std::runtime_error("In CombinatorialTrainer::runOptimization(): No best optimized topology found.");

        mPrintHelper.addLine("Combinatorial optimization complete.");
        mPrintHelper.addLine("Optimal topology is " + mBestOptimizedTopology->mIdentifier + " with a cost of " + boost::lexical_cast<std::string>(mBestOptimizedTopology->getCost()));
        mPrintHelper.addLine("Optimization took " + boost::lexical_cast<std::string>(optimizationTime) + " milliseconds.");
        mPrintHelper.printAsHeader();

        return mBestOptimizedTopology;
    }

    void CombinatorialOptimizer::initTestSets(const std::vector<std::string>& pObjectTypes, const std::vector<boost::shared_ptr<ISM::ObjectSet>>& pExamplesList)
    {
        // Try to get the number of test sets to use.
        if(!mNodeHandle.getParam("test_set_count", mTestSetCount))
           throw std::runtime_error("Please specify parameter test_set_count when starting this node.");
        if (mTestSetCount < 0)
            throw std::runtime_error("Parameter test_set_count should be larger than 0 (cannot use a negative amount of tets sets).");

        // Try to get the number of test sets to load.
        if(!mNodeHandle.getParam("loaded_test_set_count", mLoadedTestSetCount))
           throw std::runtime_error("Please specify parameter loaded_test_set_count when starting this node.");
        if (mLoadedTestSetCount < 0)
            throw std::runtime_error("Parameter loaded_test_set_count should be larger than 0 (cannot load a negative amount of tets sets).");
        if (mLoadedTestSetCount < mTestSetCount)
        {
            mLoadedTestSetCount = mTestSetCount;
            ROS_INFO_STREAM("Warning: loaded_test_set_count was smaller than test_set_count. Was changed to be equal.");
        }

        boost::shared_ptr<SceneModel::Topology> fullyMeshedTopology = mTopologyManager->getFullyMeshedTopology();
        if (!fullyMeshedTopology) throw std::runtime_error("In CombinatorialTrainer(): failed to get fully meshed topology from TopologyManager");

        // Create test set generator
        // Try to get the test set generator type.
        std::string testSetGeneratorType;
        if(!mNodeHandle.getParam("test_set_generator_type", testSetGeneratorType))
           throw std::runtime_error("Please specify parameter test_set_generator_type when starting this node.");

        boost::shared_ptr<TestSetGenerator> testSetGenerator;
        if (testSetGeneratorType == "absolute")
            testSetGenerator.reset(new AbsoluteTestSetGenerator(mEvaluator, fullyMeshedTopology, pObjectTypes));
        else if (testSetGeneratorType == "relative")
            testSetGenerator.reset(new RelativeTestSetGenerator(mEvaluator, fullyMeshedTopology, pObjectTypes));
        else if (testSetGeneratorType == "reference")
            testSetGenerator.reset(new ReferenceTestSetGenerator(mEvaluator, fullyMeshedTopology, pObjectTypes));
        else
            throw std::runtime_error("In CombinatorialOptimizer(): " + testSetGeneratorType + " is not a valid test_set_generator_type. "
                                     + "Valid types are absolute, relative, reference.");

        testSetGenerator->generateTestSets(pExamplesList, mTestSetCount); // completes the evaluator
    }

    void CombinatorialOptimizer::initFullyMeshedTopologyAndFilterLoadedTestSets()
    {
        boost::shared_ptr<SceneModel::Topology> fm = mTopologyManager->getFullyMeshedTopology();

        TestSetSelection testSetSelection(mEvaluator);

        // Take more sets than the test set count to restrict runtime while at the same time accounting for test set removal:
        testSetSelection.takeXTestSets(mLoadedTestSetCount);

        // should already have been evaluated if test sets were not loaded from file; if topology was already evaluated, Evaluator returns immediately:
        // if evaluate() did not return immediately, which indicates that the test sets were loaded from file instead of created, look at probabilities to determine a better recognition threshold:
        if (mEvaluator->evaluate(fm, true))
        {
            if (mUseFlexibleRecognitionThreshold)
            {
                double minValidProbability, maxInvalidProbability;
                testSetSelection.removeUnusableTestSets(minValidProbability, maxInvalidProbability);

                double flexibleRecognitionThreshold = ((minValidProbability - maxInvalidProbability) / 2) + maxInvalidProbability;
                mEvaluator->setRecognitionThreshold(flexibleRecognitionThreshold);
                mPrintHelper.printAsHeader("Recognition threshold set to flexible " + boost::lexical_cast<std::string>(flexibleRecognitionThreshold));
            }
            else    // remove test sets that would be misclassified with this recognition threshold
            {
                testSetSelection.removeMisclassifiedTestSets(mRecognitionThreshold);
            }

            // if there is still more test sets left than asked for, take only X = mTestSetCount of them.
            // if test sets were not loaded but created, there are exactly testSetCount of them in the first place.
            testSetSelection.takeXTestSets(mTestSetCount);

            // since the fully meshed topology was possibly run on different test sets, average recognition runtime needs to be recalculated:
            double recognitionRuntimeSum = 0;
            for (boost::shared_ptr<TestSet> valid: mEvaluator->getValidTestSets())
                recognitionRuntimeSum += valid->getFullyMeshedRecognitionRuntime();
            for (boost::shared_ptr<TestSet> invalid: mEvaluator->getInvalidTestSets())
                recognitionRuntimeSum += invalid->getFullyMeshedRecognitionRuntime();
            fm->setEvaluationResult(recognitionRuntimeSum / (mEvaluator->getValidTestSets().size() + mEvaluator->getInvalidTestSets().size()),  // new average recognition runtime
                                    0, 0); // since the test sets which would lead to false positives or negatives were specifically removed, there are none such.
        }

        mMaxAverageRecognitionRuntime = fm->getAverageRecognitionRuntime();

        mPrintHelper.addLine("Evaluation of fully meshed topology:");
        mPrintHelper.addLine("Average recognition runtime (maximum of all topologies) is " + boost::lexical_cast<std::string>(mMaxAverageRecognitionRuntime));
        mPrintHelper.printAsHeader();

        mMaxTimeInitialized = true;
    }

    void CombinatorialOptimizer::initStarTopologies()
    {
        mPrintHelper.printAsHeader("Evaluation of star topologies:");

        std::vector<boost::shared_ptr<SceneModel::Topology>> stars = mTopologyManager->getStarTopologies();
        if (stars.empty()) throw std::runtime_error("In CombinatorialTrainer::initStarTopologies(): no star topologies found.");

        unsigned int maxFalsePositives = 0;
        unsigned int maxFalseNegatives = 0;
        double minAverageRecognitionRuntime = mMaxAverageRecognitionRuntime;    // set to maximum for the beginning so it can only get lower
        for (boost::shared_ptr<SceneModel::Topology> star: stars)
        {
            if (!star) throw std::runtime_error("In CombinatorialTrainer::initStarTopologies(): invalid star topology.");

            mEvaluator->evaluate(star);
            double starFalsePositives = star->getFalsePositives();
            double starAverageRecognitionRuntime = star->getAverageRecognitionRuntime();
            double starFalseNegatives = star->getFalseNegatives();
            if (starFalsePositives > maxFalsePositives)
                maxFalsePositives = starFalsePositives;     // find maximum
            if (starFalseNegatives > maxFalseNegatives)
                maxFalseNegatives = starFalseNegatives;
            if (starAverageRecognitionRuntime <= minAverageRecognitionRuntime)
                minAverageRecognitionRuntime = starAverageRecognitionRuntime;   // find mimumum
        }
        mPrintHelper.addLine("Star topology evaluation complete.");
        mPrintHelper.addLine("Maximum number of false positives is " + boost::lexical_cast<std::string>(maxFalsePositives));
        mPrintHelper.addLine("Maximum number of false negatives is " + boost::lexical_cast<std::string>(maxFalseNegatives));
        mPrintHelper.addLine("Minimum average runtime is " + boost::lexical_cast<std::string>(minAverageRecognitionRuntime));
        mPrintHelper.printAsHeader();

        mMaxFalsePositives = maxFalsePositives;
        mMaxFPInitialized = true;
        mMaxFalseNegatives = maxFalseNegatives;
        mMaxFNInitialized = true;
    }

    void CombinatorialOptimizer::initStartingTopologies(unsigned int pNumberOfStartingTopologies, const std::string& pStartingTopologiesType)
    {
        mPrintHelper.printAsHeader("Initializing " + boost::lexical_cast<std::string>(pNumberOfStartingTopologies) + " starting topologies of type " + pStartingTopologiesType);

        if (pStartingTopologiesType == "BestStarOrFullyMeshed")
        {
            if (pNumberOfStartingTopologies > 1)
            {
                ROS_INFO_STREAM("There is only one best topology from stars and fully meshed, which will be used as the single starting topology ");
                ROS_INFO_STREAM("(change number_of_starting_topologies to 1 or start_topologies to something other than BestStarOrFullyMeshed)");
            }
            // use the best topology found in setting up the optimization:
            mStartingTopologies.push_back(mBestOptimizedTopology);
        }
        else if (pStartingTopologiesType == "Random")
        {            
            for (unsigned int i = 0; i < pNumberOfStartingTopologies; i++)
            {
                mPrintHelper.addLine("Generating random topology");
                boost::shared_ptr<SceneModel::Topology> randomTopology = mTopologyManager->getRandomTopology();
                mPrintHelper.addLine("Evaluating random topology " + randomTopology->mIdentifier);
                mPrintHelper.printAsHeader();
                mEvaluator->evaluate(randomTopology);
                mPrintHelper.printAsHeader("Random topology " + randomTopology->mIdentifier + " evaluation complete.");

                mStartingTopologies.push_back(randomTopology);
            }
        }
        else throw std::runtime_error("Parameter starting_topologies_type has invalid value " + pStartingTopologiesType);
    }

    void CombinatorialOptimizer::initCostFunction(const std::string& pType)
    {
        if (pType == "WeightedSum")
        {
            ROS_INFO_STREAM("Creating WeightedSum as cost function.");

            double falsePositivesFactor, avgRecognitionTimeFactor, falseNegativesFactor;

            // Try to get the false positives weight for the cost function.
            if(!mNodeHandle.getParam("false_positives_weight", falsePositivesFactor))
               throw std::runtime_error("Please specify parameter false_positives_weight when starting this node.");

            // Try to get the average recognition runtime weight for the cost function.
            if(!mNodeHandle.getParam("avg_recognition_time_weight", avgRecognitionTimeFactor))
               throw std::runtime_error("Please specify parameter avg_recognition_time_weight when starting this node.");

            // Try to get the false negatives weight for the cost function.
            if(!mNodeHandle.getParam("false_negatives_weight", falseNegativesFactor))
               throw std::runtime_error("Please specify parameter false_negatives_weight when starting this node.");

            if (!(mMaxTimeInitialized && mMinTimeInitialized && mMaxFPInitialized && mMinFPInitialized && mMaxFNInitialized && mMinFNInitialized))
                throw std::runtime_error("In CombinatorialTrainer::initCostFunction: some of the minima and maxima for false positives and runtime are not initialized.");
            mCostFunction.reset(new WeightedSum(mMinFalsePositives, mMaxFalsePositives, mMinFalseNegatives, mMaxFalseNegatives,
                                                mMinAverageRecognitionRuntime, mMaxAverageRecognitionRuntime,
                                                    falsePositivesFactor, avgRecognitionTimeFactor, falseNegativesFactor));
        }
        else throw std::runtime_error("Invalid cost function type " + pType);
    }

    void CombinatorialOptimizer::initNeighbourhoodFunction(const std::string& pType)
    {
        if (pType == "TopologyManager")
        {
            ROS_INFO_STREAM("Using TopologyManager as neighbourhood function.");
            mNeighbourhoodFunction = mTopologyManager;
        }
        else throw std::runtime_error("Invalid neighbourhood function type " + pType);
    }

    void CombinatorialOptimizer::initOptimizationAlgorithm(const std::string& pType)
    {
        if (pType == "HillClimbing")
        {
            ROS_INFO_STREAM("Creating ISM::HillClimbingAlgorithm as optimization algorithm.");

            double hillClimbingRandomWalkProbability;
            // Try to get the probability that hill climbing performs a random walk.
            if(!mNodeHandle.getParam("hill_climbing_random_walk_probability", hillClimbingRandomWalkProbability))
                throw std::runtime_error("Please specify parameter hill_climbing_random_walk_probability when starting this node.");

            // Try to get the probability that hill climbing performs a random restart (for all other algorithms, the default value of 0, no random restart, is used).
            if(!mNodeHandle.getParam("hill_climbing_random_restart_probability", mRandomRestartProbability))
                throw std::runtime_error("Please specify parameter hill_climbing_random_restart_probability when starting this node.");

            mOptimizationAlgorithm.reset(new ISM::HillClimbingAlogrithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, hillClimbingRandomWalkProbability));
            // class name misspelled as alOGrithm
        }
        else if (pType == "RecordHunt")
        {
            ROS_INFO_STREAM("Creating ISM::RecordHuntAlgorithm as optimization algorithm.");

            double initialAcceptableCostDelta, costDeltaDecreaseFactor;
            // Try to get the initial acceptable cost delta.
            if(!mNodeHandle.getParam("record_hunt_initial_acceptable_cost_delta", initialAcceptableCostDelta))
                throw std::runtime_error("Please specify parameter record_hunt_initial_acceptable_cost_delta when starting this node.");

            // Try to get the probability that hill climbing performs a random walk.
            if(!mNodeHandle.getParam("record_hunt_cost_delta_decrease_factor", costDeltaDecreaseFactor))
                throw std::runtime_error("Please specify parameter record_hunt_cost_delta_decrease_factor when starting this node.");


            boost::shared_ptr<ISM::AcceptanceFunction> acceptanceFunction(new ISM::CostDeltaAcceptanceFunction(initialAcceptableCostDelta, costDeltaDecreaseFactor));
            mOptimizationAlgorithm.reset(new ISM::RecordHuntAlgorithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, acceptanceFunction));
        }
        else if (pType == "SimulatedAnnealing")
        {
            ROS_INFO_STREAM("Creating ISM::SimulatedAnnealingAlgorithm as optimization algorithm.");

            double startTemperature, endTemperature, temperatureFactor;
            // Try to get the start temperature.
            if(!mNodeHandle.getParam("simulated_annealing_start_temperature", startTemperature))
                throw std::runtime_error("Please specify parameter simulated_annealing_start_temperature when starting this node.");

            // Try to get the end temperature.
            if(!mNodeHandle.getParam("simulated_annealing_end_temperature", endTemperature))
                throw std::runtime_error("Please specify parameter simulated_annealing_end_temperature when starting this node.");

            int repetitionsBeforeUpdate;
            // Try to get the number of repetitions before temperature update.
            if(!mNodeHandle.getParam("simulated_annealing_repetitions_before_update", repetitionsBeforeUpdate))
                throw std::runtime_error("Please specify parameter simulated_annealing_repetitions_before_update when starting this node.");
            if (repetitionsBeforeUpdate < 0)
                throw std::runtime_error("Parameter simulated_annealing_repetitions_before_update should be larger than 0 (cannot repeat steps a negative amount of times).");

            // Try to get the factor by which the temperature changes.
            if(!mNodeHandle.getParam("simulated_annealing_temperature_factor", temperatureFactor))
                throw std::runtime_error("Please specify parameter simulated_annealing_temperature_factor when starting this node.");

            boost::shared_ptr<ISM::CoolingSchedule> coolingSchedule(new ISM::ExponentialCoolingSchedule(startTemperature, endTemperature, repetitionsBeforeUpdate, temperatureFactor));
            mOptimizationAlgorithm.reset(new ISM::SimulatedAnnealingAlgorithm<boost::shared_ptr<SceneModel::Topology>>(mNeighbourhoodFunction, mCostFunction, coolingSchedule));
        }
        else throw std::runtime_error("Invalid optimization algorithm type " + pType);
    }

    void CombinatorialOptimizer::optimize(boost::shared_ptr<SceneModel::Topology> pStartingTopology, unsigned int pStartingTopologyNumber)
    {
        mPrintHelper.printAsHeader("Optimizing from starting topology " + boost::lexical_cast<std::string>(pStartingTopologyNumber) + " (" + pStartingTopology->mIdentifier + ")");

        boost::shared_ptr<SceneModel::Topology> optimizedTopology = mOptimizationAlgorithm->optimize(pStartingTopology);

        if (!mBestOptimizedTopology)
            throw std::runtime_error("In CombinatorialTrainer::runOptimization(): no valid older best topology.");
        if (optimizedTopology->getCost() <= mBestOptimizedTopology->getCost())
            mBestOptimizedTopology = optimizedTopology;

        mTopologyManager->printHistory(pStartingTopologyNumber);
        mTopologyManager->resetTopologies();    // set all cached topologies to "unvisited"

        mPrintHelper.addLine("Optimization from starting topology " + boost::lexical_cast<std::string>(pStartingTopologyNumber) + " (" + pStartingTopology->mIdentifier + ") complete.");
        mPrintHelper.addLine("Best topology is " + optimizedTopology->mIdentifier + " with cost " + boost::lexical_cast<std::string>(optimizedTopology->getCost()));
        mPrintHelper.printAsHeader();
    }
}
