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

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/ObjectSet.hpp>

#include <topology_creator/TopologyCreator.h>

#include "learner/foreground/ocm/combinatorial_optimization/TopologyEvaluator.h"

#include "helper/PrintHelper.h"

namespace ProbabilisticSceneRecognition {
/**
 * Generates random evidence test sets.
 */
class TestSetGenerator {

public:
    /**
     * Constructor
     * @param pEvaluator                            evaluator used to validate test sets.
     * @param pFullyMeshedTopology                  fully meshed topology used to validate test sets.
     * @param pObjectTypes                          types of objects appearing in test sets (once each).
     */
    TestSetGenerator(boost::shared_ptr<AbstractTopologyEvaluator> pEvaluator, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology, const std::vector<std::string>& pObjectTypes);

    /**
     * Desctructor.
     */
    ~TestSetGenerator();

    /**
     * Generate and validate random test sets. Set evaluator's sets accordingly.
     * @param pExamplesList list of object observations serving as basis of the test sets.
     * @param pTestSetCount number of test sets to generate.
     */
    void generateTestSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount);

private:

    /**
     * Validate whether test sets represent scene.
     * @param pTestSets the test sets to validate.
     */
    void validateSets(std::vector<boost::shared_ptr<TestSet>> pTestSets);

    /**
     * Load test sets from database file.
     * @param filename  of the database file.
     * @return the test sets loaded from the file.
     */
    std::vector<boost::shared_ptr<TestSet>> loadTestSetsFromFile(const std::string& pFilename);

    /**
     * Write generated tets sets to database file.
     * @param filename  of the database file.
     * @param testSets  the test sets to write to file
     */
    void writeTestSetsToFile(const std::string& pFilename, const std::vector<boost::shared_ptr<TestSet>>& pTestSets);

    /**
     * Simulate Occlusion of objects by cutting them from the test sets with a certain probability.
     * @param pCompleteTestSets The test sets containing all objects each.
     * @return Tets sets with missing objects.
     */
    std::vector<boost::shared_ptr<TestSet>> simulateOcclusion(std::vector<boost::shared_ptr<TestSet>> pCompleteTestSets);

protected:

    /**
     * Generate random test sets.
     * @param pExamplesList list of object observations serving as basis of the test sets.
     * @param pTestSetCount number of test sets to generate.
     */
    virtual std::vector<boost::shared_ptr<TestSet>> generateRandomSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount) = 0;

    /**
     * Set the pose of a given object relative to a reference object.
     * @param pObject       the object to have its pose transformed.
     * @param pReference    the reference object.
     */
    void setPoseOfObjectRelativeToReference(ISM::ObjectPtr pObject, ISM::ObjectPtr pReference);

private:

    /**
     * Evaluator used to validate test sets.
     */
    boost::shared_ptr<AbstractTopologyEvaluator> mEvaluator;

    /**
     * Fully meshed topology used to validate test sets.
     */
    boost::shared_ptr<SceneModel::Topology> mFullyMeshedTopology;

    /**
     * The test sets which represent the considered scene.
     */
    std::vector<boost::shared_ptr<TestSet>> mValidTestSets;

    /**
     * The test sets that resemble but do not represent the considered scene.
     */
    std::vector<boost::shared_ptr<TestSet>> mInvalidTestSets;

    /**
     * Database file to load the valid test sets from. New test sets are created if this is set to ''.
     */
    std::string mValidTestSetDbFilename;

    /**
     * Database file to load the invalid test sets from. New test sets are created if this is set to ''.
     */
    std::string mInvalidTestSetDbFilename;

    /**
     * Database file to write newly created valid testsets to. Test sets are not written if they were loaded from files or this parameter is set to ''.
     */
    std::string mWriteValidTestSetsFilename;

    /**
     * Database file to write newly created invalid testsets to. Test sets are not written if they were loaded from files or this parameter is set to ''.
     */
    std::string mWriteInvalidTestSetsFilename;

    /**
     * ID of the scene to be learned.
     */
    std::string mSceneId;

    /**
     * Probability with which an object does not appear in a test set. set to 0 to have all objects appear in each test set
     */
    double mObjectMissingInTestSetProbability;

protected:

    /**
     * Types of objects appearing in test sets (once each).
     */
    std::vector<std::string> mTypes;

    /**
     * Class used to print lines as headers, marked with dividers.
     */
    PrintHelper mPrintHelper;

};

}
