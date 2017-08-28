/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.h"

namespace ProbabilisticSceneRecognition {

TestSetGenerator::TestSetGenerator(boost::shared_ptr<AbstractTopologyEvaluator> pEvaluator, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology, const std::vector<std::string>& pObjectTypes):
    mEvaluator(pEvaluator), mFullyMeshedTopology(pFullyMeshedTopology), mTypes(pObjectTypes),
    mPrintHelper('+')
{
    ros::NodeHandle nodeHandle("~");

    // Try to get the valid test set source database filename:
    if(!nodeHandle.getParam("valid_test_set_db_filename", mValidTestSetDbFilename))
        throw std::runtime_error("Please specifiy parameter valid_test_set_db_filename when starting this node.");
    // Try to get the invalid test set source database filename:
    if(!nodeHandle.getParam("invalid_test_set_db_filename", mInvalidTestSetDbFilename))
        throw std::runtime_error("Please specifiy parameter invalid_test_set_db_filename when starting this node.");
    // Try to get the valid test set target database filename:
    if(!nodeHandle.getParam("write_valid_test_sets_filename", mWriteValidTestSetsFilename))
        throw std::runtime_error("Please specifiy parameter write_valid_test_sets_filename when starting this node.");
    // Try to get the invalid test set target database filename:
    if(!nodeHandle.getParam("write_invalid_test_sets_filename", mWriteInvalidTestSetsFilename))
        throw std::runtime_error("Please specifiy parameter write_invalid_test_sets_filename when starting this node.");

    // Try to get the probability for each object that, in a newly generated test set, this object is missing.
    if(!nodeHandle.getParam("object_missing_in_test_set_probability", mObjectMissingInTestSetProbability))
       throw std::runtime_error("Please specify parameter object_missing_in_test_set_probability when starting this node.");
}

TestSetGenerator::~TestSetGenerator()
{ }

void TestSetGenerator::generateTestSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount)
{
    mSceneId = pExamplesList[0]->mIdentifier;

    // if the file names are empty: generate new test sets.
    if (mValidTestSetDbFilename == "" || mInvalidTestSetDbFilename == "")
    {
        std::vector<boost::shared_ptr<TestSet>> randomTestSets = generateRandomSets(pExamplesList, pTestSetCount);

        // if requested, simulate occlusion:
        if (mObjectMissingInTestSetProbability > 0)
            randomTestSets = simulateOcclusion(randomTestSets);

        validateSets(randomTestSets);

        if (mWriteValidTestSetsFilename != "")
            writeTestSetsToFile(mWriteValidTestSetsFilename, mValidTestSets);
        if (mWriteInvalidTestSetsFilename != "")
            writeTestSetsToFile(mWriteInvalidTestSetsFilename, mInvalidTestSets);
    }
    else // get the test sets from the databases. Might be more than pTestSetCount.
    {
        mValidTestSets = loadTestSetsFromFile(mValidTestSetDbFilename);
        mInvalidTestSets = loadTestSetsFromFile(mInvalidTestSetDbFilename);
    }

    mEvaluator->setValidTestSets(mValidTestSets);    // update with newfound valid test sets
    mEvaluator->setInvalidTestSets(mInvalidTestSets);    // initialize properly
}

void TestSetGenerator::validateSets(std::vector<boost::shared_ptr<TestSet>> pTestSets)
{
    // Use fully meshed topology to calculate probabilities:
    // set all tests as valid (so that fully meshed topology, which cannot have false positives, gets its evaluation result set already)
    mEvaluator->setValidTestSets(pTestSets);
    mEvaluator->setInvalidTestSets(std::vector<boost::shared_ptr<TestSet>>());
    double recognitionThreshold = mEvaluator->getRecognitionThreshold();    // save actual recognition threshold.
    mEvaluator->setRecognitionThreshold(-1);    // so every test set gets recognized

    ROS_INFO_STREAM("Recognizing test sets with fully meshed topology.");
    mEvaluator->evaluate(mFullyMeshedTopology, true);
    if (mFullyMeshedTopology->getFalsePositives() != 0)
        throw std::runtime_error("In TestSetGenerator::validateSets(): Error when evaluating fully meshed topology: has false positives, should have none");

    if (!mEvaluator->getInvalidTestSets().empty())
        throw std::runtime_error("In TestSetGenerator::validateSets(): Error when evaluating fully meshed topology: found invalid test sets when those should not have been initialized yet.");

    double maxProbability = 0;
    double minProbability = 2;  // Minumum probability greater than zero. Set higher than possible maximum so it will be lowered in each case, and if invalid probability 2 is returned, this indicates that no probability > 0 was found.
    unsigned int zeroSets = 0;
    for (boost::shared_ptr<TestSet> testSet: pTestSets)
    {
        double probability = testSet->getFullyMeshedProbability();
        if (probability > maxProbability) maxProbability = probability;
        if (probability == 0) zeroSets++;
        else if (probability < minProbability) minProbability = probability;    // set lowest non-zero probability as new minimum
    }

    std::vector<boost::shared_ptr<TestSet>> validTestSets;
    std::vector<boost::shared_ptr<TestSet>> invalidTestSets;

    for (unsigned int i = 0; i < pTestSets.size(); i++)
    {
        if (pTestSets[i]->getFullyMeshedProbability() > recognitionThreshold)
            validTestSets.push_back(pTestSets[i]);
        else invalidTestSets.push_back(pTestSets[i]);
    }

    mPrintHelper.addLine("Test set creation complete.");
    mPrintHelper.addLine("Maximum probability: " + boost::lexical_cast<std::string>(maxProbability));
    if (minProbability == 2)
        minProbability = std::numeric_limits<double>::quiet_NaN();
    mPrintHelper.addLine("Minimum probability greater than zero: " + boost::lexical_cast<std::string>(minProbability));
    mPrintHelper.addLine("Recognition threshold: " + boost::lexical_cast<std::string>(recognitionThreshold));
    mPrintHelper.addLine("Found " + boost::lexical_cast<std::string>(validTestSets.size()) + " valid and " + boost::lexical_cast<std::string>(invalidTestSets.size()) + " invalid test sets.");
    mPrintHelper.addLine(boost::lexical_cast<std::string>(zeroSets) + " test sets have probability 0.");
    mPrintHelper.printAsHeader();

    mEvaluator->setRecognitionThreshold(recognitionThreshold);  // set actual recognition threshold again

    mValidTestSets = validTestSets;
    mInvalidTestSets = invalidTestSets;
}

void TestSetGenerator::setPoseOfObjectRelativeToReference(ISM::ObjectPtr pObject, ISM::ObjectPtr pReference)
{
    ISM::PosePtr newPose;
    pObject->pose->convertPoseIntoFrame(pReference->pose, newPose);
    pObject->pose = newPose;
}

std::vector<boost::shared_ptr<TestSet>> TestSetGenerator::loadTestSetsFromFile(const std::string& pFilename)
{
    ROS_INFO_STREAM("Loading test sets from file " << pFilename);
    // compare ISM CombinatorialTrainer
    try
    {
        std::vector<boost::shared_ptr<TestSet>> testSets;

        ISM::TableHelperPtr localTableHelper(new ISM::TableHelper(pFilename));
        std::vector<std::string> patternNames = localTableHelper->getRecordedPatternNames();
        if (std::find(patternNames.begin(), patternNames.end(), mSceneId) == patternNames.end())
            ROS_INFO_STREAM("In TestSetGenerator::loadTestSetsFromFile(" << pFilename + "): scene id " << mSceneId << " is not a valid pattern in the database.");
        else {
            std::vector<ISM::ObjectSetPtr> loadedTestSets;
            loadedTestSets = localTableHelper->getRecordedPattern(mSceneId)->objectSets;

            for (ISM::ObjectSetPtr loadedTestSet: loadedTestSets)
            {
                boost::shared_ptr<TestSet> testSet(new TestSet());
                for (ISM::ObjectPtr object: loadedTestSet->objects)
                    testSet->mObjectSet->insert(object);
                testSets.push_back(testSet);
            }
        }
        ROS_INFO_STREAM("Loaded " << testSets.size() << " test sets.");

        return testSets;
    }
    catch (soci::soci_error& e)
    {
        throw std::runtime_error("In TestSetGenerator::loadTestSetsFromFile(): soci error while trying to write to database file.\nProbably the file path does not exist.");
    }
}

void TestSetGenerator::writeTestSetsToFile(const std::string& pFilename, const std::vector<boost::shared_ptr<TestSet>>& pTestSets)
{
    ROS_INFO_STREAM("Writing test sets to file " << pFilename);
    if (!pTestSets.empty())
    {
        try
        {
            ISM::TableHelperPtr localTableHelper(new ISM::TableHelper(pFilename));
            localTableHelper->createTablesIfNecessary();
            localTableHelper->insertRecordedPattern(mSceneId);
            for (boost::shared_ptr<TestSet> testSet: pTestSets)
                localTableHelper->insertRecordedObjectSet(testSet->mObjectSet, mSceneId);
        }
        catch (soci::soci_error& e)
        {
            throw std::runtime_error("In TestSetGenerator::writeTestSetsToFile(): soci error while trying to write to database file " + pFilename
                                 + "\nProbably the file path does not exist.");
        }
    }
    ROS_INFO_STREAM("Wrote " << pTestSets.size() << " test sets.");
}

std::vector<boost::shared_ptr<TestSet>> TestSetGenerator::simulateOcclusion(std::vector<boost::shared_ptr<TestSet>> pCompleteTestSets)
{
    if (mObjectMissingInTestSetProbability == 0)
        return pCompleteTestSets;

    ROS_INFO_STREAM("Simulating occlusion by removing random object observations from test sets.");

    unsigned int removed = 0;

    std::random_device rd;
    boost::mt19937  eng;    // Mersenne Twister
    eng.seed(rd());
    boost::uniform_int<> dist(0,RAND_MAX);  // Normal Distribution
    boost::variate_generator<boost::mt19937,boost::uniform_int<>>  gen(eng,dist);  // Variate generator

    if (mObjectMissingInTestSetProbability < 0 ||  mObjectMissingInTestSetProbability > 1)
        throw std::runtime_error("parameter object_missing_in_test_set_probability should be in interval [0,1].");

    for (boost::shared_ptr<TestSet> testSet: pCompleteTestSets)
    {
        // The first object in each test set is treated as the reference object, which is not allowed to be occluded, so i starts at 1.
        // This also prevents empty test sets.
        unsigned int i = 1;
        unsigned int occurs;

        while (i < testSet->mObjectSet->objects.size())
        {
            occurs = gen() % (int) ((double) 1.0 / mObjectMissingInTestSetProbability);   // only with a certain probability: object i occurs

            if (occurs != 0)             // if object i does occur, move to next one
                i++;
            else
            {
                testSet->mObjectSet->objects.erase(testSet->mObjectSet->objects.begin() + i);    // otherwise erase it from test set.
                removed++;
            }
        }
    }
    ROS_INFO_STREAM("Removed " << removed << " object observations.");
    return pCompleteTestSets;
}

}
