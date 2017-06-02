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

TestSetGenerator::TestSetGenerator(boost::shared_ptr<AbstractEvaluator> pEvaluator, const std::vector<std::string>& pObjectTypes, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology,
                 double pObjectMissingInTestSetProbability):
    mEvaluator(pEvaluator), mTypes(pObjectTypes), mFullyMeshedTopology(pFullyMeshedTopology), mObjectMissingInTestSetProbability(pObjectMissingInTestSetProbability),
    mPrintHelper('+')
{ }

TestSetGenerator::~TestSetGenerator()
{ }

void TestSetGenerator::generateTestSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList,
                                        unsigned int pTestSetCount)
{
    ros::NodeHandle nodeHandle("~");
    std::string validTestSetDbFilename, invalidTestSetDbFilename, writeValidTestSetsFilename, writeInvalidTestSetsFilename;
    // Try to get the valid test set source database filename:
    if(!nodeHandle.getParam("valid_test_set_db_filename", validTestSetDbFilename))
        throw std::runtime_error("Please specifiy parameter valid_test_set_db_filename when starting this node.");
    // Try to get the invalid test set source database filename:
    if(!nodeHandle.getParam("invalid_test_set_db_filename", invalidTestSetDbFilename))
        throw std::runtime_error("Please specifiy parameter invalid_test_set_db_filename when starting this node.");
    // Try to get the valid test set target database filename:
    if(!nodeHandle.getParam("write_valid_test_sets_filename", writeValidTestSetsFilename))
        throw std::runtime_error("Please specifiy parameter write_valid_test_sets_filename when starting this node.");
    // Try to get the invalid test set target database filename:
    if(!nodeHandle.getParam("write_invalid_test_sets_filename", writeInvalidTestSetsFilename))
        throw std::runtime_error("Please specifiy parameter write_invalid_test_sets_filename when starting this node.");
    mSceneId = pExamplesList[0]->mIdentifier;

    // if the file names are empty: generate new test sets.
    if (validTestSetDbFilename == "" || invalidTestSetDbFilename == "")
    {
        validateSets(generateRandomSets(pExamplesList, pTestSetCount));
        if (writeValidTestSetsFilename != "")
            writeTestSetsToFile(writeValidTestSetsFilename, mValidTestSets);
        if (writeInvalidTestSetsFilename != "")
            writeTestSetsToFile(writeInvalidTestSetsFilename, mInvalidTestSets);
    }
    else // get the test sets from the databases. Might be more than pTestSetCount.
    {
        mValidTestSets = loadTestSetsFromFile(validTestSetDbFilename);
        mInvalidTestSets = loadTestSetsFromFile(invalidTestSetDbFilename);
    }

    mEvaluator->setValidTestSets(mValidTestSets);    // update with newfound valid test sets
    mEvaluator->setInvalidTestSets(mInvalidTestSets);    // initialize properly
}

std::vector<boost::shared_ptr<TestSet>> TestSetGenerator::generateRandomSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount)
{
    mPrintHelper.printAsHeader("Generating " + boost::lexical_cast<std::string>(pTestSetCount) + " random test sets.");

    std::random_device rd;
    boost::mt19937  eng;    // Mersenne Twister
    eng.seed(rd());
    boost::uniform_int<> dist(0,RAND_MAX);  // Normal Distribution
    boost::variate_generator<boost::mt19937,boost::uniform_int<>>  gen(eng,dist);  // Variate generator

    ROS_INFO_STREAM("Found " << mTypes.size() << " object types.");

    std::vector<boost::shared_ptr<TestSet>> testSets;

    for (unsigned int i = 0; i < pTestSetCount; i++)
    {
        boost::shared_ptr<TestSet> testSet(new TestSet());


        unsigned int randomTimestep = gen() % pExamplesList.size();
        unsigned int randomObjectIndex = gen() % pExamplesList[randomTimestep]->objects.size();
        ISM::ObjectPtr referenceObject = pExamplesList[randomTimestep]->objects[randomObjectIndex];
        //setPoseOfObjectRelativeToReference(referenceObject, referenceObject);
        testSet->mObjectSet->insert(referenceObject);

        for (std::string objectType: mTypes)
        {
            if (objectType == referenceObject->type)
                continue;
            else
            {
                unsigned int occurs;
                if (mObjectMissingInTestSetProbability > 1)
                    throw std::runtime_error("parameter object_missing_in_test_set_probability should not be larger than 1.");
                if (mObjectMissingInTestSetProbability > 0)
                    occurs = gen() % (int) ((double) 1.0 / mObjectMissingInTestSetProbability);   // only with a certain probability: object ccurs
                else occurs = 1;

                if (occurs != 0)             // object does occur
                {
                    ISM::ObjectPtr object;

                    unsigned int counter = 0;
                    while (!object && counter < pExamplesList.size())   // does not iterate over all timesteps; using example list size to scale
                    {
                        unsigned int newRandomTimestep = gen() % pExamplesList.size();
                        for (ISM::ObjectPtr currentObject: pExamplesList[newRandomTimestep]->objects)
                        {
                            if (currentObject->type == objectType)
                            {
                                object = currentObject;
                                break;
                            }
                        }
                        counter++;
                    }

                    if (!object)
                        throw std::runtime_error("In TestSetGenerator::generateTestSets(): Failed to find object of type " + objectType + " in evidence list.");
                    //setPoseOfObjectRelativeToReference(object, referenceObject);
                    testSet->mObjectSet->insert(object);
                }
            }
        }

        testSets.push_back(testSet);
    }

    return testSets;
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
    pTestSets = mEvaluator->getValidTestSets();
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
    /*
    std::vector<geometry_msgs::PoseWithCovariance> newPoses;
    for (geometry_msgs::PoseWithCovariance refPose: pReference.sampledPoses)
    {
        Eigen::Vector3d refPosePosition(refPose.pose.position.x, refPose.pose.position.y, refPose.pose.position.z);
        Eigen::Quaterniond refPoseOrientation(refPose.pose.orientation.w, refPose.pose.orientation.x, refPose.pose.orientation.y, refPose.pose.orientation.z);
        for (geometry_msgs::PoseWithCovariance objectPose: pObject.sampledPoses)
        {
            Eigen::Vector3d objectPosePosition(objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z);
            Eigen::Quaterniond objectPoseOrientation(objectPose.pose.orientation.w, objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z);
            // compare ISM::GeometryHelper
            if ((refPosePosition - objectPosePosition).norm() == 0) {
                //avoid special case
                objectPosePosition.x() = (objectPosePosition.x() + 0.0000001);
            }
            Eigen::Vector3d objToRefVector = refPosePosition - objectPosePosition;
            Eigen::Vector3d refToObjVector = objToRefVector * -1.0;
            Eigen::Quaternion<double> p = objectPoseOrientation;
            Eigen::Quaternion<double> r = refPoseOrientation;

            //rotate everything relative to ref pose
            Eigen::Quaternion<double> refToObjectPoseQuat = (r.inverse()) * p;

            geometry_msgs::PoseWithCovariance newPose;
            newPose.pose.position.x = refToObjVector.x();
            newPose.pose.position.y = refToObjVector.y();
            newPose.pose.position.z = refToObjVector.z();
            newPose.pose.orientation.w = refToObjectPoseQuat.w();
            newPose.pose.orientation.x = refToObjectPoseQuat.x();
            newPose.pose.orientation.y = refToObjectPoseQuat.y();
            newPose.pose.orientation.z = refToObjectPoseQuat.z();

            newPoses.push_back(newPose);
        }
    }

    pObject.sampledPoses = newPoses;*/
}

std::vector<boost::shared_ptr<TestSet>> TestSetGenerator::loadTestSetsFromFile(const std::string& pFilename)
{
    ROS_INFO_STREAM("Loading test sets from file " << pFilename);
    // compare ISM CombinatorialTrainer
    try
    {
        ISM::TableHelperPtr localTableHelper(new ISM::TableHelper(pFilename));
        std::vector<std::string> patternNames = localTableHelper->getRecordedPatternNames();
        if (std::find(patternNames.begin(), patternNames.end(), mSceneId) == patternNames.end())
            throw std::runtime_error("In TestSetGenerator::loadTestSetsFromFile(" + pFilename + "): scene id " + mSceneId + " is not a valid pattern in the database.");

        std::vector<ISM::ObjectSetPtr> loadedTestSets;
        loadedTestSets = localTableHelper->getRecordedPattern(mSceneId)->objectSets;

        std::vector<boost::shared_ptr<TestSet>> testSets;
        for (ISM::ObjectSetPtr loadedTestSet: loadedTestSets)
        {
            boost::shared_ptr<TestSet> testSet(new TestSet());
            for (ISM::ObjectPtr object: loadedTestSet->objects)
                testSet->mObjectSet->insert(object);
            testSets.push_back(testSet);
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
            localTableHelper->dropTables();
            localTableHelper->createTablesIfNecessary();
            localTableHelper->insertRecordedPattern(mSceneId);
            std::vector<ISM::ObjectSetPtr> objectSets;
            for (boost::shared_ptr<TestSet> testSet: pTestSets)
            {
                ISM::ObjectSetPtr objectSet(new ISM::ObjectSet());
                for (ISM::ObjectPtr object: testSet->mObjectSet->objects)
                    objectSet->insert(object);
                objectSets.push_back(objectSet);
            }

            for (unsigned int i = 0; i < objectSets.size(); ++i)
                localTableHelper->insertRecordedObjectSet(objectSets[i], mSceneId);

        }
        catch (soci::soci_error& e)
        {
            throw std::runtime_error("In TestSetGenerator::writeTestSetsToFile(): soci error while trying to write to database file " + pFilename
                                 + "\nProbably the file path does not exist.");
        }
    }
    ROS_INFO_STREAM("Wrote " << pTestSets.size() << " test sets.");
}

}
