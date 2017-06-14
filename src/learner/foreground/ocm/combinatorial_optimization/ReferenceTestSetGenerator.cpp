/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/ReferenceTestSetGenerator.h"

namespace ProbabilisticSceneRecognition {

ReferenceTestSetGenerator::ReferenceTestSetGenerator(boost::shared_ptr<AbstractTopologyEvaluator> pEvaluator, boost::shared_ptr<SceneModel::Topology> pFullyMeshedTopology, const std::vector<std::string>& pObjectTypes):
    TestSetGenerator(pEvaluator, pFullyMeshedTopology, pObjectTypes)
{ }

ReferenceTestSetGenerator::~ReferenceTestSetGenerator()
{ }

std::vector<boost::shared_ptr<TestSet>> ReferenceTestSetGenerator::generateRandomSets(std::vector<boost::shared_ptr<ISM::ObjectSet>> pExamplesList, unsigned int pTestSetCount)
{
    TestSetGenerator::mPrintHelper.printAsHeader("Generating " + boost::lexical_cast<std::string>(pTestSetCount) + " random test sets.");

    std::random_device rd;
    boost::mt19937  eng;    // Mersenne Twister
    eng.seed(rd());
    boost::uniform_int<> dist(0,RAND_MAX);  // Normal Distribution
    boost::variate_generator<boost::mt19937,boost::uniform_int<>>  gen(eng,dist);  // Variate generator

    ROS_INFO_STREAM("Found " << TestSetGenerator::mTypes.size() << " object types.");

    std::vector<boost::shared_ptr<TestSet>> testSets(pTestSetCount);

    for (unsigned int i = 0; i < pTestSetCount; i++)
    {
        // pick random reference:
        unsigned int randomTimestep = gen() % pExamplesList.size();
        unsigned int randomObjectIndex = gen() % pExamplesList[randomTimestep]->objects.size();
        ISM::ObjectPtr referenceObject(new ISM::Object(*pExamplesList[randomTimestep]->objects[randomObjectIndex])); // deep copy the object

        testSets[i].reset(new TestSet());
        testSets[i]->mObjectSet->insert(referenceObject);

        for (std::string objectType: mTypes)
        {
            if (objectType != referenceObject->type)
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
                            object = ISM::ObjectPtr(new ISM::Object(*currentObject)); // deep copy the object.
                            break;
                        }
                    }
                    counter++;
                }

                if (!object)
                    throw std::runtime_error("In TestSetGenerator::generateTestSets(): Failed to find object of type " + objectType + " in evidence list.");
                setPoseOfObjectRelativeToReference(object, referenceObject);
                testSets[i]->mObjectSet->insert(object);
            }
        }
    }

    return testSets;
}

}
