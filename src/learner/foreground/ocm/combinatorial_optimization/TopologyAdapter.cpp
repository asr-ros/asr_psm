/**

Copyright (c) 2017, Braun Kai, Gaßner Nikolai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/TopologyAdapter.h"

namespace ProbabilisticSceneRecognition
{

double TopologyAdapter::CostFunction::calculateCost(ISM::TopologyPtr instance)
{
    return instance->cost;
}

TopologyAdapter::TopologyAdapter(std::vector<std::string> pObjectTypes, std::string pSceneId):  mPSMTopologyIndex(0), mISMTopologyIndex(0), mSceneId(pSceneId)
{
    // Generate all possible relations:
    for (unsigned int i = 0; i < pObjectTypes.size() - 1; i++)
        for (unsigned int j = i + 1; j < pObjectTypes.size(); j++)
            mRelations.push_back(SceneModel::Relation(pObjectTypes[i], pObjectTypes[j]));
}

TopologyAdapter::~TopologyAdapter()
{ }

ISM::TopologyPtr TopologyAdapter::psmToIsm(boost::shared_ptr<SceneModel::Topology> pPsmTopology)
{
    ISM::TopologyPtr ismTopology(new ISM::Topology());

    ismTopology->index = mISMTopologyIndex;
    mISMTopologyIndex++;

    ismTopology->cost = pPsmTopology->getCost();

    // It is unknown whether an ism tree created for this topology would be valid.
    // always assumed to be invalid.
    ismTopology->isValid = false;

    ismTopology->identifier = pPsmTopology->mIdentifier;

    ismTopology->evaluationResult.falsePositives = pPsmTopology->getFalsePositives();
    ismTopology->evaluationResult.falseNegatives = pPsmTopology->getFalseNegatives();
    ismTopology->evaluationResult.averageRecognitionRuntime = pPsmTopology->getAverageRecognitionRuntime();

    // go over the relations and enumerate them according to their index in the list of all relations
    for (unsigned int relationIndex = 0; relationIndex < mRelations.size(); relationIndex++)
    {
        std::string objectA = mRelations[relationIndex].getObjectTypeA();
        std::string objectB = mRelations[relationIndex].getObjectTypeB();
        for (boost::shared_ptr<SceneModel::Relation> relation: pPsmTopology->mRelations)
        {
            if (relation->containsObject(objectA) && relation->containsObject(objectB))
            {
                ISM::TrackPtr trackA(new ISM::Track(objectA));
                ISM::TrackPtr trackB(new ISM::Track(objectB));
                ISM::ObjectRelationPtr ismRelation(new ISM::ObjectRelation(trackA, trackB, mSceneId));
                ismTopology->objectRelations[relationIndex] = ismRelation;
            }
        }
    }

    return ismTopology;
}

boost::shared_ptr<SceneModel::Topology> TopologyAdapter::ismToPsm(ISM::TopologyPtr pIsmTopology)
{
    boost::shared_ptr<SceneModel::Topology> psmTopology(new SceneModel::Topology());

    // Most likely not used in PSM optimization
    psmTopology->mUsedInOptimization = false;

    //mTree does not get assigned any value. Could make a tree with TopologyManager, but seems unneccessary here.

    psmTopology->setCost(pIsmTopology->cost);

    psmTopology->setEvaluationResult(pIsmTopology->evaluationResult.averageRecognitionRuntime,
                                     pIsmTopology->evaluationResult.falsePositives,
                                     pIsmTopology->evaluationResult.falseNegatives);

    // Note that this identifier may not be compatible with the ones assigned in PSM, for example in TopologyManager.
    psmTopology->mIdentifier = pIsmTopology->identifier;

    for (std::pair<unsigned int, ISM::ObjectRelationPtr> relation: pIsmTopology->objectRelations)
    {
        std::string objectTypeA = relation.second->getObjectTypeA();   // Note that ISM distinguishes between an object's type and its id, whereas in PSM, there can only be one object of each type anyways.
        std::string objectTypeB = relation.second->getObjectTypeB();
        boost::shared_ptr<SceneModel::Relation> psmRelation(new SceneModel::Relation(objectTypeA, objectTypeB));
        psmTopology->mRelations.push_back(psmRelation);
    }

    return psmTopology;
}

}

