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

#include <topology_creator/Topology.h>
#include <ISM/combinatorial_trainer/Topology.hpp>
#include <ISM/combinatorial_optimization/CostFunction.hpp>

namespace ProbabilisticSceneRecognition
{

/**
 * Transforms topologies from PSM to ISM and vice versa.
 */
class TopologyAdapter
{
private:
    /**
     * Counter used for assigning a unique index to each new PSM Topology.
     */
    unsigned int mPSMTopologyIndex;
    /**
     * Counter used for assigning a unique index to each new ISM Topology.
     */
    unsigned int mISMTopologyIndex;
    /**
     * All possible relations mapped to unique indices.
     */
    std::vector<SceneModel::Relation> mRelations;
    /**
     * Id of the scene that is currently being recognized.
     */
    std::string mSceneId;

public:
    /**
     * Returns the cost of a topology which has already been set before.
     */
    class CostFunction: public ISM::CostFunction<ISM::TopologyPtr> {
    public:
        /**
         * return the cost of the instance.
         * @param instance  to return the cost of
         * @return          the cost of the instance
         */
        double calculateCost(ISM::TopologyPtr instance);
    };

    /**
     * Constructor.
     */
    TopologyAdapter(std::vector<std::string> pObjectTypes, std::string pSceneId);

    /**
     * Destructor.
     */
    ~TopologyAdapter();

    /**
     * Transforms a PSM SceneModel::Topology into an ISM::Topology.
     * @param pPsmTopology  PSM SceneModel::Topology to transform.
     * @return ISM::Topology.
     */
    ISM::TopologyPtr psmToIsm(boost::shared_ptr<SceneModel::Topology> pPsmTopology);

    /**
     * Transforms an ISM::Topology into a PSM SceneModel::Topology.
     * @param pIsmTopology  ISM::Topology to transform.
     * @return PSM Topology.
     */
    boost::shared_ptr<SceneModel::Topology> ismToPsm(ISM::TopologyPtr pIsmTopology);
};

}
