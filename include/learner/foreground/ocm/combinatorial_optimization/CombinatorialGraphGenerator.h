/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

#include <trainer/TreeNode.h>
#include <trainer/AbstractTrainer.h>

#include <trainer/source/AbstractSource.h>
#include <trainer/source/PbdSceneGraphSource.h>

#include <trainer/generator/AbstractGraphGenerator.h>

// Local includes
#include "learner/foreground/ocm/combinatorial_optimization/CombinatorialOptimizer.h"

namespace ProbabilisticSceneRecognition {

/**
 * Generates a tree with references representing a topology selected with combinatorial optimization.
 */
class CombinatorialGraphGenerator: public SceneModel::AbstractGraphGenerator
{
public:
    /**
     * Constructor.
     * @param pLearners		the learners used to generate the models from the topologies that are considered during optimization
     * @param pObjectTypes	the possible object types in the model
     */
    CombinatorialGraphGenerator(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                                std::vector<std::string> pObjectTypes);

    /**
     * Destructor.
     */
    ~CombinatorialGraphGenerator();

    /**
     * Builds the tree.
     *
     * @param pObjectSets The list of trajectories to build the tree from.
     * @param pRoot The root node of the tree.
     */
    void buildTree(SceneModel::ObjectSetList pObjectSets, boost::shared_ptr<SceneModel::TreeNode>& pRoot);

    /**
     * Builds the tree, forces the object with the given type as root node and appends a new node to what ever node it is appropriate
     *
     * @param pTrajectories The list of trajectories to build the tree from.
     * @param pRoot The root node of the tree.
     * @param pType Type of the object that should be forced as root node.
     */
    void buildTree(SceneModel::ObjectSetList pTrajectories, boost::shared_ptr<SceneModel::TreeNode>& pRoot, std::string pType);

private:

    /**
     * The learners used to generate the models from the topologies that are considered during optimization
     */
    std::vector<boost::shared_ptr<SceneObjectLearner>> mLearners;

    /**
     * The possible object types in the model
     */
    std::vector<std::string> mObjectTypes;
};

}
