/**

Copyright (c) 2017, Gaßner Nikolai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "learner/foreground/ocm/combinatorial_optimization/CombinatorialGraphGenerator.h"
#include <ros/ros.h>

namespace ProbabilisticSceneRecognition {

CombinatorialGraphGenerator::CombinatorialGraphGenerator(std::vector<boost::shared_ptr<SceneObjectLearner>> pLearners,
                                                         std::vector<std::string> pObjectTypes):
    SceneModel::AbstractGraphGenerator(), mLearners(pLearners), mObjectTypes(pObjectTypes)
    {}

CombinatorialGraphGenerator::~CombinatorialGraphGenerator() {}

void CombinatorialGraphGenerator::buildTree(SceneModel::ObjectSetList pObjectSets, boost::shared_ptr<SceneModel::TreeNode>& pRoot)
{
    buildTree(pObjectSets, pRoot, mObjectTypes[0]);    // Builds tree with first object found as node.
}

void CombinatorialGraphGenerator::buildTree(SceneModel::ObjectSetList pTrajectories, boost::shared_ptr<SceneModel::TreeNode>& pRoot, std::string pType)
{
    boost::shared_ptr<CombinatorialOptimizer>
            combinatorialOptimizer(new CombinatorialOptimizer(mLearners, mObjectTypes, pTrajectories.mObjectSets));
    // set the optimized topology as the one to be transformed into the tree to be used for the final result
    boost::shared_ptr<SceneModel::Topology> optimizedTopology = combinatorialOptimizer->runOptimization();

    if (!optimizedTopology)
        throw std::runtime_error("no valid topology was found.");
    if (!(optimizedTopology->mTree))
        throw std::runtime_error("the topology selected through combinatorial optimization has no tree associated with it.");

    boost::shared_ptr<SceneModel::TreeNode> optimizedTree = optimizedTopology->mTree;
    while(optimizedTree->mParent)   // after rearrangement in learning, the pointer points to an inner node (through parent pointers, the whole tree is still intact)
        optimizedTree = optimizedTree->mParent;

    // Rearrange for given node type:
    optimizedTree->setNewRootNodeByType(pType);

    // set the resulting tree
    pRoot = optimizedTree;
}

}
