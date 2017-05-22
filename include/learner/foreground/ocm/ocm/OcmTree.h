/**

Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <trainer/TreeNode.h>
#include <trainer/source/ObjectSet.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>
#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

#include "helper/MappedProbabilityTable.h"
#include "helper/ProbabilityTable.h"

#include "helper/MathHelper.h"

#include <ISM/common_type/ObjectSet.hpp>
#include <ISM/common_type/Object.hpp>
#include <ISM/common_type/Pose.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This tree represents the structure of the OCM. Every note equals a slot in the model and contains a set of parameters. These are filled by term learners. The tree provides a persistence mechanism.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmTree {
  public:
    
    /**
     * Constructor.
     * 
     * @param pRoot Root node of the tree that represents the learned relationships.
     */
    OcmTree(const boost::shared_ptr<SceneModel::TreeNode> pRoot);
    
    /**
     * Destructor.
     */
    ~OcmTree();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     * @param mParent The parent ocm node.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior, 
			      OcmTree* pParent);
    
    /**
     * Saves the shape information for the given node to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void saveShape(boost::property_tree::ptree& pPt);
    
    /**
     * Returns the number of nodes in the tree.
     * 
     * @return The number of nodes in the tree.
     */
    unsigned int getNumberOfNodes();
    
  public:
    
    /**
     * The type of the object that is represented by this node.
     */
    std::string mType;
    
    /**
     * An object trajectory containing all observations of a single object over time.
     */
    boost::shared_ptr<ISM::ObjectSet> mObjectSet;
        
    /**
     * A list of child nodes.
     */
    std::vector<boost::shared_ptr<OcmTree> > mChildren;
    
    /**
     * The gaussian mixture model describing the shape of the node.
     * It states possible locations for the object assicated with this node.
     */
    GaussianMixtureModel mGaussianMixtureModelPosition;
    
    /**
     * The gaussian mixture model describing the shape of the node.
     * It states possible orientations for the object assicated with this node.
     */
    GaussianMixtureModel mGaussianMixtureModelOrientation;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer;

    bool mIsReference;

  private:

    unsigned int mReferenceToID;
  };
}
