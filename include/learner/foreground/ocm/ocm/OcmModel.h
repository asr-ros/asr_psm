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

// Local includes
#include "learner/foreground/ocm/ocm/OcmTree.h"

#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

#include "helper/ProbabilityTable.h"
#include "helper/MappedProbabilityTable.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class represents the OCM model. It contains data structures for the parameters of shape, appearance, hypothesis and occlusion.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmModel {
  public:
    
    /**
     * Constructor.
     * 
     * @param pRoot Root node of the tree that represents the learned relationships.
     */
    OcmModel(const boost::shared_ptr<SceneModel::TreeNode> pRoot);
    
    /**
     * Destructor.
     */
    ~OcmModel();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Saves the tree to XML beginning with the root node.
     * 
     * @param pWorkspaceVolume The volume of the workspace the scene takes place in qubic meters.
     * @param pPt Datastructure for handling XML operations.
     */
    void save(double pWorkspaceVolume, boost::property_tree::ptree& pPt);
    
    /**
     * Saves the shape information for the given node to XML.
     * 
     * @param pWorkspaceVolume The volume of the workspace the scene takes place in qubic meters.
     * @param pPt Data structure for handling XML operations.
     */
    void saveShape(double pWorkspaceVolume, boost::property_tree::ptree& pPt);
    
    /**
     * Saves the appearance information for the given node to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void saveAppearance(boost::property_tree::ptree& pPt);
    
    /**
     * Saves the occlusion information for the given node to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void saveOcclusion(boost::property_tree::ptree& pPt);
    
    /**
     * Returns the number of slots.
     * 
     * @return The number of slots.
     */
    unsigned int getNumberOfSlots();
    
  public:
    
    /**
     * The root node of the OCM tree data structure for modelling the hierarchical relationship of the model slots.
     */
    boost::shared_ptr<OcmTree> mRoot;
    
    /**
     * A probability table for the appearance that could be adressey with cleartext object names.
     */
    boost::shared_ptr<MappedProbabilityTable> mAppearanceTable;
    
    /**
     * A probability table for the hypothesis that could be adressey with cleartext object names.
     */
    boost::shared_ptr<MappedProbabilityTable> mHypothesisTable;
    
    /**
     * A probability table representing the occlusion probabilities.
     */
    boost::shared_ptr<ProbabilityTable> mOcclusionTable;
  };
}
