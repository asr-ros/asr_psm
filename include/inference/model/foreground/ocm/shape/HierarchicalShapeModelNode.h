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

// Package includes
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrSceneGraph.h>

#include <Pose.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>
#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/ocm/shape/GaussianMixtureDistribution.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A node in object constellation model (OCM). It is responsible for maintaining the gaussian mixture distributions for this node. Note that all distributions are modelled in the coordinate system of their parent nodes and the transformation of evidence into this coordinate system is also handled here.
   */
  class HierarchicalShapeModelNode {
  public:

    /**
    * Constructor.
    * 
    * @param pPt Data structure for performing XML operations.
    */
    HierarchicalShapeModelNode(boost::property_tree::ptree& pPt);
    
    /**
    * Destructor.
    */
    ~HierarchicalShapeModelNode();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Integrate the learning data in form of a AsrSceneGraph into the model.
     *
     * @param pParent The AsrNode that acts as parent for this node.
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void handleSceneGraph(asr_msgs::AsrNode& pParent, const boost::shared_ptr<const asr_msgs::AsrSceneGraph>& pSceneGraph);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Sets the absolute pose of the parent node.
     * 
     * @param pPose The absolute pose.
     */
    void setAbsoluteParentPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Calculates the probability for a hypothesis with the given assignments.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pAssignments Assignments of parts to slots.
     * @param pSlotId The id of the slot this node is associated with.
     * @param pCut True, if a zero-object was assigned to a direct parent node.
     * @return Probability as determined by this subtree the hierarchical shape model.
     */
    double calculateProbabilityForHypothesis(std::vector<asr_msgs::AsrObject> pEvidenceList, std::vector<unsigned int> pAssignments, unsigned int& pSlotId, bool pCut);
    
    /**
     * Update the visualizers based on the evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void visualize(std::vector<asr_msgs::AsrObject> pEvidenceList);
    
    /**
     * Return the number of nodes in the OCM.
     */
    unsigned int getNumberOfNodes();
    
  private:
    
    /**
     * The type of the object that is represented by this node.
     */
    std::string mSceneObject;
    
    /**
     * The pose of this object in absolute coordinates. This variable is used during inference and reused in the visualization.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mAbsolutePose;
    
    /**
     * The pose of this object in coordinates relative to the parent frame. This variable is used during inference and reused in the visualization.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mRelativePose;
    
    /**
     * The pose of the parent object in absolute coordinates.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mAbsoluteParentPose;
    
    /**
     * The gaussian mixture distribution representing the position part of the shape.
     */
    boost::shared_ptr<GaussianMixtureDistribution> mGaussianMixtureDistributionPosition;
    
    /**
     * The gaussian mixture distribution representing the orientation part of the shape.
     */
    boost::shared_ptr<GaussianMixtureDistribution> mGaussianMixtureDistributionOrientation;
    
    /**
     * A list of all samples that were used to learn the gaussian mixture distributions.
     */
    std::vector<Eigen::Vector3d> mRawData;
    
    /**
     * The chrildren of this node.
     */
    std::vector<HierarchicalShapeModelNode> mChildren;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer;

    bool mIsReference;
    unsigned int mReferenceTo;
  };
}
