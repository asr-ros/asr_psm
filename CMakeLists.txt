cmake_minimum_required(VERSION 2.4.6)
project(asr_psm)

# Set compiler flags to enable the C++11 standard.
add_definitions("-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rosbag
  roslib
  asr_msgs
  std_msgs
  tf
  visualization_msgs

  asr_psm_visualizations
  asr_relation_graph_generator
  asr_resources_for_psm
  asr_lib_ism
)

find_package(cmake_modules)
# include Eigen library
find_package(Eigen REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  iostreams
  system
  filesystem
)


add_library(psm_inference  src/inference.cpp
 			    src/inference/SceneInferenceEngine.cpp

			    src/inference/model/SceneModelDescription.cpp src/inference/model/SceneDescription.cpp src/inference/model/SceneContent.cpp src/inference/model/InferenceAlgorithm.cpp src/inference/model/SceneIdentifier.cpp src/inference/model/ObjectEvidence.cpp src/inference/model/KalmanFilter.cpp
			    
			    src/inference/model/background/BackgroundSceneContent.cpp
			    src/inference/model/background/BackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/PowerSetBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/SummarizedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MultipliedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MaximumBackgroundInferenceAlgorithm.cpp
			    
			    src/inference/model/foreground/ForegroundSceneContent.cpp
			    src/inference/model/foreground/ForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/PowerSetForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SummarizedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MultipliedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MaximumForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SceneObjectDescription.cpp src/inference/model/foreground/SceneObjectContent.cpp

			    src/inference/model/foreground/ocm/OcmSceneObjectContent.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModel.cpp src/inference/model/foreground/ocm/shape/ShapeTermEvaluator.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModelNode.cpp src/inference/model/foreground/ocm/shape/GaussianMixtureDistribution.cpp src/inference/model/foreground/ocm/shape/GaussianKernel.cpp
			    src/inference/model/foreground/ocm/appearance/AppearanceTermEvaluator.cpp src/inference/model/foreground/ocm/occlusion/OcclusionTermEvaluator.cpp src/inference/model/foreground/ocm/TermEvaluator.cpp
			    
			    src/helper/MappedProbabilityTable.cpp src/helper/ProbabilityTable.cpp src/helper/MathHelper.cpp src/helper/SerializationHelper.cpp src/helper/ObjectTransformation.cpp
			    
			    src/inference/model/foreground/ocm/shape/conditional_probability/MinimumConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/MultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/RootOfMultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/AverageConditionalProbability.cpp
			    )
			    
target_link_libraries(psm_inference  
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES psm_inference
  CATKIN_DEPENDS rospy roscpp rosbag roslib asr_msgs std_msgs tf visualization_msgs asr_psm_visualizations asr_relation_graph_generator asr_resources_for_psm
  DEPENDS Eigen Boost
)


include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
)




############################################################
# ROS Nodes that are using the Probabilistic Scene Model.
############################################################

add_executable(learner src/learner.cpp
			     src/learner/SceneLearningEngine.cpp
			     src/learner/SceneModelLearner.cpp src/learner/SceneLearner.cpp
			     
			     src/learner/background/BackgroundSceneLearner.cpp
			     src/learner/foreground/ForegroundSceneLearner.cpp src/learner/foreground/ocm/OcmForegroundSceneLearner.cpp
			     src/learner/foreground/ocm/SceneObjectLearner.cpp src/learner/foreground/ocm/ocm/OcmSceneObjectLearner.cpp src/learner/foreground/ocm/ocm/OcmModel.cpp src/learner/foreground/ocm/ocm/OcmTree.cpp src/learner/foreground/ocm/ocm/TermLearner.cpp
			     
			     src/learner/foreground/ocm/ocm/shape/ShapeTermLearner.cpp src/learner/foreground/ocm/ocm/shape/GaussianKernel.cpp src/learner/foreground/ocm/ocm/shape/GaussianMixtureModel.cpp
			     src/learner/foreground/ocm/ocm/shape/GMMParameterEstimator.cpp
			     
			     src/learner/foreground/ocm/ocm/appearance/AppearanceTermLearner.cpp
			     
			     src/learner/foreground/ocm/ocm/occlusion/OcclusionTermLearner.cpp
			     
			     src/helper/MappedProbabilityTable.cpp src/helper/ProbabilityTable.cpp src/helper/MathHelper.cpp src/helper/SerializationHelper.cpp src/helper/ObjectTransformation.cpp
			     
			     src/learner/foreground/ocm/combinatorial_optimization/WeightedSum.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/AbstractEvaluator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/Evaluator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TopologyAdapter.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TopologyManager.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialTrainer.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialGraphGenerator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialOptimizer.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSetSelection.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSet.cpp)
target_link_libraries(learner
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  psm_inference
  ism
)

add_executable(learner_batch src/learner_batch.cpp
			     src/learner/SceneLearningEngine.cpp
			     src/learner/SceneModelLearner.cpp src/learner/SceneLearner.cpp
			     
			     src/learner/background/BackgroundSceneLearner.cpp
			     src/learner/foreground/ForegroundSceneLearner.cpp src/learner/foreground/ocm/OcmForegroundSceneLearner.cpp
			     src/learner/foreground/ocm/SceneObjectLearner.cpp src/learner/foreground/ocm/ocm/OcmSceneObjectLearner.cpp src/learner/foreground/ocm/ocm/OcmModel.cpp src/learner/foreground/ocm/ocm/OcmTree.cpp src/learner/foreground/ocm/ocm/TermLearner.cpp
			     
			     src/learner/foreground/ocm/ocm/shape/ShapeTermLearner.cpp src/learner/foreground/ocm/ocm/shape/GaussianKernel.cpp src/learner/foreground/ocm/ocm/shape/GaussianMixtureModel.cpp
			     src/learner/foreground/ocm/ocm/shape/GMMParameterEstimator.cpp
			     
			     src/learner/foreground/ocm/ocm/appearance/AppearanceTermLearner.cpp
			     
			     src/learner/foreground/ocm/ocm/occlusion/OcclusionTermLearner.cpp
			     
			     src/helper/MappedProbabilityTable.cpp src/helper/ProbabilityTable.cpp src/helper/MathHelper.cpp src/helper/SerializationHelper.cpp src/helper/ObjectTransformation.cpp
			     
			     src/learner/foreground/ocm/combinatorial_optimization/WeightedSum.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/AbstractEvaluator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/Evaluator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSetGenerator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TopologyAdapter.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TopologyManager.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialTrainer.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialGraphGenerator.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/CombinatorialOptimizer.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSetSelection.cpp
			     src/learner/foreground/ocm/combinatorial_optimization/TestSet.cpp)
target_link_libraries(learner_batch
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  psm_inference
  ism
)

add_executable(inference src/inference.cpp
 			    src/inference/SceneInferenceEngine.cpp

			    src/inference/model/SceneModelDescription.cpp src/inference/model/SceneDescription.cpp src/inference/model/SceneContent.cpp src/inference/model/InferenceAlgorithm.cpp src/inference/model/SceneIdentifier.cpp src/inference/model/ObjectEvidence.cpp src/inference/model/KalmanFilter.cpp
			    
			    src/inference/model/background/BackgroundSceneContent.cpp
			    src/inference/model/background/BackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/PowerSetBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/SummarizedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MultipliedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MaximumBackgroundInferenceAlgorithm.cpp
			    
			    src/inference/model/foreground/ForegroundSceneContent.cpp
			    src/inference/model/foreground/ForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/PowerSetForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SummarizedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MultipliedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MaximumForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SceneObjectDescription.cpp src/inference/model/foreground/SceneObjectContent.cpp

			    src/inference/model/foreground/ocm/OcmSceneObjectContent.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModel.cpp src/inference/model/foreground/ocm/shape/ShapeTermEvaluator.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModelNode.cpp src/inference/model/foreground/ocm/shape/GaussianMixtureDistribution.cpp src/inference/model/foreground/ocm/shape/GaussianKernel.cpp
			    src/inference/model/foreground/ocm/appearance/AppearanceTermEvaluator.cpp src/inference/model/foreground/ocm/occlusion/OcclusionTermEvaluator.cpp src/inference/model/foreground/ocm/TermEvaluator.cpp
			    
			    src/helper/MappedProbabilityTable.cpp src/helper/ProbabilityTable.cpp src/helper/MathHelper.cpp src/helper/SerializationHelper.cpp src/helper/ObjectTransformation.cpp
			    
			    src/inference/model/foreground/ocm/shape/conditional_probability/MinimumConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/MultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/RootOfMultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/AverageConditionalProbability.cpp
			    )

target_link_libraries(inference 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  
)

add_executable(inference_batch src/inference_batch.cpp
 			    src/inference/SceneInferenceEngine.cpp

			    src/inference/model/SceneModelDescription.cpp src/inference/model/SceneDescription.cpp src/inference/model/SceneContent.cpp src/inference/model/InferenceAlgorithm.cpp src/inference/model/SceneIdentifier.cpp src/inference/model/ObjectEvidence.cpp src/inference/model/KalmanFilter.cpp
			    
			    src/inference/model/background/BackgroundSceneContent.cpp
			    src/inference/model/background/BackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/PowerSetBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/SummarizedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MultipliedBackgroundInferenceAlgorithm.cpp
			    src/inference/model/background/MaximumBackgroundInferenceAlgorithm.cpp
			    
			    src/inference/model/foreground/ForegroundSceneContent.cpp
			    src/inference/model/foreground/ForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/PowerSetForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SummarizedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MultipliedForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/MaximumForegroundInferenceAlgorithm.cpp
			    src/inference/model/foreground/SceneObjectDescription.cpp src/inference/model/foreground/SceneObjectContent.cpp

			    src/inference/model/foreground/ocm/OcmSceneObjectContent.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModel.cpp src/inference/model/foreground/ocm/shape/ShapeTermEvaluator.cpp src/inference/model/foreground/ocm/shape/HierarchicalShapeModelNode.cpp src/inference/model/foreground/ocm/shape/GaussianMixtureDistribution.cpp src/inference/model/foreground/ocm/shape/GaussianKernel.cpp
			    src/inference/model/foreground/ocm/appearance/AppearanceTermEvaluator.cpp src/inference/model/foreground/ocm/occlusion/OcclusionTermEvaluator.cpp src/inference/model/foreground/ocm/TermEvaluator.cpp
			    
			    src/helper/MappedProbabilityTable.cpp src/helper/ProbabilityTable.cpp src/helper/MathHelper.cpp src/helper/SerializationHelper.cpp src/helper/ObjectTransformation.cpp
			    
			    src/inference/model/foreground/ocm/shape/conditional_probability/MinimumConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/MultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/RootOfMultipliedConditionalProbability.cpp
			    src/inference/model/foreground/ocm/shape/conditional_probability/AverageConditionalProbability.cpp)
target_link_libraries(inference_batch
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}  
  
)
