#!/usr/bin/env python

'''
Copyright (c) 2016, Braun Kai, Gehrung Joachim, Heizmann Heinrich, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import os

import rospy
import rosbag
import roslib

import random
import math
import time

import subprocess

roslib.load_manifest('psm')

from pbd_msgs.msg import PbdObject
from visualization_msgs.msg import Marker

# NOTE: IN CASE THE SCENE_GRAPH_GENERATOR CANT TRANSFORM THE OBJECTS INTO THE RIGHT COORDINATE FRAME,
#       DO THE FOLLOWING: roslaunch kinematic_chain_pbd transformation_publishers_left.launch


def publishPdbObjectMessage(pub, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType):
  
  # build object ...
  msg = PbdObject()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = '/PTU'
  msg.providedBy = 'textured'
  msg.poseEstimation.pose.position.x = posX;
  msg.poseEstimation.pose.position.y = posY;
  msg.poseEstimation.pose.position.z = posZ;
  msg.poseEstimation.pose.orientation.w = oriW;
  msg.poseEstimation.pose.orientation.x = oriX;
  msg.poseEstimation.pose.orientation.y = oriY;
  msg.poseEstimation.pose.orientation.z = oriZ;
  msg.type = objType
  
  # ... and publish it
  pub.publish(msg)

def rosbagPdbObjectMessage(bag, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType):
  
  # build object ...
  msg = PbdObject()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = '/PTU'
  msg.providedBy = 'textured'
  msg.poseEstimation.pose.position.x = posX;
  msg.poseEstimation.pose.position.y = posY;
  msg.poseEstimation.pose.position.z = posZ;
  msg.poseEstimation.pose.orientation.w = oriW;
  msg.poseEstimation.pose.orientation.x = oriX;
  msg.poseEstimation.pose.orientation.y = oriY;
  msg.poseEstimation.pose.orientation.z = oriZ;
  msg.type = objType
  
  # ... and write it into bag file
  bag.write('/stereo/objects', msg)

def publishVisualizationMessage(pub, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh):
  
  # build object ...
  msg = Marker()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = '/PTU'
  msg.ns = 'textured'
  msg.id = mid
  msg.type = 10
  msg.action = 0
  msg.pose.position.x = posX;
  msg.pose.position.y = posY;
  msg.pose.position.z = posZ;
  msg.pose.orientation.w = oriW;
  msg.pose.orientation.x = oriX;
  msg.pose.orientation.y = oriY;
  msg.pose.orientation.z = oriZ;
  msg.scale.x = 0.001
  msg.scale.y = 0.001
  msg.scale.z = 0.001
  msg.mesh_resource = mesh
  msg.mesh_use_embedded_materials = True
  
  # ... and publish it
  pub.publish(msg)
  
def rosbagVisualizationMessage(bag, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh):
  
  # build object ...
  msg = Marker()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = '/PTU'
  msg.ns = 'textured'
  msg.id = mid
  msg.type = 10
  msg.action = 0
  msg.pose.position.x = posX;
  msg.pose.position.y = posY;
  msg.pose.position.z = posZ;
  msg.pose.orientation.w = oriW;
  msg.pose.orientation.x = oriX;
  msg.pose.orientation.y = oriY;
  msg.pose.orientation.z = oriZ;
  msg.scale.x = 0.001
  msg.scale.y = 0.001
  msg.scale.z = 0.001
  msg.mesh_resource = mesh
  msg.mesh_use_embedded_materials = True
  
  # ... and write it into bag file
  bag.write('/stereo/visualization_marker', msg)
  
def publishEvidence(bag, pubObj, pubVis, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh):
  
  # publish
  publishPdbObjectMessage(pubObj, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType)
  publishVisualizationMessage(pubVis, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh)
  
  # if rosbag defined, also write to rosbag
  if bag:
    rosbagPdbObjectMessage(bag, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType)
    rosbagVisualizationMessage(bag, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh)
  
def publishNoisedEvidence(bag, pubObj, pubVis, posX, posY, posZ, oriW, oriX, oriY, oriZ, objType, mid, mesh, sigma):
  
  # generate random noise for the position
  pX = posX + random.normalvariate(0.0, sigma)
  pY = posY + random.normalvariate(0.0, sigma)
  pZ = posZ + random.normalvariate(0.0, sigma)
  
  # generate random noise for the orientation
  oW = oriW + random.normalvariate(0.0, sigma)
  oX = oriX + random.normalvariate(0.0, sigma)
  oY = oriY + random.normalvariate(0.0, sigma)
  oZ = oriZ + random.normalvariate(0.0, sigma)
  
  # normalize quaternion
  magnitude = math.sqrt(math.pow(oW, 2) + math.pow(oX, 2) + math.pow(oY, 2) + math.pow(oZ, 2))
  
  oW /= magnitude
  oX /= magnitude
  oY /= magnitude
  oZ /= magnitude

  # publish object and visualization message
  publishEvidence(bag, pubObj, pubVis, pX, pY, pZ, oW, oX, oY, oZ, objType, mid, mesh)
  
def simulateFourObjectsSzenario(meshTexturedPath, meshSegmentablePath, pubObj, pubVis):
  
  print "Running sample generation."
  
  for i in range(0,1000):
    sigma = 0.05;
    
    # subset_scene_simulated
    publishNoisedEvidence(None, pubObj, pubVis, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko', 0, meshTexturedPath + 'vitalis_schoko.dae', sigma)
    publishNoisedEvidence(None, pubObj, pubVis, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Smacks', 1, meshTexturedPath + 'smacks.dae', sigma)
    publishNoisedEvidence(None, pubObj, pubVis, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'CoffeeBox', 2, meshTexturedPath + 'coffeebox.dae', sigma)
    publishNoisedEvidence(None, pubObj, pubVis, 1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Plate', 3, meshSegmentablePath + 'plate_deep/object.dae', sigma)
  
    time.sleep(0.01)
    
  print "Done generating samples."
  
def simulateObjectEvidence(meshTexturedPath, meshSegmentablePath, pubObj, pubVis, number):
  
  print "Generating object evidences."
  
  for i in range(0,1000):
    
    sigma = 0.05;
    
    # publish n object evidence
    for obj in range(1, number):
      publishNoisedEvidence(None, pubObj, pubVis, i * (1.0 / samplesOnEdge), obj + 1, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko-' + str(obj), obj + 1, meshTexturedPath + 'vitalis_schoko.dae', 0.05)
      
    time.sleep(0.01)
    
  print "Done generating object evidences."
  
def runtimeCreateSamplesSimple(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      # create rosbag
      bag = rosbag.Bag(bagObjects, 'w')
      
      try:
	# create m samples...
	for i in range(0, numberOfSamples):
	  
	  publishNoisedEvidence(bag, pubObj, pubVis, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Smacks', 0, meshTexturedPath + 'smacks.dae', 0.05)
	  
	  # publish evidence into rosbag, also publish to message system for online visualization
	  for obj in range(0, sceneobject):
	    publishNoisedEvidence(bag, pubObj, pubVis, i * (1.0 / samplesOnEdge), obj + 1, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko-' + str(obj), obj + 1, meshTexturedPath + 'vitalis_schoko.dae', 0.05)
      finally:
	bag.close()
	
def runtimePrepareStandalonePSM(meshTexturedPath, pubObj, pubVis, pathToBags, sceneObjects, samplesPerObject):
  
  # iterate over all objects to generate models
  for sceneobject in range(1, sceneObjects + 1):

    print 'Publishing ' + str(samplesPerObject) + ' samples for ' + str(sceneobject) + ' objects.'
    
    # define the id of the scene and the path to the bag file
    sceneId = 'run_' + str(sceneobject).zfill(2)
    bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
    
    # create rosbag
    bag = rosbag.Bag(bagObjects, 'w')
    
    try:
      # publish evidence into rosbag, also publish to message system for online visualization
      for i in range(0, samplesPerObject):
	for obj in range(0, sceneobject):
	  publishNoisedEvidence(bag, pubObj, pubVis, 0.0, obj + 1, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko-' + str(obj), obj + 1, meshTexturedPath + 'vitalis_schoko.dae', 0.05)
    finally:
      bag.close()
      
    #print 'Generating scene graphs.'
    
    #bagSceneGraph = pathToBags + '/scenegraphs/SCENEGRAPH_' + sceneId + '.bag'
    
    #subprocess.call(['rosrun', 'scene_graph_generator', 'scene_graph_generator', '_base_frame:=/PTU', '_scene_id:=' + sceneId,'_im_frame_id:=null', '_rosbag_source:=' + bagObjects, '_rosbag_target:=' + bagSceneGraph])
    
    #print "Learn scene model."
    
    ## Create a temporary launch file and execute it.
    #with open(pathToBags + "/temp.launch", "wt") as fout:
      #with open(pathToBags + "/learner.launch", "rt") as fin:
	#for line in fin:
	  #line = line.replace('$$$SCENE_GRAPH_BAGS$$$', '[\'' + bagSceneGraph + '\']')
	  #line = line.replace('$$$SCENARIO$$$', sceneId)
	  #line = line.replace('$$$OUTPUT_DIRECTORY$$$', pathToBags + '/psm_models')
	  #line = line.replace('$$$KERNELS_MIN$$$', "1")
	  #line = line.replace('$$$KERNELS_MAX$$$', "1")
	  #fout.write(line)

    #subprocess.check_call(['roslaunch', pathToBags + '/temp.launch'])
    #os.remove(pathToBags + "/temp.launch")
    
def runtimeEvaluateStandalonePSM(meshTexturedPath, pubObj, pubVis, pathToBags, sceneObjects, samplesPerObject):
  
  evidences = sceneObjects
  
  # Iterate over all objects to execute runtime evaluation.
  for sceneobject in range(1, sceneObjects + 1):
    
    # Evaluate all number of evidences up to the given threshold for the current model.
    for evidence in range(1, evidences + 1):
      
      print "Execute PSM inference for " + str(evidence) + " evidences."
      
      # Define scene id, name of the model and the bag file.
      sceneId = 'run_' + str(evidence).zfill(2)
      modelFilename = pathToBags + '/psm_models/run_' + str(sceneobject) + '.xml'
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      # Create a subfolder for the runtime evaluation results.
      psmRuntimeLogFolder = pathToBags + '/runtime/obj_' + str(sceneobject) + '_' + sceneId
      if not os.path.exists(psmRuntimeLogFolder):
	os.makedirs(psmRuntimeLogFolder)
      
      # Execute the PSM inference in batch mode to get the runtime values.
      subprocess.call(['rosrun', 'psm', 'inference_batch', '_plot:=false', '_object_topic:=/stereo/objects', '_scene_graph_topic:=/scene_graphs', '_scene_model_filename:=' + modelFilename, '_bag_filenames_list:=', '_base_frame_id:=/PTU', '_scale_factor:=1.0', '_sigma_multiplicator:=2.0', '_targeting_help:=false', '_inference_algorithm:=maximum', '_runtime_log_path:=' + psmRuntimeLogFolder, '_bag_path:=' + bagObjects])
      
def runtimeEvaluateStandalonePSMKickass(meshTexturedPath, pubObj, pubVis, pathToBags, sceneObjects):
  
  # Iterate over all objects to execute runtime evaluation.
  for sceneobject in range(1, sceneObjects + 1):
    
    print "Execute PSM inference for " + str(sceneobject) + " evidences."
    
    # Define scene id, name of the model and the bag file.
    sceneId = 'run_' + str(sceneobject).zfill(2)
    modelFilename = pathToBags + '/psm_models/' + sceneId + '.xml'
    bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
    
    # Create a subfolder for the runtime evaluation results.
    psmRuntimeLogFolder = pathToBags + '/runtime/PSM/' + sceneId
    if not os.path.exists(psmRuntimeLogFolder):
      os.makedirs(psmRuntimeLogFolder)
    
    # Execute the PSM inference in batch mode to get the runtime values.
    subprocess.call(['rosrun', 'psm', 'inference_batch', '_plot:=false', '_object_topic:=/stereo/objects', '_scene_graph_topic:=/scene_graphs', '_scene_model_filename:=' + modelFilename, '_bag_filenames_list:=', '_base_frame_id:=/PTU', '_scale_factor:=1.0', '_sigma_multiplicator:=2.0', '_targeting_help:=false', '_inference_algorithm:=maximum', '_runtime_log_path:=' + psmRuntimeLogFolder, '_bag_path:=' + bagObjects])
      
def runtimeCreateSamplesComplex(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects(' + str(scenario) + " clusters)"
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      # direction of movement and position offsets
      direction = 0
      posX = 0
      posY = 0
      
      # create rosbag
      bag = rosbag.Bag(bagObjects, 'w')
      
      try:
	#  iterate over all turns
	for seq in range(0, scenario):
	  for i in range(0, samplesOnEdge):
	    if direction:
	      posY += 1.0 / samplesOnEdge
	    else:
	      posX += 1.0 / samplesOnEdge
	    
	    publishNoisedEvidence(bag, pubObj, pubVis, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'Smacks', 0, meshTexturedPath + 'smacks.dae', 0.05)
	    
	    # publish evidence into rosbag, also publish to message system for online visualization
	    for obj in range(0, sceneobject):
	      publishNoisedEvidence(bag, pubObj, pubVis, posX - obj, posY + obj, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko-' + str(obj), obj + 1, meshTexturedPath + 'vitalis_schoko.dae', 0.05)
	      
	  # toggle between directions
	  direction += 1
	  if direction > 1:
	    direction = 0
      finally:
	bag.close()
	
def runtimePreparePSM(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge, useComplex):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
	
      print 'Generating scene graphs.'
      
      bagSceneGraph = pathToBags + '/scenegraphs/SCENEGRAPH_' + sceneId + '.bag'
	
      # Execute subprocess that generates the scene graph
      subprocess.call(['rosrun', 'scene_graph_generator', 'scene_graph_generator', '_base_frame:=/PTU', '_scene_id:=' + sceneId,'_im_frame_id:=null', '_rosbag_source:=' + bagObjects, '_rosbag_target:=' + bagSceneGraph])
      
      print "Learn scene model."
      6,
      # Create a temporary launch file and execute it.
      with open(pathToBags + "/temp.launch", "wt") as fout:
	with open(pathToBags + "/learner.launch", "rt") as fin:
	  for line in fin:
	    line = line.replace('$$$SCENE_GRAPH_BAGS$$$', '[\'' + bagSceneGraph + '\']')
	    line = line.replace('$$$SCENARIO$$$', sceneId)
	    line = line.replace('$$$OUTPUT_DIRECTORY$$$', pathToBags + '/psm_models')
	    
	    if useComplex:
	      line = line.replace('$$$KERNELS_MIN$$$', str(scenario))
	      line = line.replace('$$$KERNELS_MAX$$$', str(scenario))
	    else:
	      line = line.replace('$$$KERNELS_MIN$$$', "1")
	      line = line.replace('$$$KERNELS_MAX$$$', "1")
	    fout.write(line)

      subprocess.check_call(['roslaunch', pathToBags + '/temp.launch'])
      os.remove(pathToBags + "/temp.launch")
      
def runtimeEvaluatePSM(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      print "Execute PSM inference in batch mode to determine runtime."
      
      modelFilename = pathToBags + '/psm_models/' + sceneId + '.xml'
      
      # Create a subfolder for the runtime evaluation results.
      psmRuntimeLogFolder = pathToBags + '/runtime/PSM/' + sceneId
      if not os.path.exists(psmRuntimeLogFolder):
	os.makedirs(psmRuntimeLogFolder)
      
      # Execute the PSM inference in batch mode to get the runtime values.
      subprocess.call(['rosrun', 'psm', 'inference_batch', '_plot:=false', '_object_topic:=/stereo/objects', '_scene_graph_topic:=/scene_graphs', '_scene_model_filename:=' + modelFilename, '_bag_filenames_list:=', '_base_frame_id:=/PTU', '_scale_factor:=1.0', '_sigma_multiplicator:=2.0', '_targeting_help:=false', '_inference_algorithm:=maximum', '_runtime_log_path:=' + psmRuntimeLogFolder, '_bag_path:=' + bagObjects])
      
def runtimeEvaluatePSMStandalone(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      print "Execute PSM inference in batch mode to determine runtime."
      
      modelFilename = pathToBags + '/psm_models/' + sceneId + '.xml'
      
      # Create a subfolder for the runtime evaluation results.
      psmRuntimeLogFolder = pathToBags + '/runtime/PSM/' + sceneId
      if not os.path.exists(psmRuntimeLogFolder):
	os.makedirs(psmRuntimeLogFolder)
      
      # Execute the PSM inference in batch mode to get the runtime values.
      subprocess.call(['rosrun', 'psm', 'inference_batch', '_plot:=false', '_object_topic:=/stereo/objects', '_scene_graph_topic:=/scene_graphs', '_scene_model_filename:=' + modelFilename, '_bag_filenames_list:=', '_base_frame_id:=/PTU', '_scale_factor:=1.0', '_sigma_multiplicator:=2.0', '_targeting_help:=false', '_inference_algorithm:=maximum', '_runtime_log_path:=' + psmRuntimeLogFolder, '_bag_path:=' + bagObjects])
      
def runtimePrepareISM(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):
  
  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      print "Build ISM database."
      
      pathToIsmDatabase = pathToBags + '/ism_models/' + sceneId + '.sqlite.ism_test'
      
      # Run the database build process.
      subprocess.call(['rosrun', 'ism', 'recorder_batch', '_objectTopic:=/stereo/objects', '_cameraInfoTopic:=/stereo/left/camera_info', '_window_adjustment:=1', '_sceneName:=' + sceneId, '_capture_interval:=1', '_baseFrame:=/PTU', '_dbfilename:=' + pathToIsmDatabase, '_multiview_recording:=true', '_bag_path:=' + bagObjects])
      
      # Train the database build in the last step.
      subprocess.call(['rosrun', 'ism', 'trainer_batch', '_useClustering:=true', '_staticBreakRatio:=0.01', '_togetherRatio:=0.90', '_maxAngleDeviation:=45' , '_visualization_topic:=/visualization_marker', '_baseFrame:=/PTU', '_dbfilename:=' + pathToIsmDatabase])

def runtimePrepareStandaloneISM(meshTexturedPath, pubObj, pubVis, pathToBags, sceneObjects):

  # iterate over all scene objects in the scenario
  for sceneobject in range(1, sceneObjects + 1):
    print 'Creating and learning database for ' + str(sceneobject) + ' objects.'
      
    # define the id of the scene and the path to the bag file
    sceneId = '/run_' + str(sceneobject).zfill(2)
    bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
    print "Build ISM database."
      
    pathToIsmDatabase = pathToBags + '/ism_models/' + sceneId + '.sqlite.ism_test'
      
    # Run the database build process.
    subprocess.call(['rosrun', 'ism', 'recorder_batch', '_objectTopic:=/stereo/objects', '_cameraInfoTopic:=/stereo/left/camera_info', '_window_adjustment:=1', '_sceneName:=' + sceneId, '_capture_interval:=1', '_baseFrame:=/PTU', '_dbfilename:=' + pathToIsmDatabase, '_multiview_recording:=true', '_bag_path:=' + bagObjects])
      
    # Train the database build in the last step.
    subprocess.call(['rosrun', 'ism', 'trainer_batch', '_useClustering:=true', '_staticBreakRatio:=0.01', '_togetherRatio:=0.90', '_maxAngleDeviation:=45' , '_visualization_topic:=/visualization_marker', '_baseFrame:=/PTU', '_dbfilename:=' + pathToIsmDatabase])
      
def runtimeEvaluateISM(meshTexturedPath, pubObj, pubVis, pathToBags, scenarios, sceneObjects, samplesOnEdge):

  # iterate over all scenarios (that means number of samples times 100)
  for scenario in range(1, scenarios + 1):
    numberOfSamples = samplesOnEdge * scenario
    
    # iterate over all scene objects in the scenario
    for sceneobject in range(0, sceneObjects + 1):
      print 'Publishing ' + str(numberOfSamples) + ' samples with ' + str(sceneobject) + ' objects.'
      
      # define the id of the scene and the path to the bag file
      sceneId = str(numberOfSamples).zfill(3) + '_' + str(sceneobject).zfill(2)
      bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
      print "Execute ISM inference in batch mode to determine runtime."
      
      # Create a subfolder for the runtime evaluation results.
      ismRuntimeLogFolder = pathToBags + '/runtime/ISM/' + sceneId
      if not os.path.exists(ismRuntimeLogFolder):
	os.makedirs(ismRuntimeLogFolder)
	
      # Create path to test database.
      ismDatabase = pathToBags + '/ism_models/' + sceneId + '.sqlite.ism_test'
      ismRuntimeFile = ismRuntimeLogFolder + '/' + sceneId + '.csv'
      
      # Execute the ISM helper class to get the runtime values.
      subprocess.call(['rosrun', 'ism', 'testRunnerExtended', '-d', ismDatabase, '-t', ismTestDatabase, '-c', ismRuntimeFile, '-o', str(sceneobject + 1), '-s', str(numberOfSamples)])
      
      # Remove test database.
      os.remove(ismTestDatabase)

def runtimeEvaluateStandaloneISM(meshTexturedPath, pubObj, pubVis, pathToBags, sceneObjects, numberOfSamples):

  # iterate over all scene objects in the scenario
  for sceneobject in range(0, sceneObjects + 1):
    print 'Running runtime evaluation for ' + str(sceneobject) + ' objects.'
      
    # define the id of the scene and the path to the bag file
    sceneId = '/run_' + str(sceneobject).zfill(2)
    bagObjects = pathToBags + '/objects/' + sceneId + '.bag'
      
    print "Execute ISM inference in batch mode to determine runtime."
      
    # Create a subfolder for the runtime evaluation results.
    ismRuntimeLogFolder = pathToBags + '/runtime/ISM/' + sceneId
    if not os.path.exists(ismRuntimeLogFolder):
      os.makedirs(ismRuntimeLogFolder)
	
    # Create path to test database.
    ismDatabase = pathToBags + '/ism_models/' + sceneId + '.sqlite.ism_test'
    ismTestDatabase = ismRuntimeLogFolder + '/test.sqlite'
    ismRuntimeFile = ismRuntimeLogFolder + '/' + sceneId + '.csv'
      
    # Execute the ISM helper class to get the runtime values.
    subprocess.call(['rosrun', 'ism', 'testRunnerExtended', '-d', ismDatabase, '-t', ismTestDatabase, '-c', ismRuntimeFile, '-o', str(sceneobject + 1), '-s', str(numberOfSamples)])
      
    # Remove test database.
    os.remove(ismTestDatabase)  


def simulateFalseRecognitionBenachmark(meshTexturedPath, pubObj, pubVis, pathToBags):
  
  print "Generating samples for all objects"
  
  # create rosbag
  bag = rosbag.Bag(pathToBags + '/abweichung.bag', 'w')
  
  try:
    # create m samples...
    for i in range(0, 1000):
      
      # ...for a reference object...
      publishNoisedEvidence(bag, pubObj, pubVis, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'CoffeeBox', 0, meshTexturedPath + 'coffeebox.dae', 0.05)
      
      # ...and another object that positions forms multiple clusters.
      for pos in range(0, 5):
	publishNoisedEvidence(bag, pubObj, pubVis, -pos, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 'VitalisSchoko', 1, meshTexturedPath + 'vitalis_schoko.dae', 0.05)
  finally:
    bag.close()
    
def main():
  
  # path to object meshes
  meshPath = 'file:///homes/students/gehrung/Developement/ilcasRosPkg/trunk/perception/visual/object_localization/stereo_based/recognition_for_grasping/object_database/'
  meshTexturedPath = meshPath + "textured_objects/"
  meshSegmentablePath = meshPath + "segmentable_objects/"
  
  # path to bag file directory
  pathToBags = '/media/data_ext/data/gehrung/evaluation'
  
  # init node
  rospy.init_node('measurement_simulator')
  
  # instantiate publishers
  pubObj = rospy.Publisher('/stereo/objects', PbdObject)
  pubVis = rospy.Publisher('/stereo/visualization_marker', Marker)
  
  # Run one of the various szenarios
  #simulateFourObjectsSzenario(meshTexturedPath, meshSegmentablePath, pubObj, pubVis)
  #simulateObjectEvidence(meshTexturedPath, meshSegmentablePath, pubObj, pubVis, 1)
  #simulateFalseRecognitionBenachmark(meshTexturedPath, pubObj, pubVis, pathToBags)
  
  # Runtime benchmark
  #runtimeCreateSamplesSimple(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/simple/', 4, 5, 100)
  #runtimeCreateSamplesComplex(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/complex/', 4, 5, 100)
  
  #runtimePreparePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/simple', 4, 5, 100, False)
  #runtimePreparePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/complex', 4, 5, 100, True)
  #runtimePrepareISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/simple', 4, 5, 100)
  #runtimePrepareISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/complex', 4, 5, 100)
  
  #runtimeEvaluatePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/simple', 4, 5, 100)
  #runtimeEvaluatePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/complex', 4, 5, 100)
  #runtimeEvaluateISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/simple', 4, 5, 100)
  #runtimeEvaluateISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/complex', 4, 5, 100)
  
  # Standalone PSM Evaluation.
  #runtimePrepareStandalonePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/standalone', 6, 100)
  #runtimeEvaluateStandalonePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/standalone', 6, 100)
  
  # Compare ISM and PSM.
  #runtimePrepareStandalonePSM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/kickass', 10, 10)
  #runtimeEvaluateStandalonePSMKickass(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/kickass', 10)
  
  #runtimePrepareStandaloneISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/kickass', 10)
  runtimeEvaluateStandaloneISM(meshTexturedPath, pubObj, pubVis, pathToBags + '/Laufzeitmessung/kickass', 10, 10)

if __name__ == "__main__":
    main()
