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

import csv
import numpy as np
import matplotlib.pyplot as plt

# Used for increasing all values less than the given value to make them visible on the bar chart.
thresholdForValueIncreasing = 0.1
thresholdForValueIncreasingStandalone = 0.001
thresholdForValueIncreasingStandaloneLog = 0.00001
  
# Define colors for bar segments.
# For colors see here: http://micropore.wordpress.com/2012/05/16/python-html-colors/
#colors = ['#b167ce','#77cfbe','#9e79cd','#80d6a4','#9785d6','#8fd3a6']
colors = ['Salmon','Khaki','GreenYellow','Aquamarine','Thistle','Moccasin']

def generateRuntimePlot(pathToCsv, pathToImage, meanOverTurns, xtics, textoffsetX, textoffsetY):
  
  maxSamples = 4
  maxSceneobjects = 6
  samplesPerKernel = 100
  
  #************************************#
  # Preprocess values.
  #************************************#
  
  # Iterate over all ISM experiments and calculate its runtime.
  psmScore = []
  for samples in range(1, maxSamples + 1):
    for sceneobjects in range(0, maxSceneobjects):
      experiment = str(samples * samplesPerKernel) + '_' + str(sceneobjects).zfill(2)
      pathExperimentCsv = pathToCsv + '/' + experiment + '/' + experiment + '.csv'
      
      # Get the number of evaluation turns from the number of samples.
      numberOfTurns = sum(1 for row in csv.reader(open(pathExperimentCsv, 'rb'), delimiter=','))
      
      if meanOverTurns:
	numberOfTurns /= (sceneobjects + 1)
	print 'Evaluating experiment ' + experiment + '. Number of turns is ' + str(numberOfTurns) + '.'
      else:	
	print 'Evaluating experiment ' + experiment + '.'
      
      # Open experiment file and calculate the mean runtime.
      with open(pathExperimentCsv, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=',')
	
	# Skip csv header.
	reader.next()
	
	score = []
	# Sum over all samples of a turn, then add it to the score list.
	for turn in range(0, numberOfTurns - 1):
	  turnScore = 0.0;
	  
	  if meanOverTurns:
	    for i in range(0, sceneobjects):
	      turnScore += float(reader.next()[1])
	    turnScore /= (sceneobjects + 1)
	  else:
	    turnScore += float(reader.next()[6])
	    
	  score.append(turnScore)
	  
	# Calculate the runtime as mean of the score list.
	# Skip the first six entries because they were calculated for an insufficient number of evidences.
	if meanOverTurns:
	  psmScore.append(np.array(score[6:]).mean())
	else:
	  psmScore.append(np.array(score).mean())
  
  #************************************#
  # Convert data in plottable format.
  #************************************#
  
  # Create yLabels
  yLabels = []
  for samples in range(1, maxSamples + 1):
    yLabels.append(samples * samplesPerKernel)
  
  # Build data and labels.
  data = np.zeros((maxSceneobjects, maxSamples))
  labels = np.zeros((maxSamples, maxSceneobjects))
  for x in range(0, maxSamples):
    for y in range(0, maxSceneobjects):
      value = psmScore[x * (maxSceneobjects) + y]
      
      # Set label
      labels[x][y] = value
      
      # Every value to small to be visualized is set to a given threshold value.
      #if value  < thresholdForValueIncreasing:
	#data[y][x] = thresholdForValueIncreasing
      #else:
	#data[y][x] = value
      data[y][x] = value
  
  #************************************#
  # Plot stacked bar diagram.
  #************************************#
  
  # Build indices for position on y-axis.
  y_pos = np.arange(len(yLabels))
  
  # Prepare figure
  fig = plt.figure(figsize=(10,4))
  ax = fig.add_subplot(111)
  
  # Generate patches for each bar.
  patch_handles = []
  left = np.zeros(len(yLabels))
  for i, d in enumerate(data):
    patch_handles.append(ax.barh(y_pos, d, color=colors[i % len(colors)], align='center', label='%i' % (i + 1), left=left))
    left += d

  # Annotate each bar segment with the given label.
  for j in xrange(len(patch_handles)):
    for i, patch in enumerate(patch_handles[j].get_children()):
        bl = patch.get_xy()
        
        # Set text position.
	x = 0.5 * patch.get_width() + bl[0] + textoffsetX
        y = 0.5 * patch.get_height() + bl[1] + textoffsetY
        
        # Print text, if there's enough space
        if meanOverTurns:
	  if labels[i,j] > 110:
	    ax.text(x,y, "%10.2f" % (labels[i,j]), ha='center', fontweight='bold', fontsize=15)
	else:
	  if labels[i,j] > 1.5:
	    ax.text(x,y, "%10.2f" % (labels[i,j]), ha='center', fontweight='bold', fontsize=15)

  # Configure axes and plot.
  ax.set_yticks(y_pos)
  ax.set_yticklabels(yLabels)
  
  ax.xaxis.grid()

  ax.set_xlabel('Laufzeit in Millisekunden', fontweight='bold')
  ax.set_ylabel('Datenpunkte pro Trajektorie', fontweight='bold')
  ax.invert_yaxis()
  
  ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.08), ncol=6, fancybox=True, shadow=True)
  
  plt.xlim([0,xtics])
  #plt.show()
  plt.savefig(pathToImage)
  
def generateRuntimePlotStandalone(pathToCsv, pathToImage, maxSceneobjects, useLog, textoffsetX, textoffsetY):
  
  #************************************#
  # Preprocess values.
  #************************************#
  
  # Iterate over all ISM experiments and calculate its runtime.
  psmScore = []
  for obj in range(1, maxSceneobjects + 1):
    for evidence in range(1, maxSceneobjects + 1):
      pathExperimentCsv = pathToCsv + '/obj_' + str(obj) + '_run_' + str(evidence) + '/run_' + str(obj) + '.csv'
      
      print 'Evaluating ' + str(obj) + ' object(s) with ' + str(evidence) + ' evidences.'
      
      # Get the number of evaluation turns from the number of samples.
      numberOfTurns = sum(1 for row in csv.reader(open(pathExperimentCsv, 'rb'), delimiter=','))
      numberOfTurns /= (maxSceneobjects + 1)
      
      # Open experiment file and calculate the mean runtime.
      with open(pathExperimentCsv, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=',')
	
	# Skip csv header.
	reader.next()
	
	score = []
	# Sum over all samples of a turn, then add it to the score list.
	for turn in range(0, numberOfTurns - 1):
	  turnScore = 0.0;

	  for i in range(0, obj):
	    turnScore += float(reader.next()[1])

	  score.append(turnScore / obj)
	  
	# Calculate the runtime as mean of the score list.
	# Skip the first six entries because they were calculated for an insufficient number of evidences.
	psmScore.append(np.array(score[6:]).mean())
  
  #************************************#
  # Convert data in plottable format.
  #************************************#
  
  # Create yLabels
  yLabels = []
  for samples in range(1, maxSceneobjects + 1):
    yLabels.append(samples)
  
  # Build data and labels.
  data = np.zeros((maxSceneobjects, maxSceneobjects))
  labels = np.zeros((maxSceneobjects, maxSceneobjects))
  for x in range(0, maxSceneobjects):
    for y in range(0, maxSceneobjects):
      value = psmScore[x * (maxSceneobjects) + y]
      
      # Set label
      labels[x][y] = value
      
      # Every value to small to be visualized is set to a given threshold value.
      if useLog:
	#if value < thresholdForValueIncreasingStandaloneLog:
	  #data[y][x] = thresholdForValueIncreasingStandaloneLog
	#else:
	  #data[y][x] = value
	data[y][x] = value
      else:
	#if value < thresholdForValueIncreasingStandalone:
	  #data[y][x] = thresholdForValueIncreasingStandalone
	#else:
	  #data[y][x] = value
        data[y][x] = value
	  
  #************************************#
  # Plot stacked bar diagram.
  #************************************#
  
  # Build indices for position on y-axis.
  y_pos = np.arange(len(yLabels))
  
  # Prepare figure
  fig = plt.figure(figsize=(10,4))
  ax = fig.add_subplot(111)
  
  # Generate patches for each bar.
  patch_handles = []
  left = np.zeros(len(yLabels))
  for i, d in enumerate(data):
    patch_handles.append(ax.barh(y_pos, d, color=colors[i % len(colors)], align='center', label='%i' % (i + 1), left=left))
    left += d

  # Annotate each bar segment with the given label.
  for j in xrange(len(patch_handles)):
    for i, patch in enumerate(patch_handles[j].get_children()):
        bl = patch.get_xy()
        
        # Set text position.
	x = 0.5 * patch.get_width() + bl[0] + textoffsetX
        y = 0.5 * patch.get_height() + bl[1] + textoffsetY
        
        # Print text, if there's enough space
	if labels[i,j] > 5:
	  ax.text(x,y, "%10.2f" % (labels[i,j]), ha='center', va='center', fontweight='bold', fontsize=15)

  # Configure axes and plot.
  ax.set_yticks(y_pos)
  ax.set_yticklabels(yLabels)
  
  ax.xaxis.grid()

  ax.set_xlabel('Laufzeit in Millisekunden', fontweight='bold')
  ax.set_ylabel('Objekte pro Szene', fontweight='bold')
  ax.invert_yaxis()
  
  ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.08), ncol=6, fancybox=True, shadow=True)
  
  # Set x-axis to logarithmic
  if useLog:
    ax.set_xscale('log')

  plt.xlim([0,40])
  #plt.show()
  plt.savefig(pathToImage)

def generateRuntimePlotKickass(pathToCsv, pathToImage, maxSceneobjects, useLog, meanOverTurns):
  
  #************************************#
  # Preprocess values.
  #************************************#
  
  # Iterate over all ISM experiments and calculate its runtime.
  psmScore = []
  for obj in range(1, maxSceneobjects + 1):
    sceneId = 'run_' + str(obj).zfill(2)
    pathExperimentCsv = pathToCsv + '/' + sceneId + '/' + sceneId + '.csv'
      
    print 'Evaluating ' + str(obj) + ' object(s) with ' + str(obj) + ' evidences.'
      
    # Get the number of evaluation turns from the number of samples.
    numberOfTurns = sum(1 for row in csv.reader(open(pathExperimentCsv, 'rb'), delimiter=','))

    if meanOverTurns:
      numberOfTurns /= obj

    # Open experiment file and calculate the mean runtime.
    reader = csv.reader(open(pathExperimentCsv, 'rb'), delimiter=',')

    # Skip csv header.
    reader.next()

    score = []
    # Sum over all samples of a turn, then add it to the score list.
    for turn in range(0, numberOfTurns - 1):
      turnScore = 0.0;

      if meanOverTurns:
	for i in range(0, obj):
	  turnScore += float(reader.next()[1])
	turnScore /= obj
      else:
	turnScore += float(reader.next()[6])
	    
      score.append(turnScore)
	  
    # Calculate the runtime as mean of the score list.
    # Skip the first six entries because they were calculated for an insufficient number of evidences.
    if meanOverTurns:
      objScore = np.array(score[6:]).mean() / 1000
    else:
      objScore = np.array(score).mean() / 1000
    
    #if objScore == 0.0: objScore = 1.0
    #print str(obj) + ": " + str(objScore)
    psmScore.append(objScore)

  #************************************#
  # Plot line diagram.
  #************************************#

  # Create data for x- and y-axis
  xData = np.arange(1.0, maxSceneobjects + 1, 1.0)
  yData = np.array(psmScore)

  # Prepare figure
  fig = plt.figure(figsize=(10,4))
  ax = fig.add_subplot(111)
  
  ax.xaxis.grid()

  ax.set_xlabel('Objekte pro Szene', fontweight='bold')
  ax.set_ylabel('Laufzeit in Sekunden', fontweight='bold')

  ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.08), ncol=6, fancybox=True, shadow=True)
  
  # Set x-axis to logarithmic
  if useLog:
    ax.set_yscale('log')

  ax.plot(xData, yData, 'ro-')

  #plt.show()
  plt.savefig(pathToImage)
  
def main():
  
  # Path to the directory containing the results of the experiments.
  pathToExperiments = '/home/joachim/Schreibtisch/Evaluation/Laufzeitmessung'
  
  # Generate runtime bar plots.
  #generateRuntimePlot(pathToExperiments + '/simple/runtime/PSM', pathToExperiments + '/images/PSM_simple.png', True, 600, -10.0, 0.1)
  #generateRuntimePlot(pathToExperiments + '/simple/runtime/ISM', pathToExperiments + '/images/ISM_simple.png', False, 25, -0.7, 0.1)
  #generateRuntimePlot(pathToExperiments + '/complex/runtime/PSM', pathToExperiments + '/images/PSM_complex.png', True, 600, -10.0, 0.1)
  #generateRuntimePlot(pathToExperiments + '/complex/runtime/ISM', pathToExperiments + '/images/ISM_complex.png', False, 25, -0.7, 0.1)
  
  generateRuntimePlotStandalone(pathToExperiments + '/standalone/runtime/PSM', pathToExperiments + '/images/PSM_standalone.png', 5, False, -1.2, 0.05)
  #generateRuntimePlotKickass(pathToExperiments + '/kickass/runtime/PSM', pathToExperiments + '/images/PSM_kickass.png', 8, False, True)
  #generateRuntimePlotKickass(pathToExperiments + '/kickass/runtime/ISM', pathToExperiments + '/images/ISM_kickass.png', 8, False, False)

if __name__ == "__main__":
    main()
