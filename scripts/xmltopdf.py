# script to transform a PSM-xml model into a graph (saved as pdf) with objects as nodes and parent-child relationships as vertices using graphviz dot.
#the central function here is xmltopdf(). it takes a directory (with trailing "//" or "\") and a name (without file ending).
#it opens the directory + <name>.xml model and creates pdfs containing the trees described in the models as graphs for each scene and reference object in the model
#using graphviz (for usage example, see "test()").

import subprocess
from copy import deepcopy

graphviz_command = "dot"

class TreeNode:
  name = ""
  parent = []
  reference = False

def read(directory, name):
  filename = directory + name + ".xml"
  with open(filename, 'r') as i:
    contents = i.read()
    return contents

def xmltodot(directory, name):
  contents = read(directory, name)
  scenes = contents.split("<scene ")
  dots = []
  bitcomp = ""
  bitcomp += directory + name + "; "

  nodelist = []
  rootlist = []
  for scene in scenes:
    if not scene.find("ocm") == -1:
      scenename = scene.split("name=\"")[1].split("\"")[0]
      nodes = scene.split("<")
      del nodelist[:]
      del rootlist[:]
      for node in nodes:
        values = node.split(" ")
        if values[0] == "object":
          if len(nodelist) > 0:
            rootlist.append([deepcopy(nodelist), root.name])
            del nodelist[:]
          top = TreeNode()
          top.name = "NULL"
          root = TreeNode()
          root.name = values[1].split("\"")[1]
          root.parent = [top]
          currentroot = root
          nodelist.append(root) 
        if values[0] == "child":
          child = TreeNode()
          child.name = values[1].split("\"")[1]
          child.parent = [currentroot]
          child.reference = (len(values) > 2)
          currentroot = child
          nodelist.append(child)
        if values[0] == "/child>":
          currentroot = currentroot.parent[0]
      rootlist.append([deepcopy(nodelist), root.name]) #last root
          
      for root in rootlist:
        nodelist = root[0]
        rootname = root[1]

        dot = "digraph " + name + "_" + rootname + " {\n"
        
        #create graph nodes:
        for o in rootlist:
          dot += o[1] + "[label=" + o[1] + "]\n"
        dot += "\n"

        #create edges:
        for o in nodelist:
          if not o.name == "NULL" and not o.parent[0].name == "NULL":
            dot += o.parent[0].name + " -> " + o.name
            if o.reference:
              dot += "[style=dashed]"
            dot += "\n"              

        dot += "}\n\n"

        print(dot)

        dotname = scenename + "_" + rootname
        outname = directory + name + "_" + dotname + ".dot"
        with open(outname, 'w') as f:
          f.write(dot)

        dots.append(scenename + "_" + rootname)
        
  return dots

def dottopdf(directory, name, graphs):
  for graph in graphs:
    subprocess.call([graphviz_command, "-Tpdf", directory + name + "_" + graph + ".dot", "-o", directory + name + "_" + graph + ".pdf"])

def xmltopdf(directory, name):
  graphs = xmltodot(directory, name)
  dottopdf(directory, name, graphs)

def test():  
  directory = "../data/"
  name = "advertisement"
  xmltopdf(directory, name)
