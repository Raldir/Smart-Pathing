import networkx as nx
import matplotlib.pyplot as plt
import glob
import cv2
from PIL import Image
import numpy
import os


G = nx.DiGraph()
#print(os.path.basename(open(str(file), 'r').name).split('.',1)[0])
pos2 = {}
labels={}
#edgelabels = {}
f = open('dev/OutputMap/nodes', 'r')
for line in f:
    arguments = line.split()
# print(str(int(float(arguments[1]))) + " " + str(int(float(arguments[2]))) + '\n')
    G.add_node(float(arguments[0]), pos = (float(arguments[1]), float(arguments[2])))
    pos2[arguments[0]] = (arguments[1], arguments[2])
    labels[arguments[0]] = arguments[0]
f.close()
f = open('dev/OutputMap/edges', 'r')
for line in f:
    argumentsedge = line.split()
    G.add_edge(float(argumentsedge[0]), float(argumentsedge[1]), width = 0.05)
    G.add_edge(float(argumentsedge[1]), float(argumentsedge[0]), width = 0.05)
nx.draw_networkx_labels(G, pos2, labels, font_size = 0.1)
#print(G.nodes())
pos= nx.get_node_attributes(G,'pos')
edges = G.edges()
width = [G[u][v]['width'] for u,v in edges]
#edgelabels =  [G[u][v]['weight'] for u,v in edges]
#edgelabels =[(u,v) for (u,v,d) in G.edges(data=True)]
nx.draw(G, pos, node_size = 0.001, width = width)
#nx.draw_networkx_edges(G,pos,edge_labels = edgelabels,
#                width=6)
plt.savefig('TestPicture.png', dpi = 500)