import networkx as nx
import matplotlib.pyplot as plt
import glob
import cv2
from PIL import Image
import numpy
import os

files = glob.glob('Output/*.txt')

G = nx.DiGraph()
f = open('dev/OutputMap/nodes', 'r')
pos2 = {}
labels={}
for line in f:
    arguments = line.split()
    # print(str(int(float(arguments[1]))) + " " + str(int(float(arguments[2]))) + '\n')
    G.add_node(float(arguments[0]), pos = (float(arguments[1]), float(arguments[2])))
    pos2[arguments[0]] = (arguments[1], arguments[2])
    labels[arguments[0]] = arguments[0]
f.close()
#print(G.nodes())
pos= nx.get_node_attributes(G,'pos')
f = open('dev/OutputMap/edges', 'r')
counter = 0
for line in f:
    argumentsedge = line.split()
    G.add_edge(float(argumentsedge[0]), float(argumentsedge[1]), color = ' ', weight = 0, width = 0.2)
    counter+=2
counter = 1
f.close()
f = open('dev/OutputMap/edges', 'r')
for line in f:
    argumentsedge = line.split()
    G.add_edge(float(argumentsedge[1]), float(argumentsedge[0]), color = ' ', weight = 0, width = 0.2)
    counter+=2
nx.draw_networkx_labels(G, pos2, labels, font_size = 0.1)


for file in files:
    path = os.path.basename(open(str(file), 'r').name).split('.',1)[0]
    #print(os.path.basename(open(str(file), 'r').name).split('.',1)[0])
    #edgelabels = {}
    weights = []
    maximumweights = []

    g = open(str(file), 'r')
    for line in g:
        argumentsweights = line.split()
        weights.append(float(argumentsweights[1]))
        maximumweights.append(int(argumentsweights[2]))
    g.close()

    f = open('dev/OutputMap/edges', 'r')
    counter = 0
    for line in f:
        argumentsedge = line.split()
        ratio = weights[counter] / maximumweights[counter]
        color = ' '
        #edgelabels[str(counter)] = str(counter)
        if(ratio < 0.3):
            color = 'g'
        elif(ratio < 0.7):
            color = 'y'
        else:
            color = 'r'
        G[float(argumentsedge[0])][float(argumentsedge[1])] ['color'] = color
        G[float(argumentsedge[0])][float(argumentsedge[1])]['weight'] = counter
        counter+=2
    counter = 1
    f.close()
    f = open('dev/OutputMap/edges', 'r')
    for line in f:
        argumentsedge = line.split()
        ratio = weights[counter] / maximumweights[counter]
        color = ' '
        if(ratio < 0.3):
            color = 'g'
        elif(ratio < 0.7):
            color = 'y'
        else:
            color = 'r'
        G[float(argumentsedge[1])][float(argumentsedge[0])] ['color'] = color
        G[float(argumentsedge[1])][float(argumentsedge[0])]['weight'] = counter
        counter+=2
    edges = G.edges()
    colors = [G[u][v]['color'] for u,v in edges]
    width = [G[u][v]['width'] for u,v in edges]
    edgelabels = nx.get_edge_attributes(G,'weight')
    #edgelabels =  [G[u][v]['weight'] for u,v in edges]
    #edgelabels =[(u,v) for (u,v,d) in G.edges(data=True)]
    #nx.draw_networkx_edges(G,pos,edge_labels = edgelabels,
    #                width=6)
    #nx.draw_networkx_edge_labels(G, pos, edge_labels = edgelabels,  font_size = 0.03)
    nx.draw(G, pos, node_size = 0.05, width = width , edge_color = colors)
    plt.savefig('Visualisation/visofSteps/' + path  + '.png', dpi = 500)
    print(path  + '.png created')