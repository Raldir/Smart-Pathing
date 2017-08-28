import networkx as nx
import matplotlib.pyplot as plt
import glob
import cv2
from PIL import Image
import numpy
import os

files = glob.glob('../SmartCarsSequentiell/Output/*.txt')
for file in files:
    G = nx.DiGraph()
    path = os.path.basename(open(str(file), 'r').name).split('.',1)[0]
    #print(os.path.basename(open(str(file), 'r').name).split('.',1)[0])
    pos2 = {}
    labels={}
    f = open('nodes', 'r')
    for line in f:
        arguments = line.split()
    # print(str(int(float(arguments[1]))) + " " + str(int(float(arguments[2]))) + '\n')
        G.add_node(float(arguments[0]), pos = (float(arguments[1]), float(arguments[2])))
        pos2[arguments[0]] = (arguments[1], arguments[2])
        labels[arguments[0]] = arguments[0]
    f.close()
    #print(G.nodes())
    pos= nx.get_node_attributes(G,'pos')



    weights = []
    maximumweights = []

    g = open(str(file), 'r')
    for line in g:
        argumentsweights = line.split()
        weights.append(float(argumentsweights[1]))
        maximumweights.append(int(argumentsweights[2]))
    g.close()

    f = open('edges', 'r')
    counter = 0
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
        G.add_edge(float(argumentsedge[0]), float(argumentsedge[1]), color = color)
        counter+=2
    counter = 1
    f.close()
    f = open('edges', 'r')
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
        G.add_edge(float(argumentsedge[1]), float(argumentsedge[0]), color = color)
        counter+=2


    edges = G.edges()
    colors = [G[u][v]['color'] for u,v in edges]
    nx.draw(G, pos, node_size = 2, edge_size = 2, edge_color = colors)
    #pos2=nx.spring_layout(G)
    nx.draw_networkx_labels(G, pos2, labels, font_size = 9)
    plt.savefig('visofSteps/' + path  + '.png')

filesImage = glob.glob('visofSteps/*.png')
Images = {}
for file in filesImage :
    #print(str(file))
    index = os.path.basename(open(str(file), 'r').name).split('.',1)[0]
    strIndex = str(index)
    Images[strIndex] = Image.open(str(file))

height, width, layers =  numpy.array(Images['1']).shape
video = cv2.VideoWriter("demo3_4.avi", # Filename
                        -1, # Negative 1 denotes manual codec selection. You can make this automatic by defining the "fourcc codec" with "cv2.VideoWriter_fourcc"
                        10, # 10 frames per second is chosen as a demo, 30FPS and 60FPS is more typical for a YouTube video
                        (width,height) # The width and height come from the stats of image1
                        )

for i in range(1,len(filesImage) + 1):
    for j in range (0, 6) :
        video.write(cv2.cvtColor(numpy.array(Images[str(i)]), cv2.COLOR_RGB2BGR))

video.release()
