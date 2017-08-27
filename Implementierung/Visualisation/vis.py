import networkx as nx
import matplotlib.pyplot as plt
import glob

files = glob.glob('../SmartCarsSequentiell/Output/*.txt')
totalcount = 0
for file in files:
    G = nx.DiGraph()
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
    plt.savefig('visofSteps/' +str(totalcount)+ ' .png')
    totalcount+=1