import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()
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
f = open('edges', 'r')
for line in f:
    argumentsedge = line.split()
   # print(str(int(float(arguments[1]))) + " " + str(int(float(arguments[2]))) + '\n')
    G.add_edge(float(argumentsedge[0]), float(argumentsedge[1]))
f.close()

nx.draw(G, pos, node_size = 2)
#pos2=nx.spring_layout(G)
nx.draw_networkx_labels(G, pos2, labels, font_size = 9)
plt.savefig("path.png")