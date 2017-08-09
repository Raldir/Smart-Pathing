"""
Parser that extract the edge and vertex information of an OSM document in two seperate files
Partly use of the code of Brandon Martin-Anderson under the BSD License http://github.com/brianw/osmgeocode
"""
import xml.sax
import copy

def read_osm(filename_or_stream, only_roads=True):
    "TODO Nomalisiere Koordinaten der Nodes"
    osm = OSM(filename_or_stream)
    f = open('edges', 'w')
    print(type(osm.ways))
    nodes = []
    for key, edge in osm.ways.items():
        if only_roads and 'highway' not in edge.tags:
            continue
        for i in range (0, len(edge.nds)):
            print(i)
            if i == 0 or i == len(edge.nds) - 1:
                nodes = nodes + [edge.nds[i]]
                f.write(edge.nds[i] + " ")
        f.write('\n')
    f.close()
    f = open('nodes', 'w')
    for key, node in osm.nodes.items():
        print(node.id)
        if node.id in nodes:
            f.write(node.id + " " + str(node.x) + " " + str(node.y) + '\n')
    f.close()

class Node:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.tags = {}
        
class Way:
    def __init__(self, id, osm):
        self.osm = osm
        self.id = id
        self.nds = []
        self.tags = {}
        
    def split(self, dividers):
        # slice the node-array using this nifty recursive function
        def slice_array(ar, dividers):
            for i in range(1,len(ar)-1):
                if dividers[ar[i]]>1:
                    #print "slice at %s"%ar[i]
                    left = ar[:i+1]
                    right = ar[i:]
                    
                    rightsliced = slice_array(right, dividers)
                    
                    return [left]+rightsliced
            return [ar]
            
        slices = slice_array(self.nds, dividers)
        
        # create a way object for each node-array slice
        ret = []
        i=0
        for slice in slices:
            littleway = copy.copy( self )
            littleway.id += "-%d"%i
            littleway.nds = slice
            ret.append( littleway )
            i += 1
            
        return ret
        

class OSM:
    "TODO Stelle auf Tree um, ist effizienter"
    def __init__(self, file):
        nodes = {}
        ways = {}
        superself = self
        
        class OSMHandler(xml.sax.ContentHandler):
            @classmethod
            def setDocumentLocator(self,loc):
                pass
            
            @classmethod
            def startDocument(self):
                pass
                
            @classmethod
            def endDocument(self):
                pass
                
            @classmethod
            def startElement(self, name, attrs):
                if name=='node':
                    self.currElem = Node(attrs['id'], float(attrs['lon']), float(attrs['lat']))
                elif name=='way':
                    self.currElem = Way(attrs['id'], superself)
                elif name=='tag':
                    self.currElem.tags[attrs['k']] = attrs['v']
                elif name=='nd':
                    self.currElem.nds.append( attrs['ref'] )
                
            @classmethod
            def endElement(self,name):
                if name=='node':
                    nodes[self.currElem.id] = self.currElem
                elif name=='way':
                    ways[self.currElem.id] = self.currElem
                
            @classmethod
            def characters(self, chars):
                pass

        xml.sax.parse(file, OSMHandler)
        
        self.nodes = nodes
        self.ways = ways
            
        #count times each node is used
        node_hash = dict.fromkeys(self.nodes.keys(), 0 )
        for way in self.ways.values():
            if len(way.nds) < 2:       #if a way has only one node, delete it out of the osm collection
                del self.ways[way.id]
            else:
                for node in way.nds:
                    node_hash[node] += 1
        new_ways = {}
        for id, way in self.ways.items():
            split_ways = way.split(node_hash)
            for split_way in split_ways:
                new_ways[split_way.id] = split_way
        self.ways = new_ways
     
read_osm('map.osm')        