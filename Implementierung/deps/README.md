# Content of the deps directory

This directory contains dependencies to third-party code.
Scripts to download code and build (if possible), otherwise open source compatible code can be stored here.

##Needed Library for extracting Edges and Nodes form .osm:


##Needed Libraries for running the Simulation(incl. RoutingTable calculations, without Paralllel BGL):
http://www.boost.org/users/download/
Tested with Boost 1.60, needed libs:
astar_search.hpp
adjacency_list.hpp
random.hpp
Do not forget to update the correct path to the Boost Library to
the makefile in ../srcParallel/makefile

##Additional needed Libraries for running WITH Parallel BGL):
Since the normal Lib does not build the mpi libraries it is
needed to build them manually:
1. run bootstrap.sh
2. add at the bottom of project-config.jam: using mpi ;
3. run .\b2
4. ./b2 stage threading=multi link=shared
Now there should be a folder called stage with all libraries, including
the mpi library and parallel graph library.
Both Libraries and serialization/vector.hpp are required to run the parallel RoutingTable calculations with Parallel BGL.

##Needed Libaries for the Visualisation:
python networkx
python matplotlib
python cv2!!
python PIL

We would have liked to include files to install or download all above mentioned
libaries directly, this will be hopefully done in the near future!
 





