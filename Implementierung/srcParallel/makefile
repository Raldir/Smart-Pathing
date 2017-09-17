#TODO: Implement better structure, automatic installation of the boost mpi library

#Please execute spack load --dependencies mpi beforehand
#Pleae use correct path for $LIBS of your boost and built mpi boost, since the size of the library is too big to link it directly


LIBS = /home/aly/boost_1_65_0/

CC = mpic++
SRC = Project1.cpp Edge.cpp Simulation.cpp VertexFileReader.cpp RoutingTable.cpp TrafficLight.cpp ObserverPattern.cpp Spawner.cpp Graph.cpp EdgeFileReader.cpp Vertex.cpp Car.cpp

LIBSL =stage/lib -lboost_serialization -lboost_system -lboost_mpi -lboost_graph_parallel

FLAGS = -Wl,-rpath=$(LIBS)stage/lib


TARGET = simulation

program: $(SRC)
	$(CC) -I$(LIBS) -L$(LIBS)$(LIBSL) $(FLAGS) $(SRC) -o $(TARGET)




