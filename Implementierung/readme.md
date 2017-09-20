# Smart Pathing

This Project is designed to demonstrate a high performance and parallized version of smart pathing in a directed graph.


## Running the Simulation
To run our simulation, following steps must be done beforehand.

1.Please read the description in deps to download and install all needed dependencies and libraries to run the program.
2. Please make sure to set the correct path to the boost library in the makefile in srcParallel/makefile

You are good to go, and can start the simulation by running the run.sh. The prints will be saved in job.out in SmartCarsParallel/SmartCarsParallel/job.out and the
outputs in ./Output

## Creating Pictures
In case you want to build different Graphs, just run GenerateDataFromMap.bat.
To create pictures of the Simulation output, run DrawPictures.bat. The Pictures are in Visualisation/visOfSteps

## Creating Animation
To create a animation out of the pictures, run CreateMovie.bat 
It will be saved in ./output.mp4