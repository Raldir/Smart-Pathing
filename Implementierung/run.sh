#!/bin/bash

cd srcParallel
spack load --dependencies mpi
make
cd ..
cd SmartCarsParallel/SmartCarsParallel
sbatch mpi.slurm

