#!/bin/bash
	
# Time limit is one minute. See "man sbatch" for other time formats.
#SBATCH --time=1
# Run a total of ten tasks on two nodes (that is, five tasks per node).
#SBATCH --nodes=2
#SBATCH --ntasks=8
# Use "west" partition.
#SBATCH --partition=west
# Output goes to "job.out", error messages to "job.err".
#SBATCH --output=job.out
#SBATCH --error=job.err

. /etc/profile.d/wr-spack.sh
spack load --dependencies mpi

if [ "${SLURM_PARTITION}" = 'abu' ]
then
	export MPICH_NEMESIS_NETMOD=ib
fi

srun hostname
mpiexec ../../srcParallel/simulation
