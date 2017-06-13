// This file is part of papo-model.
//
// papo-model is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// papo-model is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Author: Julian Kunkel

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <mpi.h>

#include <model.h>
#include <util-main.h>
#include <util-time.h>
#include <option-parser.h>

static void init_options(){
  memset(& o, 0, sizeof(o));
  // set the values of the options to sth. different than 0, if necessary
  o.x = 10;
  o.y = 10;
}

// the supported command line options
static option_help options [] = {
  {'x', NULL, "The dimensionality of the grid in X", OPTION_OPTIONAL_ARGUMENT, 'd', & o.x},
  {'y', NULL, "The dimensionality of the grid in Y", OPTION_OPTIONAL_ARGUMENT, 'd', & o.y},
  {'t', "timesteps", "The number of timesteps to simulate", OPTION_REQUIRED_ARGUMENT, 'd', & o.timesteps},
  {'v', "verbose", "Increase the verbosity level", OPTION_FLAG, 'd', & o.verbosity},
  LAST_OPTION
  };



int main(int argc, char ** argv){
  int printhelp = 0;
  init_options();

  MPI_Init(& argc, & argv);
  MPI_Comm_rank(MPI_COMM_WORLD, & o.rank);
  MPI_Comm_size(MPI_COMM_WORLD, & o.size);
  option_parseOptions(argc, argv, options, & printhelp, o.rank == 0);
  if(printhelp != 0){
    if (o.rank == 0){
      printf("\nSynopsis: %s ", argv[0]);
      option_print_help(options, 0);
    }
    MPI_Finalize();
    if(printhelp == 1){
      exit(0);
    }else{
      exit(1);
    }
  }

  if (o.rank == 0){
    printf("Model starttime: ");
    print_current_time();
    printf("\nOptions:\n");
    option_print_current_values(options);
  }

  model_init();

  // measure time for main program
  timer t_start;
  start_timer(& t_start);

  for(int t = 0; t < o.timesteps; t++){
    if(o.verbosity){
      printf("%d: timestep: %d\n", o.rank, t);
    }
    model_timestep(t);
  }

  // end
  double runtime = stop_timer(t_start);
  printf("End runtime: %.3fs endtime: ", runtime);
  print_current_time();
  printf("\n");

  model_finalize();

  MPI_Finalize();
  return 0;
}
