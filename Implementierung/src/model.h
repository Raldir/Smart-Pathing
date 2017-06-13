#ifndef MODEL_H
#define MODEL_H

// the command line options your model may use or global useful variables
struct model_options{
  int verbosity;
  int timesteps;
  // dimensions
  int x;
  int y;
  // useful variables:
  int rank;
  int size;
};

extern struct model_options o;

void model_init();
void model_timestep(int t);
void model_finalize();

#endif
