#include <stdarg.h>
#include <stdlib.h>
#include <mpi.h>

#include <util-main.h>

void critical_error(const char * msg, ...){
  int rank;
  MPI_Comm_rank(MPI_COMM_WORLD, & rank);

  printf("rank %d: critical error: ", rank);
  va_list args;
  va_start(args, msg);
  vprintf(msg, args);
  va_end(args);

  MPI_Abort(MPI_COMM_WORLD, 1);
}

void * safe_malloc(size_t count){
  void * data = malloc(count);
  if(data == NULL){
    critical_error("Error in malloc of %ld elems\n", count);
  }
  return data;
}
