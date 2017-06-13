#include <assert.h>
#include <stdio.h>

#include <grid/grid.h>

int main(){
  grid_dim_t dims = {10, 10};

  grid_t * grid = grid_init(dims, GRID_RECTANGLES);

  printf("centers: %ld vertex: %ld edges: %ld\n", grid->centers, grid->vertexs, grid->edges );
  assert(grid->centers == 100);

  grid_var_t * var;

  var = grid_var_alloc(grid, GRID_VAR_SCOPE_CENTER, "testvar", 10);
  grid_var_free(& var);
  var = grid_var_alloc(grid, GRID_VAR_SCOPE_EDGE, "testvar", 10);
  grid_var_free(& var);
  var = grid_var_alloc(grid, GRID_VAR_SCOPE_VERTEX, "testvar", 10);
  grid_var_free(& var);

  grid_free(& grid);
  return 0;
}
