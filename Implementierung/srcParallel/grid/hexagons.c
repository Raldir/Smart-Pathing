#include <grid/grid-internal.h>

/*
  vocabulary
  neigbors: upper left, upper, upper right, bottom left, bottom, bottom right
  edges: upper left, upper, upper right, bottom left, bottom, bottom right
  vertex: left, upper left, upper right, right, bottom left, bottom right
 */

static int init_grid(grid_t * grid, grid_dim_t dim){
  size_t centers = dim.x * dim.y;

  grid->centers = centers;
  grid->edges = centers * 3;
  grid->vertexs = centers * 2; // every vertex is shared by three hex, responsible for upper two

  return 0;
}

grid_plugin_t grid_hexagons = {
  init_grid
};
