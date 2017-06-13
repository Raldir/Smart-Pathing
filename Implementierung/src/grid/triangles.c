#include <grid/grid-internal.h>

/*
  vocabulary (rotated triangle)
  neigbors: (upper) left, (upper) right, bottom
  edges: (upper) left, (upper) right, bottom
  vertex: (upper) left, (upper) right, bottom

  Upper can be omitted as it is clear from the context.
 */

static int init_grid(grid_t * grid, grid_dim_t dim){
  // we refine it first to a hexagonal structure
  size_t hex_centers = dim.x * dim.y;
  size_t centers = hex_centers * 6;
  grid->centers = centers;
  grid->edges = hex_centers * 3 * 6;
  grid->vertexs = hex_centers * 3; // every vertex is shared by three hex, responsible for (upper) two

  return 0;
}

grid_plugin_t grid_triangle = {
  init_grid
};
