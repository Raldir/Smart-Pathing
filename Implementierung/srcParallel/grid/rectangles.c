#include <grid/grid-internal.h>

/*
  vocabulary
  neigbors: left, right, upper, bottom
  edges: left, right, upper, bottom
  vertex: UL, UR, BL, BR (upper left/)
 */

static int init_grid(grid_t * grid, grid_dim_t dim){
  size_t centers = dim.x * dim.y;

  grid->centers = centers;
  grid->edges = centers * 2;
  grid->vertexs = centers; // every one is responsible for its left upper vertex

  return 0;
}

grid_plugin_t grid_rectangle = {
  init_grid
};
