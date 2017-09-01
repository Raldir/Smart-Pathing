#ifndef GRID_INTERNAL_H
#define GRID_INTERNAL_H

#include <grid/grid.h>
// modular programming using function pointers:
typedef struct{
  // initialize the grid itself
  int (*init_grid)(grid_t * grid, grid_dim_t dim);
} grid_plugin_t;

// make the grid plugins known:
extern grid_plugin_t grid_rectangle;
extern grid_plugin_t grid_hexagons;
extern grid_plugin_t grid_triangle;

#endif
