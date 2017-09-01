#ifndef PAPO_GRID_H
#define PAPO_GRID_H

#include <stdlib.h>

/*
 @brief The grid provides a 2D torus world
 */

typedef enum {
  GRID_RECTANGLES,
  GRID_HEXAGONS,
  GRID_TRIANGLES
} grid_type_t;

typedef enum {
  GRID_VAR_SCOPE_CENTER,
  GRID_VAR_SCOPE_EDGE,
  GRID_VAR_SCOPE_VERTEX
} grid_variable_scope_t;

typedef struct {
  int x;
  int y;
} grid_dim_t;

typedef struct {
  grid_dim_t dim;
  grid_type_t type;

  size_t edges;
  size_t centers;
  size_t vertexs;
} grid_t;

typedef struct{
  const grid_t * grid;
  grid_variable_scope_t scope;
  void * data;
} grid_var_t;

#define VAR_DATA(type, var, index) (type*)(var->data)[index]

// initialize a variable along the grid
grid_t * grid_init(grid_dim_t dim, grid_type_t type);
grid_var_t * grid_var_alloc(const grid_t*grid, grid_variable_scope_t scope, const char * name, size_t type_size);
// free the variable
void grid_var_free(grid_var_t ** var);

// free the grid
void grid_free(grid_t ** grid);

#endif
