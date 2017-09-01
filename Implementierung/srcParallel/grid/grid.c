#include <assert.h>

#include <grid/grid-internal.h>
#include <util-main.h>


grid_t * grid_init(grid_dim_t dim, grid_type_t type){
  grid_t * grid = safe_malloc(sizeof(grid_t));
  grid->dim = dim;
  grid->type = type;

  assert(dim.x > 0);
  assert(dim.y > 0);

  switch(type){
  case(GRID_RECTANGLES):{
    grid_rectangle.init_grid(grid, dim);
    break;
  }
  case(GRID_HEXAGONS):{
    grid_hexagons.init_grid(grid, dim);
    break;
  }
  case(GRID_TRIANGLES):{
    grid_triangle.init_grid(grid, dim);
    break;
  }
  default:
    assert(0 && "undefined grid");
  }
  return grid;
}

grid_var_t * grid_var_alloc(const grid_t*grid, grid_variable_scope_t scope, const char * name, size_t type_size){
  grid_var_t * var = safe_malloc(sizeof(grid_var_t));
  var->grid = grid;
  var->scope = scope;

  size_t elems;
  switch(scope){
    case(GRID_VAR_SCOPE_EDGE):
      elems = grid->edges;
      break;
    case(GRID_VAR_SCOPE_CENTER):
      elems = grid->centers;
      break;
    case(GRID_VAR_SCOPE_VERTEX):
      elems = grid->vertexs;
      break;
    default:
      assert(0 && "undefined scope");
  };
  var->data = safe_malloc(elems * type_size);

  return var;
}

void grid_var_free(grid_var_t ** var){
  assert((*var) != NULL);

  free(*var);
  *var = NULL;
}

void grid_free(grid_t ** grid){
  assert(*grid != NULL);

  free(*grid);
  *grid = NULL;
}
