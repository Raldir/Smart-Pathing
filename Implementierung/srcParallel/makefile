# include_directories()
add_library(papo-utils SHARED grid/grid.c grid/hexagons.c grid/triangles.c grid/rectangles.c option-parser.c util-time.c util-main.c)

target_link_libraries(papo-utils
  ${MPI_C_LIBRARIES}
)

# your executable requires certain source files:
add_executable(my-model.exe main.c model.c)

# link it against useful libraries
target_link_libraries(my-model.exe
  papo-utils
)

# install required libraries
install(TARGETS papo-utils LIBRARY DESTINATION lib)
install(FILES grid/grid.h DESTINATION include/grid/)
