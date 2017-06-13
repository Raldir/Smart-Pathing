#ifndef UTIL_MAIN_H
#define UTIL_MAIN_H

#include <assert.h> // make use of assert, where possible!
#include <stdio.h>

#define CRITICAL_ERR(msg, ...) critical_error(msg" (file: %s, line: %d)\n", __VA_ARGS__, __FILE__, __LINE__)

#define CHECK_MPI_RET(ret) if (ret != MPI_SUCCESS){ CRITICAL_ERR("Unexpected error in MPI: %d", ret);}

void critical_error(const char * msg, ...);
void * safe_malloc(size_t count);

#endif
