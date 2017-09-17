#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hdf5.h"

#include "mat.h"
#include "mex.h"

// read mat file containing flow estimate
int mat2hdf5(const char *path, const char *dataset, const char type,
             int verbose, int quiet);
void vel2hdf5(const char *path, const char *dataset, int verbose, int quiet);
void cell2hdf5(const char *path, const char *dataset, int verbose, int quiet);
void grid2hdf5(const char *path, const char *dataset, int verbose, int quiet);
void get_subscript(const mxArray *array_ptr, mwSize index, int loc[]);
void WriteData(const int * dims, const int dims_size, const char * path,
               const char * dataset, const char * field, float * vec_xdata);
