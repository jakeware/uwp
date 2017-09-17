#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mat.h"
#include "mex.h"

static void analyze_structure_rrg(const mxArray *structure_array_ptr);
mxClassID analyze_class_rrg(const mxArray *array_ptr);
static void analyze_cell_rrg(const mxArray *cell_array_ptr);
static void analyze_full_rrg(const mxArray *numeric_array_ptr);
static void analyze_single_rrg(const mxArray *array_ptr);
