#include "matlab_helpers.h"

static void analyze_structure_rrg(const mxArray *structure_array_ptr) {
  mwSize total_num_of_elements;
  mwIndex index;
  int number_of_fields, field_index;
  const char *field_name;
  const mxArray *field_array_ptr;

  mexPrintf("\n");
  total_num_of_elements = mxGetNumberOfElements(structure_array_ptr);
  number_of_fields = mxGetNumberOfFields(structure_array_ptr);

  /* Walk through each structure element. */
  for (index = 0; index < total_num_of_elements; index++) {

    /* For the given index, walk through each field. */
    //for (field_index = 0; field_index < number_of_fields; field_index++) {
    for (field_index = 0; field_index < 1; field_index++) {
      mexPrintf("\n\t\t");
      display_subscript(structure_array_ptr, index);
      field_name = mxGetFieldNameByNumber(structure_array_ptr, field_index);
      mexPrintf(".%s\n", field_name);
      field_array_ptr = mxGetFieldByNumber(structure_array_ptr, index,
                                           field_index);
      if (field_array_ptr == NULL) {
        mexPrintf("\tEmpty Field\n");
      } else {
        /* Display a top banner. */
        mexPrintf("------------------------------------------------\n");
        get_characteristics(field_array_ptr);
        analyze_class_rrg(field_array_ptr);
        mexPrintf("\n");
      }
    }
    mexPrintf("\n\n");
  }
}

/* Determine the category (class) of the input array_ptr, and then
 branch to the appropriate analysis routine. */
mxClassID analyze_class_rrg(const mxArray *array_ptr) {
  mxClassID category;

  category = mxGetClassID(array_ptr);

  if (mxIsSparse(array_ptr)) {
    //analyze_sparse(array_ptr);
  } else {
    switch (category) {
      case mxLOGICAL_CLASS:
        fprintf(stderr, "logical\n");
        //analyze_logical(array_ptr);
        break;
      case mxCHAR_CLASS:
        fprintf(stderr, "string\n");
        //analyze_string(array_ptr);
        break;
      case mxSTRUCT_CLASS:
        fprintf(stderr, "structure\n");
        //analyze_structure(array_ptr);
        break;
      case mxCELL_CLASS:
        fprintf(stderr, "cell\n");
        analyze_cell_rrg(array_ptr);
        break;
      case mxUNKNOWN_CLASS:
        mexWarnMsgIdAndTxt("MATLAB:explore:unknownClass", "Unknown class.");
        break;
      default:
        fprintf(stderr, "full\n");
        analyze_full_rrg(array_ptr);
        break;
    }
  }

  return (category);
}

static void analyze_cell_rrg(const mxArray *cell_array_ptr) {
  mwSize total_num_of_cells;
  mwIndex index;
  const mxArray *cell_element_ptr;

  total_num_of_cells = mxGetNumberOfElements(cell_array_ptr);
  mexPrintf("total num of cells = %d\n", total_num_of_cells);
  mexPrintf("\n");

  /* Each cell mxArray contains m-by-n cells; Each of these cells
   is an mxArray. */
  for (index = 0; index < total_num_of_cells; index++) {
    mexPrintf("\n\n\t\tCell Element: ");
    display_subscript(cell_array_ptr, index);
    mexPrintf("\n");
    cell_element_ptr = mxGetCell(cell_array_ptr, index);
    if (cell_element_ptr == NULL) {
      mexPrintf("\tEmpty Cell\n");
    } else {
      /* Display a top banner. */
      mexPrintf("------------------------------------------------\n");
      get_characteristics(cell_element_ptr);
      analyze_class_rrg(cell_element_ptr);
      mexPrintf("\n");
    }
  }
  mexPrintf("\n");
}

static void analyze_full_rrg(const mxArray *numeric_array_ptr) {
  mxClassID category;

  category = mxGetClassID(numeric_array_ptr);
  switch (category) {
    case mxINT8_CLASS:
      fprintf(stderr, "int8\n");
      //analyze_int8(numeric_array_ptr);
      break;
    case mxUINT8_CLASS:
      fprintf(stderr, "uint8\n");
      //analyze_uint8(numeric_array_ptr);
      break;
    case mxINT16_CLASS:
      fprintf(stderr, "int16\n");
      //analyze_int16(numeric_array_ptr);
      break;
    case mxUINT16_CLASS:
      fprintf(stderr, "uint16\n");
      //analyze_uint16(numeric_array_ptr);
      break;
    case mxINT32_CLASS:
      fprintf(stderr, "int32\n");
      //analyze_int32(numeric_array_ptr);
      break;
    case mxUINT32_CLASS:
      fprintf(stderr, "uint32\n");
      //analyze_uint32(numeric_array_ptr);
      break;
    case mxINT64_CLASS:
      fprintf(stderr, "int64\n");
      //analyze_int64(numeric_array_ptr);
      break;
    case mxUINT64_CLASS:
      fprintf(stderr, "uint64\n");
      //analyze_uint64(numeric_array_ptr);
      break;
    case mxSINGLE_CLASS:
      fprintf(stderr, "single\n");
      analyze_single_rrg(numeric_array_ptr);
      break;
    case mxDOUBLE_CLASS:
      fprintf(stderr, "double\n");
      //analyze_double(numeric_array_ptr);
      break;
    default:
      break;
  }
}

static void analyze_single_rrg(const mxArray *array_ptr) {
  float *pr, *pi;
  mwSize total_num_of_elements, index;

  pr = (float *) mxGetData(array_ptr);
  pi = (float *) mxGetImagData(array_ptr);
  total_num_of_elements = mxGetNumberOfElements(array_ptr);

  for (index = 0; index < total_num_of_elements; index++) {
    //mexPrintf("\t");
    //display_subscript(array_ptr, index);
    if (mxIsComplex(array_ptr)) {
      //mexPrintf(" = %g + %gi\n", *pr++, *pi++);
    } else {
      //mexPrintf(" = %g\n", *pr++);
    }
  }
}
