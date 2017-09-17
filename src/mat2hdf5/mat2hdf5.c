#include "mat2hdf5.h"

int mat2hdf5(const char *path, const char *dataset, const char type,
             int verbose, int quiet) {
  //char CurrentPath[50];
  //getcwd(CurrentPath, sizeof(CurrentPath));
  //fprintf(stderr, "path: %s\n", CurrentPath);

  if (type == 'v') {
    vel2hdf5(path, dataset, verbose, quiet);
  } else if (type == 'c') {
    cell2hdf5(path, dataset, verbose, quiet);
  } else if (type == 'w') {
    grid2hdf5(path, dataset, verbose, quiet);
  } else {
    fprintf(stderr, "Error: Unrecognized data type");
    return 1;
  }

  return (0);
}

void vel2hdf5(const char *path, const char *dataset, int verbose, int quiet) {
  char file_in[100];
  strcpy(file_in, path);
  const char *variable = "velocity";
  strcat(file_in, variable);
  strcat(file_in, ".mat");

  MATFile *pmat;
  const char *name;

  if (!quiet) {
    printf("Reading file %s...\n", file_in);
  }

  pmat = matOpen(file_in, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file_in);
    return;
  }

  mxArray *struct_ptr = matGetNextVariable(pmat, &name);
  if (struct_ptr == NULL) {
    printf("Error reading in file %s\n", file_in);
    return;
  }

  //analyze_structure_rrg(pa);

  // get number of elements and fields
  //mwSize num_struct_elems = mxGetNumberOfElements(struct_ptr);
  int num_fields = mxGetNumberOfFields(struct_ptr);

  if (!quiet && verbose) {
    fprintf(stderr, "fields: %i\n", num_fields);
  }

  // loop through field elements
  int i = 0;
  for (i = 0; i < num_fields; i++) {
    const char *field_name = mxGetFieldNameByNumber(struct_ptr, i);
    if (!quiet && verbose) {
      fprintf(stderr, "Variable: %s\n", field_name);
    }

    // find data in struct
    const mxArray *field_ptr = mxGetFieldByNumber(struct_ptr, 0, i);
    if (field_ptr == NULL) {
      fprintf(stderr, "Empty Field\n");
    } else {
      const mxArray *cell_ptr = mxGetCell(field_ptr, 0);
      if (cell_ptr == NULL) {
        fprintf(stderr, "Empty Cell\n");
      } else {
        float *pr = (float *) mxGetData(cell_ptr);
        mwSize num_elems = mxGetNumberOfElements(cell_ptr);
        mwSize num_dims = mxGetNumberOfDimensions(cell_ptr);
        const mwSize *dims = mxGetDimensions(cell_ptr);

        if (!quiet && verbose) {
          fprintf(stderr, "Total Elements: %d\n", (int) num_elems);
          fprintf(stderr, "Total Dimensions: %d\n", (int) num_dims);
          fprintf(stderr, "Dimensions:\n");
          int d = 0;
          for (d = 0; d < (int) num_dims; d++) {
            fprintf(stderr, "%d\t", (int) dims[d]);
          }
          fprintf(stderr, "\n");
        }

        // read data
        int idims[3];
        idims[0] = (int) dims[0];
        idims[1] = (int) dims[1];
        idims[2] = (int) dims[2];
        float * vec_data = (float *) malloc(
            idims[0] * idims[1] * idims[2] * sizeof(float));

        int loc[(const int) num_dims];
        int k = 0;
        for (k = 0; k < num_elems; k++) {
          get_subscript(cell_ptr, k, loc);
          // need to switch x and y indices due to matlab being column major
          vec_data[loc[1] + loc[0] * idims[0] + loc[2] * idims[0] * idims[1]] =
              *pr++;
        }

        WriteData(idims, num_dims, path, dataset, field_name, vec_data);
      }
    }
  }

  mxDestroyArray(struct_ptr);

  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n", file_in);
    return;
  }
  if (!quiet) {
    printf("Done\n\n");
  }
}

void cell2hdf5(const char *path, const char *dataset, int verbose, int quiet) {
  char file_in[100];
  strcpy(file_in, path);
  const char *variable = "celltype";
  strcat(file_in, variable);
  strcat(file_in, ".mat");

  MATFile *pmat;
  const char *name;

  if (!quiet) {
    printf("Reading file %s...\n", file_in);
  }

  pmat = matOpen(file_in, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file_in);
    return;
  }

  mxArray *struct_ptr = matGetNextVariable(pmat, &name);
  if (struct_ptr == NULL) {
    printf("Error reading in file %s\n", file_in);
    return;
  }

  //analyze_structure_rrg(struct_ptr);

  // get number of elements and fields
  //mwSize num_struct_elems = mxGetNumberOfElements(struct_ptr);
  int num_fields = mxGetNumberOfFields(struct_ptr);

  if (!quiet && verbose) {
    fprintf(stderr, "fields: %i\n", num_fields);
  }

  // find data in struct
  const mxArray *cell_ptr = mxGetCell(struct_ptr, 0);
  if (cell_ptr == NULL) {
    fprintf(stderr, "Empty Cell\n");
  } else {
    float *pr = (float *) mxGetData(cell_ptr);
    mwSize num_elems = mxGetNumberOfElements(cell_ptr);
    mwSize num_dims = mxGetNumberOfDimensions(cell_ptr);
    const mwSize *dims = mxGetDimensions(cell_ptr);

    if (!quiet && verbose) {
      fprintf(stderr, "Total Elements: %d\n", (int) num_elems);
      fprintf(stderr, "Total Dimensions: %d\n", (int) num_dims);
      fprintf(stderr, "Dimensions:\n");
      int d = 0;
      for (d = 0; d < (int) num_dims; d++) {
        fprintf(stderr, "%d\t", (int) dims[d]);
      }
      fprintf(stderr, "\n");
    }

    const char * field_name = "t";

    // read data
    int idims[3];
    idims[0] = (int) dims[0];
    idims[1] = (int) dims[1];
    idims[2] = (int) dims[2];
    float * vec_data = (float *) malloc(
        idims[0] * idims[1] * idims[2] * sizeof(float));

    int loc[(const int) num_dims];
    int k = 0;
    for (k = 0; k < num_elems; k++) {
      get_subscript(cell_ptr, k, loc);
      // need to switch x and y indices due to matlab being column major
      vec_data[loc[1] + loc[0] * idims[0] + loc[2] * idims[0] * idims[1]] =
          *pr++;
    }

    WriteData(idims, num_dims, path, dataset, field_name, vec_data);
  }

  mxDestroyArray(struct_ptr);

  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n", file_in);
    return;
  }
  if (!quiet) {
    printf("Done\n\n");
  }
}

void grid2hdf5(const char *path, const char *dataset, int verbose, int quiet) {
  char file_in[100];
  strcpy(file_in, path);
  const char *variable = "windgrid";
  strcat(file_in, variable);
  strcat(file_in, ".mat");

  MATFile *pmat;
  const char *name;

  if (!quiet) {
    printf("Reading file %s...\n", file_in);
  }

  pmat = matOpen(file_in, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file_in);
    return;
  }

  mxArray *struct_ptr = matGetNextVariable(pmat, &name);
  if (struct_ptr == NULL) {
    printf("Error reading in file %s\n", file_in);
    return;
  }

  //analyze_structure_rrg(pa);

  // get number of elements and fields
  //mwSize num_struct_elems = mxGetNumberOfElements(struct_ptr);
  int num_fields = mxGetNumberOfFields(struct_ptr);

  if (!quiet && verbose) {
    fprintf(stderr, "fields: %i\n", num_fields);
  }

  // loop through field elements
  int i = 0;
  for (i = 0; i < num_fields; i++) {
    const char *field_name = mxGetFieldNameByNumber(struct_ptr, i);
    if (!quiet && verbose) {
      fprintf(stderr, "Variable: %s\n", field_name);
    }

    // find data in struct
    const mxArray *field_ptr = mxGetFieldByNumber(struct_ptr, 0, i);
    if (field_ptr == NULL) {
      fprintf(stderr, "Empty Field\n");
    } else {
      float *pr = (float *) mxGetData(field_ptr);
      mwSize num_elems = mxGetNumberOfElements(field_ptr);
      mwSize num_dims = mxGetNumberOfDimensions(field_ptr);
      const mwSize *dims = mxGetDimensions(field_ptr);

      if (!quiet && verbose) {
        fprintf(stderr, "Total Elements: %d\n", (int) num_elems);
        fprintf(stderr, "Total Dimensions: %d\n", (int) num_dims);
        fprintf(stderr, "Dimensions:\n");
        int d = 0;
        for (d = 0; d < (int) num_dims; d++) {
          fprintf(stderr, "%d\t", (int) dims[d]);
        }
        fprintf(stderr, "\n");
      }

      // read data
      int idims[3];
      idims[0] = (int) dims[0];
      idims[1] = (int) dims[1];
      idims[2] = (int) dims[2];
      float * vec_data = (float *) malloc(
          idims[0] * idims[1] * idims[2] * sizeof(float));

      int loc[(const int) num_dims];
      int k = 0;
      for (k = 0; k < num_elems; k++) {
        get_subscript(field_ptr, k, loc);
        // need to switch x and y indices due to matlab being column major
        vec_data[loc[1] + loc[0] * idims[0] + loc[2] * idims[0] * idims[1]] =
            *pr++;
      }

      WriteData(idims, num_dims, path, dataset, field_name, vec_data);
    }
  }

  mxDestroyArray(struct_ptr);

  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n", file_in);
    return;
  }
  if (!quiet) {
    printf("Done\n\n");
  }
}

void get_subscript(const mxArray *array_ptr, mwSize index, int loc[]) {
  mwSize inner, subindex, total, d, q, number_of_dimensions;
  mwSize *subscript;
  const mwSize *dims;

  number_of_dimensions = mxGetNumberOfDimensions(array_ptr);
  subscript = mxCalloc(number_of_dimensions, sizeof(mwSize));
  dims = mxGetDimensions(array_ptr);

//  mexPrintf("(");
  subindex = index;
  for (d = number_of_dimensions - 1;; d--) { /* loop termination is at the end */
    for (total = 1, inner = 0; inner < d; inner++)
      total *= dims[inner];
    subscript[d] = subindex / total;
    subindex = subindex % total;
    if (d == 0) {
      break;
    }
  }

//  for (q = 0; q < number_of_dimensions - 1; q++) {
//    mexPrintf("%d,", subscript[q] + 1);
//  }
//  mexPrintf("%d)\n", subscript[number_of_dimensions - 1] + 1);

  for (q = 0; q < number_of_dimensions; q++) {
    loc[q] = subscript[q];
  }

  mxFree(subscript);
}

void WriteData(const int * dims, const int dims_size, const char * path,
               const char * dataset, const char * field, float * vec_data) {

  // write to hdf5 file
  hid_t file, space, dset;
  herr_t status;
  hsize_t dims_hdf[1];
  dims_hdf[0] = dims[0] * dims[1] * dims[2];

  // create file name and path
  char file_out[100];
  snprintf(file_out, sizeof(file_out), "%s%s_V%s_X%i_Y%i_Z%i.h5", path,
           dataset, field, dims[0], dims[1], dims[2]);

  file = H5Fcreate(file_out, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  space = H5Screate_simple(1, dims_hdf, NULL);
  dset = H5Dcreate(file, dataset, H5T_NATIVE_FLOAT, space,
  H5P_DEFAULT,
                   H5P_DEFAULT,
                   H5P_DEFAULT);
  status = H5Dwrite(dset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                    vec_data);
  status = H5Dclose(dset);
  status = H5Sclose(space);
  status = H5Fclose(file);

  free(vec_data);

  return;
}
