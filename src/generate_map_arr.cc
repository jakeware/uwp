// Copyright 2015 Jake Ware

// INCLUDES
// c system includes

// cpp system includes
#include <vector>
#include <string>

// external includes
#include <H5Cpp.h>

// project includes

void PopulateData(
    const int * dims,
    std::vector<std::vector<std::vector<std::vector<float> > > > & vels,
    std::vector<std::vector<std::vector<std::vector<float> > > > & grid) {
  // grid
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        grid[0][i][j][k] = static_cast<float>(i);  // x
        grid[1][i][j][k] = static_cast<float>(j);  // y
        grid[2][i][j][k] = static_cast<float>(k);  // z
      }
    }
  }

  // vels
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        vels[0][i][j][k] = static_cast<float>(1);  // u
        vels[1][i][j][k] = static_cast<float>(0);  // v
        vels[2][i][j][k] = static_cast<float>(0);  // w
      }
    }
  }

  return;
}

void WriteData(const int * dims, const int dims_size, std::string * path_str,
               std::string * dataset_str, std::string * field_str,
               std::vector<std::vector<std::vector<float> > > * data) {
  // write to hdf5 file
  hid_t file, space, dset;
  herr_t status;
  hsize_t dims_hdf[3];
  dims_hdf[0] = dims[0];
  dims_hdf[1] = dims[1];
  dims_hdf[2] = dims[2];

  // create file name and path
  char file_out[100];
  snprintf(file_out, sizeof(file_out), "%s%s_windgrid_V%s_DX%i_DY%i_DZ%i.h5",
           path_str->c_str(), dataset_str->c_str(), field_str->c_str(), dims[0],
           dims[1], dims[2]);

  file = H5Fcreate(file_out, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  space = H5Screate_simple(dims_size, dims_hdf, NULL);
  dset = H5Dcreate(file, dataset_str->c_str(), H5T_NATIVE_FLOAT, space,
  H5P_DEFAULT,
                   H5P_DEFAULT,
                   H5P_DEFAULT);
  status = H5Dwrite(dset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                    data->data());
  status = H5Dclose(dset);
  status = H5Sclose(space);
  status = H5Fclose(file);

  return;
}

int main(int argc, char **argv) {
  const int dims_size = 3;
  const int dims[dims_size] = { 100, 100, 100 };

  // initialize vels
  std::vector<std::vector<std::vector<std::vector<float> > > > vels;
  vels.resize(dims_size);
  for (int d = 0; d < dims_size; ++d) {
    vels[d].resize(dims[0]);
    for (int i = 0; i < dims[0]; ++i) {
      vels[d][i].resize(dims[1]);
      for (int j = 0; j < dims[1]; ++j) {
        vels[d][i][j].resize(dims[2]);
      }
    }
  }

  // initialize grid
  std::vector<std::vector<std::vector<std::vector<float> > > > grid;
  grid.resize(dims_size);
  for (int d = 0; d < dims_size; ++d) {
    grid[d].resize(dims[0]);
    for (int i = 0; i < dims[0]; ++i) {
      grid[d][i].resize(dims[1]);
      for (int j = 0; j < dims[1]; ++j) {
        grid[d][i][j].resize(dims[2]);
      }
    }
  }

  // create dataset
  PopulateData(dims, vels, grid);

  // dataset details
  std::string dataset_str = "DSM";
  std::string path_str = "./";
  std::vector<std::string> grid_str;
  grid_str.resize(dims_size);
  grid_str[0] = 'x';
  grid_str[1] = 'y';
  grid_str[2] = 'z';
  std::vector<std::string> vels_str;
  vels_str.resize(dims_size);
  vels_str[0] = 'u';
  vels_str[1] = 'v';
  vels_str[2] = 'w';

  // write velocity
  for (int i = 0; i < dims_size; i++) {
    WriteData(dims, dims_size, &path_str, &dataset_str, &vels_str[i], &vels[i]);
  }

  // write grid
  for (int i = 0; i < dims_size; i++) {
    WriteData(dims, dims_size, &path_str, &dataset_str, &grid_str[i], &grid[i]);
  }

  return 0;
}
