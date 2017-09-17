// Copyright 2015 Jake Ware

// INCLUDES
// c system includes
#include <eigen3/Eigen/Dense>

// cpp system includes
#include <vector>
#include <string>

// external includes
#include <H5Cpp.h>

// project includes

void PopulateData(const int * dims,
                  std::vector<float> & x,
                  std::vector<float> & y,
                  std::vector<float> & z,
                  std::vector<float> & u,
                  std::vector<float> & v,
                  std::vector<float> & w,
                  std::vector<float> & t) {
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        // grid
        x[i + dims[0] * j + dims[0] * dims[1] * k] = static_cast<float>(i);
        y[i + dims[0] * j + dims[0] * dims[1] * k] = static_cast<float>(j);
        z[i + dims[0] * j + dims[0] * dims[1] * k] = static_cast<float>(k);

        // vels
        if (j == 2) {
          u[i + dims[0] * j + dims[0] * dims[1] * k] = 10.0;
        } else {
          u[i + dims[0] * j + dims[0] * dims[1] * k] = -5.0;
        }
        v[i + dims[0] * j + dims[0] * dims[1] * k] = 0.0;
        w[i + dims[0] * j + dims[0] * dims[1] * k] = 0.0;

        // type
        t[i + dims[0] * j + dims[0] * dims[1] * k] = 1.0;
      }
    }
  }




  return;
}

void WriteData(const int * dims, const int dims_size, std::string * path_str,
               std::string * dataset_str, std::string * field_str,
               std::vector<float> & v1) {
  // write to hdf5 file
  hid_t file, space, dset;
  herr_t status;
  hsize_t dims_hdf[1];
  dims_hdf[0] = dims[0] * dims[1] * dims[2];

  // create file name and path
  char file_out[100];
  snprintf(file_out, sizeof(file_out), "%s%s_windgrid_V%s_DX%i_DY%i_DZ%i.h5",
           path_str->c_str(), dataset_str->c_str(), field_str->c_str(), dims[0],
           dims[1], dims[2]);

  file = H5Fcreate(file_out, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  space = H5Screate_simple(1, dims_hdf, NULL);
  dset = H5Dcreate(file, dataset_str->c_str(), H5T_NATIVE_FLOAT, space,
  H5P_DEFAULT,
                   H5P_DEFAULT,
                   H5P_DEFAULT);
  status = H5Dwrite(dset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                    v1.data());
  status = H5Dclose(dset);
  status = H5Sclose(space);
  status = H5Fclose(file);

  return;
}

int main(int argc, char **argv) {
  const int dims_size = 3;
  const int dims[dims_size] = { 3, 3, 4 };

  // initialize vels
  std::vector<float> u;
  u.resize(dims[0] * dims[1] * dims[2]);
  std::vector<float> v;
  v.resize(dims[0] * dims[1] * dims[2]);
  std::vector<float> w;
  w.resize(dims[0] * dims[1] * dims[2]);

  // initialize grid
  std::vector<float> x;
  x.resize(dims[0] * dims[1] * dims[2]);
  std::vector<float> y;
  y.resize(dims[0] * dims[1] * dims[2]);
  std::vector<float> z;
  z.resize(dims[0] * dims[1] * dims[2]);

  // occupancy
  std::vector<float> t;
  t.resize(dims[0] * dims[1] * dims[2]);

  // create dataset
  PopulateData(dims, x, y, z, u, v, w, t);

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

  std::string type_str;
  type_str = 't';

  // write velocity
  WriteData(dims, dims_size, &path_str, &dataset_str, &vels_str[0], u);
  WriteData(dims, dims_size, &path_str, &dataset_str, &vels_str[1], v);
  WriteData(dims, dims_size, &path_str, &dataset_str, &vels_str[2], w);

  // write grid
  WriteData(dims, dims_size, &path_str, &dataset_str, &grid_str[0], x);
  WriteData(dims, dims_size, &path_str, &dataset_str, &grid_str[1], y);
  WriteData(dims, dims_size, &path_str, &dataset_str, &grid_str[2], z);

  // write occupancy
  WriteData(dims, dims_size, &path_str, &dataset_str, &type_str, t);

  return 0;
}
