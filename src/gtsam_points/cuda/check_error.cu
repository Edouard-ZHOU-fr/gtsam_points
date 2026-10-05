// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <new>
#include <gtsam_points/cuda/check_error.cuh>

namespace gtsam_points {

void CUDACheckError::operator<<(cudaError_t error) const {
  if (error == cudaSuccess) {
    return;
  }

  if (error == cudaErrorMemoryAllocation) {
    // Out of memory is not a sticky error and the CUDA context is still valid here.
    // Throw before the unallocated buffer is accessed by a kernel (it would cause a sticky cudaErrorIllegalAddress).
    std::cerr << "error: " << cudaGetErrorName(error) << std::endl;
    std::cerr << "     : " << cudaGetErrorString(error) << std::endl;
    cudaGetLastError();  // Reset the error state so that it is not reported again by later API calls
    throw std::bad_alloc();
  }

  const std::string error_name = cudaGetErrorName(error);
  const std::string error_string = cudaGetErrorString(error);

  std::cerr << "warning: " << error_name << std::endl;
  std::cerr << "       : " << error_string << std::endl;
}

CUDACheckError check_error;

}  // namespace gtsam_points