SET(targetname rmd)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

SET(sources
  depthmap.cpp
  check_cuda_device.cu
  depthmap_denoiser.cu
  epipolar_match.cu
  reduction.cu
  reduction_kernels.cu
  seed_check.cu
  seed_init.cu
  seed_matrix.cu
  seed_update.cu
  triangulation.cu
  )

SET(headers
  depthmap.h
  check_cuda_device.cuh
  cuda_exception.cuh
  denoise_device_data.cuh
  depthmap_denoiser.cuh
  device_image.cuh
  helper_vector_types.cuh
  matrix.cuh
  mvs_device_data.cuh
  pinhole_camera.cuh
  reduction.cuh
  se3.cuh
  seed_matrix.cuh
  texture_memory.cuh
  )

# add_library(${targetname} ${sources} ${headers})
CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
