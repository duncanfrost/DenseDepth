SET(targetname MonoLib)


INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

SET(sources
  MonoDepthEstimator_CUDA.cu
  MonoDepthEstimator_CPU.h
  MonoDepthEstimator_CPU.cpp
  )

SET(headers
  MonoDepthEstimator.h
  MonoDepthEstimatorFactory.h
  MonoDepthEstimator_CUDA.h
  Shared/DenseMonoMath.h
  Shared/Intrinsics.h
  Shared/Keyframe.h
  Shared/L0minimization.h
  Shared/Map.h
  Shared/MonoDepthEstimator_Shared.h
  Shared/MonoPyramidLevel.h
  Shared/MonoView.h
  Shared/OptimPyramidLevel.h
  Shared/TrackerData.h
  Shared/DepthData.h
  )

CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
