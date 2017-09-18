##############################
# CMakeLists.txt for MonoLib #
##############################

###########################
# Specify the target name #
###########################

SET(targetname MonoLib)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
  MonoDepthEstimator_CUDA.cu
  MonoDepthEstimator_CPU.h
  MonoDepthEstimator_CPU.cpp
  )

SET(headers
  MonoDepthEstimator.h
  MonoDepthEstimatorFactory.h
  MonoDepthEstimator_CUDA.h
  Shared/MonoDepthEstimator_Shared.h
  Shared/MonoView.h
  Shared/MonoPyramidLevel.h
  Shared/OptimPyramidLevel.h
  Shared/Keyframe.h
  Shared/TrackerData.h
  )

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)
