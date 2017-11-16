SET(targetname ImageSource)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

SET(sources
  DepthSource.cpp
  ImageSource.cpp
  PhoneSource.cpp
  TUMSource.cpp
  ARKitSource.cpp
  TUMDepthSource.cpp
  TUMFeatureSource.cpp
  )

SET(headers
  DepthSource.h
  ImageSource.h
  PhoneSource.h
  TUMSource.h
  ARKitSource.h
  TUMDepthSource.h
  TUMFeatureSource.h
  )


# add_library(${targetname} ${sources} ${headers})

CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
